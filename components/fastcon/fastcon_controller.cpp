
#include "esphome/core/component_iterator.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include <algorithm>
#include "esphome/components/light/color_mode.h"
#include "esphome/components/light/light_state.h"
// USE_TIME is only defined (and esphome/components/time/*'s sources only added to the
// build) when a `time:` platform is actually configured - guard the include, matching
// core components with an optional time_id (e.g. deep_sleep).
#ifdef USE_TIME
#include "esphome/components/time/real_time_clock.h"
#endif
#include "fastcon_controller.h"
#include "protocol.h"

#ifndef FASTCON_VERSION
#define FASTCON_VERSION "0.3.2-dev"
#endif

namespace esphome {
namespace fastcon {

static const char *const TAG = "fastcon.controller";

void FastconController::queueCommand(uint32_t light_id_, const std::vector<uint8_t> &data) {
  std::lock_guard<std::mutex> lock(queue_mutex_);
  if (queue_.size() >= max_queue_size_) {
    ESP_LOGW(TAG, "Command queue full (size=%d), dropping command for light %d", (int)queue_.size(), (int)light_id_);
    return;
  }
  Command cmd;
  cmd.data = data;
  cmd.timestamp = millis();
  cmd.retries = 0;
  queue_.push(cmd);
  ESP_LOGV(TAG, "Command queued, queue size: %d", (int)queue_.size());
}

void FastconController::clear_queue() {
  std::lock_guard<std::mutex> lock(queue_mutex_);
  std::queue<Command> empty;
  std::swap(queue_, empty);
}

void FastconController::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Fastcon BLE Controller...");
  ESP_LOGCONFIG(TAG, "  Advertisement interval: %d-%d", this->adv_interval_min_, this->adv_interval_max_);
  ESP_LOGCONFIG(TAG, "  Advertisement duration: %dms", this->adv_duration_);
  ESP_LOGCONFIG(TAG, "  Advertisement gap: %dms", this->adv_gap_);
}

void FastconController::loop() {
  const uint32_t now = millis();
  switch (adv_state_) {
    case AdvertiseState::IDLE: {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (queue_.empty()) return;
      Command cmd = queue_.front();
      queue_.pop();

      esp_ble_adv_params_t adv_params = {
          .adv_int_min = adv_interval_min_,
          .adv_int_max = adv_interval_max_,
          .adv_type = ADV_TYPE_NONCONN_IND,
          .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
          .peer_addr = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
          .peer_addr_type = BLE_ADDR_TYPE_PUBLIC,
          .channel_map = ADV_CHNL_ALL,
          .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
      };

      uint8_t adv_data_raw[31] = {0};
      uint8_t adv_data_len = 0;

      // Flags
      adv_data_raw[adv_data_len++] = 2;
      adv_data_raw[adv_data_len++] = ESP_BLE_AD_TYPE_FLAG;
      adv_data_raw[adv_data_len++] = ESP_BLE_ADV_FLAG_BREDR_NOT_SPT | ESP_BLE_ADV_FLAG_GEN_DISC;

      // Manufacturer data
      // Length byte per BLE Core spec: covers AD Type (1) + Company ID (2) + payload,
      // i.e. cmd.data.size() + 3 - NOT + 2. The old +2 declared this AD structure one
      // byte short of its real contents, producing a malformed raw-advertising buffer
      // that esp_ble_gap_config_adv_data_raw() rejects (confirmed live: err=258
      // ESP_ERR_INVALID_ARG on literally the first command processed after boot).
      adv_data_raw[adv_data_len++] = cmd.data.size() + 3;
      adv_data_raw[adv_data_len++] = ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE;
      adv_data_raw[adv_data_len++] = MANUFACTURER_DATA_ID & 0xFF;
      adv_data_raw[adv_data_len++] = (MANUFACTURER_DATA_ID >> 8) & 0xFF;

      // Bounds check - legacy BLE advertising is capped at 31 bytes total, and
      // adv_data_raw is sized to match. All command types (control frames and, since
      // set_group_members() moved to prepare_membership_payload()'s framing, membership
      // writes too) land on exactly 24 wire bytes, so adv_data_len + 24 = 31 fits exactly.
      // This check is defense in depth against a future command type or a wider mask
      // (mask.size() > 1, never confirmed on hardware) growing past that budget - it was
      // added after an earlier bug (a membership frame that really did land at 30 bytes,
      // via the wrong wire framing) had no check here and silently ran memcpy() past the
      // buffer, corrupting adjacent stack memory (very likely `cmd` itself, a stack-local
      // holding a std::vector<uint8_t> whose heap-owning fields sit right next to this
      // array) - confirmed live: the crash (`heap_caps_free ... free() target pointer is
      // outside heap areas`) matched a corrupted vector destructor freeing a smashed
      // pointer. Dropping the command and logging is the safe failure mode if this is
      // ever hit again.
      if (adv_data_len + cmd.data.size() > sizeof(adv_data_raw)) {
        ESP_LOGE(TAG, "Command payload (%zu bytes) does not fit in a legacy BLE "
                      "advertisement (need %u, max %zu) - dropping command instead of "
                      "corrupting memory. See fastcon_controller.cpp's own comment here.",
                 cmd.data.size(), (unsigned) (adv_data_len + cmd.data.size()), sizeof(adv_data_raw));
        return;
      }
      memcpy(&adv_data_raw[adv_data_len], cmd.data.data(), cmd.data.size());
      adv_data_len += cmd.data.size();

      esp_err_t err = esp_ble_gap_config_adv_data_raw(adv_data_raw, adv_data_len);
      if (err != ESP_OK) {
        ESP_LOGW(TAG, "Error setting raw advertisement data (err=%d): %s", err, esp_err_to_name(err));
        return;
      }
      err = esp_ble_gap_start_advertising(&adv_params);
      if (err != ESP_OK) {
        ESP_LOGW(TAG, "Error starting advertisement (err=%d): %s", err, esp_err_to_name(err));
        return;
      }
      adv_state_ = AdvertiseState::ADVERTISING;
      state_start_time_ = now;
      ESP_LOGV(TAG, "Started advertising");
      break;
    }
    case AdvertiseState::ADVERTISING: {
      if (now - state_start_time_ >= adv_duration_) {
        esp_ble_gap_stop_advertising();
        adv_state_ = AdvertiseState::GAP;
        state_start_time_ = now;
        ESP_LOGV(TAG, "Stopped advertising, entering gap period");
      }
      break;
    }
    case AdvertiseState::GAP: {
      if (now - state_start_time_ >= adv_gap_) {
        adv_state_ = AdvertiseState::IDLE;
        ESP_LOGV(TAG, "Gap period complete");
      }
      break;
    }
  }
}

// --- helpers for channel resolution ---
static inline uint8_t to8(float v) {
  if (v < 0.0f) 
      v = 0.0f; 
  if (v > 1.0f) 
      v = 1.0f; 

  return static_cast<uint8_t>(v * 255.0f + 0.5f);
}

static inline bool all_zero(float r, float g, float b, float cw, float ww) {
  return r == 0.0f && g == 0.0f && b == 0.0f && cw == 0.0f && ww == 0.0f;
}

std::vector<uint8_t> FastconController::get_light_data(light::LightState *state) {
  // Protocol: 6 bytes when ON
  // [0] 0x80 | (brightness 0..127)
  // [1] Blue, [2] Red, [3] Green, [4] Warm, [5] Cold
  // When OFF, a single 0x00 byte is returned.

  auto &values = state->current_values;
  const bool is_on = values.is_on();
  if (!is_on) {
    return std::vector<uint8_t>({0x00});
  }

  // Compute final channel levels from current values so brightness/color_brightness are applied.
  float r=0, g=0, b=0, cw=0, ww=0;
  state->current_values_as_rgbww(&r, &g, &b, &cw, &ww, /*constant_brightness=*/false);  // per ESPHome light API

  // If color mode is WHITE on RGBW fixtures (no CW/WW), map to RGB white.
  const auto mode = values.get_color_mode();
  // Heuristic: if traits have a valid CT range, we treat device as supporting CW/WW.
  const bool supports_cwww = state->get_traits().get_min_mireds() > 0.0f;

  if ((mode == light::ColorMode::WHITE || mode == light::ColorMode::COLD_WARM_WHITE) && !supports_cwww) {
    float m = (ww > 0 ? ww : cw);
    r = g = b = m; cw = ww = 0.0f;
  }

  // Fallback for UNKNOWN color mode / zeroed channels: first ON should be warm white for RGBCW, RGB white otherwise.
  if (all_zero(r,g,b,cw,ww)) {
    if (supports_cwww) { ww = 1.0f; /* warm white */ }
    else { r = g = b = 1.0f; }
  }

  // ESPHome's LightColorValues default-constructs cold_white_/warm_white_ to 1.0 each
  // (light_color_values.h), not 0 - so a CWWW entity that's never had a color explicitly
  // set (its first-ever ON, exactly the case for a fresh group entity) reaches here with
  // cw=ww=1.0 rather than tripping the all_zero() fallback above. Sent as-is that's
  // warm=0xFF, cold=0xFF - summing to 510, not the 255 every real captured frame from the
  // app shows (protocol expects warm+cold == 255). Normalize down, preserving the ratio,
  // so a real bulb never sees an out-of-spec pair.
  if (cw + ww > 1.0f) {
    const float total = cw + ww;
    cw /= total;
    ww /= total;
  }

  // Compose payload
  const float blevel = std::min(values.get_brightness() * 127.0f, 127.0f);
  std::vector<uint8_t> light_data = {
      static_cast<uint8_t>(0x80 | static_cast<uint8_t>(blevel)),
      to8(b),  // Blue
      to8(r),  // Red
      to8(g),  // Green
      to8(ww), // Warm
      to8(cw)  // Cold
  };

  return light_data;
}

// special payload for white LED with color_interlock 
std::vector<uint8_t> FastconController::get_white_light_data(light::LightState *state) {
  auto &values = state->current_values;
  const bool is_on = values.is_on();
  if (!is_on) {
    return std::vector<uint8_t>({0x00});
  }

  const float blevel = std::min(values.get_brightness() * 127.0f, 127.0f);
  std::vector<uint8_t> light_data = {
      static_cast<uint8_t>(0x80 | static_cast<uint8_t>(blevel)),
      0,
      0,
      0,
      127, // Warm
      127  // Cold
  };

  return light_data;
}

std::vector<uint8_t> FastconController::single_control(uint32_t light_id_, const std::vector<uint8_t> &light_data) {
  std::vector<uint8_t> result_data(12);
  result_data[0] = 2 | (((0x0FFFFFF & (light_data.size() + 1)) << 4));
  result_data[1] = light_id_;
  std::copy(light_data.begin(), light_data.end(), result_data.begin() + 2);

  // Debug: hex dump with bounded size; our vector_to_hex_string() returns std::vector<char>
  const auto hex_vec = vector_to_hex_string(result_data);           // std::vector<char>
  const std::string hex(hex_vec.begin(), hex_vec.end());            // make a real string
  ESP_LOGD(TAG, "Inner Payload v%s (%zu bytes): %s",
           FASTCON_VERSION, result_data.size(), hex.c_str());

  return this->generate_command(5, light_id_, result_data, true);
}

std::vector<uint8_t> FastconController::group_control(uint8_t group_id, const std::vector<uint8_t> &light_data) {
  std::vector<uint8_t> result_data(12);
  result_data[0] = 3 | (((0x0FFFFFF & (light_data.size() + 3)) << 4));
  result_data[1] = GROUP_MARKER_HI;
  result_data[2] = GROUP_MARKER_LO;
  result_data[3] = group_id;
  std::copy(light_data.begin(), light_data.end(), result_data.begin() + 4);

  const auto hex_vec = vector_to_hex_string(result_data);
  const std::string hex(hex_vec.begin(), hex_vec.end());
  ESP_LOGD(TAG, "Group Payload v%s (%zu bytes): %s",
           FASTCON_VERSION, result_data.size(), hex.c_str());

  return this->generate_command(5, 0, result_data, true);
}

std::vector<uint8_t> FastconController::set_group_members(uint8_t group_id, const std::vector<uint8_t> &mask) {
  // 18 bytes, matching the app's own logged inner payload exactly - confirmed against real
  // BRMesh captures (two live "getPayloadWithInnerRetry"/"send--->"/"calculatedPayload"
  // triples, different masks/nonces, both reproduced byte-for-byte). The apparent 31-byte
  // overflow a previous fix here worked around by truncating to 12 bytes was real, but the
  // truncation was the wrong fix: this frame type was never going through the standard
  // address+CRC wire framing that overflowed. It uses a different envelope entirely - see
  // prepare_membership_payload() in protocol.cpp - which lands at 24 wire bytes regardless,
  // the same as every other command. 18 bytes still leaves room for a 13-byte mask, i.e.
  // 104 lights.
  const size_t frame_len = std::max<size_t>(18, 5 + mask.size());
  std::vector<uint8_t> result_data(frame_len, 0);

  // The length nibble is 4 on every observed frame; the member mask sits outside it.
  result_data[0] = 5 | (4 << 4);
  result_data[1] = group_id;

  // Nonce. Fresh per write - the app never repeats one, so treat it as a replay guard.
  const uint32_t nonce = random_uint32();
  result_data[2] = 1 + ((nonce >> 16) % 3);  // observed range 0x01-0x03
  result_data[3] = (nonce >> 8) & 0xff;
  result_data[4] = nonce & 0xff;

  std::copy(mask.begin(), mask.end(), result_data.begin() + 5);

  const auto hex_vec = vector_to_hex_string(result_data);
  const std::string hex(hex_vec.begin(), hex_vec.end());
  ESP_LOGD(TAG, "Membership Payload v%s (%zu bytes): %s",
           FASTCON_VERSION, result_data.size(), hex.c_str());

  return this->generate_command(5, 0, result_data, true, /*membership_framing=*/true);
}

void FastconController::ensure_group(uint8_t group_id, const std::vector<uint8_t> &mask) {
  if (mask.empty())
    return;  // group is managed elsewhere (id 0, or defined in the app)

  // Always rewrite membership (2026-09-03, per direct confirmation of real hardware
  // behavior: "the lights only hold 1 group assignment, anytime a group is commanded the
  // membership must be rewritten"). This used to skip the write whenever group_masks_
  // already held the same mask for this group_id within membership_ttl_, on the assumption
  // a bulb remembers membership in several different groups at once and only needs
  // reminding once that record goes stale. False on this hardware: each bulb has exactly
  // one group slot, so ANY other group_id's membership write that happens to include this
  // same bulb silently evicts it from this group - with no ack in this protocol, this
  // controller has no way to detect that eviction, and the old group_masks_ freshness check
  // had no way to know a different group_id had since claimed the same bulb. Confirmed live
  // 2026-09-03: Day Light's group 23 (lights 2/4/6, shared with TV Low's groups 20/21 and
  // Evening's shared group_slot) failed to turn off on a repeat dispatch that skipped
  // re-defining membership (still "fresh" under the old membership_ttl_ window) - almost
  // certainly because an intervening command to one of those other group_ids had already
  // reclaimed one or more of the same bulbs. membership_ttl_ and the group_masks_ freshness
  // check are consequently unused now (the config option/setter are kept for backward YAML
  // compatibility - see fastcon_controller.py) - every call now pays the full
  // membership-write-plus-retries cost, matching what real captured app traffic already
  // suggested (it never assumes a bulb remembers a prior group either).
  ESP_LOGD(TAG, "Defining group %u (%zu mask byte(s))", (unsigned) group_id, mask.size());

  // Lights self-select from this broadcast, and a miss silently drops a light from the
  // group, so repeat it the way the app does.
  auto adv_data = this->set_group_members(group_id, mask);
  for (uint8_t i = 0; i < this->membership_retries_; i++)
    this->queueCommand(group_id, adv_data);

  this->group_masks_[group_id] = GroupState{mask, millis()};
}

// Dynamic groups (2026-09-02 night): brightness arrives already on the 0-127 wire scale and
// blue/red/green/warm/cold already on the 0-255 wire scale, computed by the CALLER (an
// `api: actions:` lambda fed by an HA-side Jinja template, see brmesh-bridge.yaml and
// scripts.yaml's living_room_tv_low) rather than here. Deliberate: get_light_data()'s own
// color_temp_kelvin-to-warm/cold conversion depends on light::LightState/LightTraits (mireds
// range, current color mode) that a group with no backing entity simply doesn't have, and
// hand-rolling a second, separate implementation of that conversion in this file would risk
// silently diverging from the one individual/static-group entities already use - producing a
// visibly different color for the exact same nominal Kelvin depending on which dispatch path
// commanded it. Keeping this method dumb (pack whatever bytes it's given) means there is
// exactly one place color math happens for BrMesh commands overall right now: HA-side Jinja,
// auditable and adjustable without a firmware recompile - see
// docs/fastcongroupconfig.md's "Dynamic groups (api action)" section for the exact formula.
void FastconController::dynamic_group_command(uint8_t group_id, const std::vector<uint8_t> &members,
                                                bool state, uint8_t brightness,
                                                uint8_t blue, uint8_t red, uint8_t green,
                                                uint8_t warm, uint8_t cold) {
  // Same bitmask packing as FastconLight::set_member_ids() (fastcon_light.cpp) - bit N of
  // byte K addresses light_id 8K+N+1 - duplicated rather than shared because that method
  // lives on an entity (mutates this->members_, re-applies last_state_) and this path has
  // neither; both independently match docs/fastcongroupconfig.md's documented mask rule.
  //
  // group_id 0 is firmware-owned ("all lights") and never gets a membership write, same
  // rule light.py's _validate_addressing() enforces at compile time for a static entity
  // (`group_id: 0` + `members:` together is a config error there) - enforced here too,
  // defensively, since a dynamic caller has no such compile-time check. A caller wanting
  // group 0 (e.g. living_room_lights_evening, scripts.yaml) can still pass all 6 member
  // ids for its own target_state/believed_state bookkeeping; the mask is simply never
  // computed or written for this one reserved id.
  std::vector<uint8_t> mask;
  if (group_id != 0) {
    for (uint8_t id : members) {
      if (id < 1) {
        ESP_LOGW(TAG, "Ignoring out-of-range dynamic group member id %u (must be >= 1)", (unsigned) id);
        continue;
      }
      const size_t byte = (size_t) (id - 1) / 8;
      if (mask.size() <= byte)
        mask.resize(byte + 1, 0);
      mask[byte] |= 1 << ((id - 1) % 8);
    }
  }

  // Same time-sync bracketing + ensure_group/group_control/queueCommand order as
  // FastconLight::write_state()'s own group path - see that method's own comments
  // (fastcon_light.cpp) for why. ensure_group() itself already no-ops on an empty mask
  // (group 0's own case, per the guard above, and any group_id passed with no members).
  this->send_time_sync();
  this->ensure_group(group_id, mask);

  std::vector<uint8_t> light_data;
  if (!state) {
    light_data = {0x00};
  } else {
    light_data = {
        static_cast<uint8_t>(0x80 | (brightness & 0x7F)),
        blue, red, green, warm, cold,
    };
  }
  auto payload = this->group_control(group_id, light_data);
  this->queueCommand(group_id, payload);
  this->send_time_sync();

  ESP_LOGD(TAG, "Dynamic group command: group=%u members=%zu state=%d brightness=%u payload_len=%d",
           (unsigned) group_id, members.size(), (int) state, (unsigned) brightness, (int) payload.size());
}

void FastconController::send_time_sync() {
#ifdef USE_TIME
  if (this->time_source_ == nullptr) {
    ESP_LOGV(TAG, "No time source configured, skipping time-sync frame");
    return;
  }
  auto now = this->time_source_->now();
  if (!now.is_valid()) {
    ESP_LOGV(TAG, "Time not synced yet, skipping time-sync frame");
    return;
  }

  // cmd 9, [0]=0x89 [1]=0x00 [2..8]=yy mm dd dow hh mm ss, zero-padded to 12 bytes -
  // reverse-engineered from real BRMesh app captures (see docs/fastcongroupconfig.md).
  // ESPHome's day_of_week is Sunday=1..Saturday=7; the app's own frames use ISO
  // (Monday=1..Sunday=7), confirmed against captures where a Wednesday encoded as 3.
  uint8_t dow = now.day_of_week - 1;
  if (dow == 0) dow = 7;

  std::vector<uint8_t> data(12, 0);
  data[0] = 0x89;
  data[1] = 0x00;
  data[2] = now.year % 100;
  data[3] = now.month;
  data[4] = now.day_of_month;
  data[5] = dow;
  data[6] = now.hour;
  data[7] = now.minute;
  data[8] = now.second;

  ESP_LOGD(TAG, "Time-sync %04u-%02u-%02u %02u:%02u:%02u", now.year, now.month, now.day_of_month, now.hour,
           now.minute, now.second);

  this->queueCommand(0, this->generate_command(5, 0, data, true));
#else
  // No `time:` platform anywhere in this build, so time_id could never have been set
  // (its schema requires cv.use_id(time.RealTimeClock)) - time_source_ is always null.
  ESP_LOGV(TAG, "Built without USE_TIME, skipping time-sync frame");
#endif
}

std::vector<uint8_t> FastconController::generate_command(uint8_t n, uint32_t light_id_, const std::vector<uint8_t> &data, bool forward,
                                                            bool membership_framing) {
  static uint8_t sequence = 0;

  // Create command body with header
  std::vector<uint8_t> body(data.size() + 4);
  uint8_t i2 = (light_id_ / 256);

  // Header
  body[0] = (i2 & 0b1111) | ((n & 0b111) << 4) | (forward ? 0x80 : 0);
  body[1] = sequence++;
  if (sequence >= 255) sequence = 1;
  body[2] = this->mesh_key_[3];  // Safe key

  // Copy data
  std::copy(data.begin(), data.end(), body.begin() + 4);

  // Checksum
  uint8_t checksum = 0;
  for (size_t i = 0; i < body.size(); i++) {
    if (i != 3) checksum = checksum + body[i];
  }
  body[3] = checksum;

  // Encrypt header and data
  for (size_t i = 0; i < 4; i++) {
    body[i] = DEFAULT_ENCRYPT_KEY[i & 3] ^ body[i];
  }
  for (size_t i = 0; i < data.size(); i++) {
    body[4 + i] = this->mesh_key_[i & 3] ^ body[4 + i];
  }

  // RF protocol formatting
  if (membership_framing)
    return prepare_membership_payload(body);

  std::vector<uint8_t> addr = {DEFAULT_BLE_FASTCON_ADDRESS.begin(), DEFAULT_BLE_FASTCON_ADDRESS.end()};
  return prepare_payload(addr, body);
}

} // namespace fastcon
} // namespace esphome
