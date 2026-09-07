
#include "esphome/core/component_iterator.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include <algorithm>
#include "esp_system.h"
#include "esphome/components/light/color_mode.h"
#include "esphome/components/light/light_state.h"
// USE_TIME is only defined (and esphome/components/time/*'s sources only added to the
// build) when a `time:` platform is actually configured - guard the include, matching
// core components with an optional time_id (e.g. deep_sleep).
#ifdef USE_TIME
#include "esphome/components/time/real_time_clock.h"
#endif
#include "fastcon_controller.h"
#include "fastcon_light.h"
#include "protocol.h"
#include "utils.h"

#ifndef FASTCON_VERSION
#define FASTCON_VERSION "0.3.6-dev"
#endif

namespace esphome {
namespace fastcon {

static const char *const TAG = "fastcon.controller";

void FastconController::queueCommand(uint32_t light_id_, const std::vector<uint8_t> &data, uint8_t repeat,
                                       CommandKind kind, const std::vector<uint8_t> &group_mask) {
  if (repeat == 0)
    repeat = this->command_retries_;

  std::lock_guard<std::mutex> lock(queue_mutex_);

  // Supersede stale queued frames this dispatch makes obsolete (2026-09-07) - closes a
  // backlog window a live queue-depth diagnostic caught directly: a burst of competing
  // dispatches (confirmed live during the post-reflash reboot-recovery mode-cycling storm,
  // but the same thing can happen from any sufficiently dense run of triggers) can queue
  // many SECONDS of frames for the same bulb across different, now-obsolete looks before
  // any of them actually transmit - the bulb then dutifully plays back every stale one in
  // sequence, visible as real, multi-step physical flicker well after HA already believes
  // the final look is applied. Only removes frames still WAITING in queue_; a frame already
  // popped into the active advertise/gap cycle in loop() is untouched, matching "don't touch
  // what's already irrevocably started, only supersede what hasn't gone out yet". Settle
  // pauses (data empty, no target of their own) are deliberately left alone - an orphaned
  // one just adds a harmless short gap, nothing protocol-breaking; see TIME_SYNC_TARGET's
  // own header comment for why light_id_/group_id 0 specifically still needs its own
  // carve-out despite this.
  //
  // Rules, per direct request (2026-09-07, revised same day - see CommandKind's own
  // comment for the live-confirmed bug the revision fixes: a group's membership write and
  // its control frame share a target but are NOT duplicates of each other, and the
  // original "exact same target, unconditional" rule 1 below used to erase the membership
  // write the instant its own control frame was queued a few ms later, every time, before
  // the membership frame could ever transmit - the mesh's OWN group semantics never
  // actually worked, only the +1s individual retransmit fallback ever reached a bulb):
  //  1. A fresh GROUP_MEMBERSHIP write for a group_id clears EVERY still-queued entry for
  //     that SAME group_id, regardless of kind - a membership redefinition makes a
  //     still-queued membership write from an earlier dispatch to the same group pointless
  //     (this one is about to redefine the same thing) AND a still-queued control frame
  //     for that group stale (it was built against whatever membership existed before this
  //     rewrite, which is about to change under it) - both should go, not just one.
  //  2. Otherwise, exact same target AND same kind supersedes - a genuine repeat (e.g. a
  //     second control-frame dispatch for the same group before the first one drained).
  //  3. A fresh GROUP_CONTROL also supersedes a queued INDIVIDUAL command for one of ITS
  //     OWN members. The reverse never happens - an incoming individual command never
  //     supersedes a queued group command, even for a member of that group - since
  //     discarding a queued individual command never drops state for any other light.
  //  4. A fresh GROUP_CONTROL also supersedes a queued DIFFERENT group_id's GROUP_CONTROL
  //     only when this group's membership fully COVERS the queued group's (a strict
  //     superset or equal set) - never on a partial/unrelated overlap. A partial overlap is
  //     left alone: discarding it would drop the queued group's effect on whichever of its
  //     members aren't also in the new group, which nothing here would ever re-address. A
  //     bare membership write never triggers this rule - it carries no light state, so
  //     there is nothing for a queued group's control frame to be made redundant by.
  //  5. A TIME_SYNC frame takes no part in any of this, in either direction. It carries no
  //     light state, so it can never make another frame stale, and it is a component OF the
  //     group dispatch that queued it rather than an independent entry, so nothing that
  //     dispatch queues afterwards may remove it. Without this the two brackets around a
  //     single group dispatch deleted each other under rule 2 - see CommandKind::TIME_SYNC's
  //     own comment (fastcon_controller.h) for the capture that caught it.
  size_t superseded = 0;
  // Short-circuits the whole scan for a TIME_SYNC without re-indenting it (rule 5).
  const bool scan = (kind != CommandKind::TIME_SYNC);
  // Group_ids whose queued CONTROL frame this pass erases via rule 4 below - their sibling
  // GROUP_MEMBERSHIP entry (if still queued) is orphaned by that same erasure and gets swept
  // up in the second pass following this loop. See queueCommand()'s own header comment
  // (fastcon_controller.h) for why: "the queue should process them as a set" - a membership
  // rewrite left behind for a control frame that will never follow it is worse than useless,
  // it needlessly reassigns bulbs to a group nothing is ever going to command.
  std::vector<uint32_t> orphaned_group_ids;
  for (auto it = this->queue_.begin(); scan && it != this->queue_.end();) {
    bool erase = false;
    // Rule 5, the receiving half: a queued TIME_SYNC is never a supersede candidate.
    if (!it->data.empty() && it->kind != CommandKind::TIME_SYNC) {
      if (kind == CommandKind::GROUP_MEMBERSHIP && it->target == light_id_) {
        erase = true;  // rule 1
      } else if (it->target == light_id_ && it->kind == kind) {
        erase = true;  // rule 2
      } else if (kind == CommandKind::GROUP_CONTROL && it->kind == CommandKind::INDIVIDUAL) {
        // rule 3
        if (light_id_ == 0) {
          erase = true;  // group 0 ("all lights") includes every bulb
        } else if (!group_mask.empty() && it->target >= 1) {
          const size_t byte = (size_t) (it->target - 1) / 8;
          if (byte < group_mask.size() && (group_mask[byte] & (1 << ((it->target - 1) % 8))))
            erase = true;
        }
      } else if (kind == CommandKind::GROUP_CONTROL && it->kind == CommandKind::GROUP_CONTROL) {
        // rule 4
        if (light_id_ == 0 && it->target != 0) {
          erase = true;  // group 0 is a superset of every other group by definition
        } else if (light_id_ != 0 && it->target != 0 && !group_mask.empty() && !it->group_mask.empty()) {
          bool superset = true;
          for (size_t byte = 0; byte < it->group_mask.size() && superset; byte++) {
            const uint8_t theirs = it->group_mask[byte];
            const uint8_t ours = byte < group_mask.size() ? group_mask[byte] : 0;
            if ((theirs & ours) != theirs)
              superset = false;
          }
          if (superset)
            erase = true;
        }
        if (erase)
          orphaned_group_ids.push_back(it->target);
      }
    }
    if (erase) {
      it = this->queue_.erase(it);
      superseded++;
    } else {
      ++it;
    }
  }
  if (!orphaned_group_ids.empty()) {
    for (auto it = this->queue_.begin(); it != this->queue_.end();) {
      if (it->kind == CommandKind::GROUP_MEMBERSHIP &&
          std::find(orphaned_group_ids.begin(), orphaned_group_ids.end(), it->target) != orphaned_group_ids.end()) {
        it = this->queue_.erase(it);
        superseded++;
      } else {
        ++it;
      }
    }
  }
  if (superseded > 0) {
    ESP_LOGD(TAG, "Superseded %zu stale queued frame(s) for target %u", superseded, (unsigned) light_id_);
  }

  for (uint8_t i = 0; i < repeat; i++) {
    if (queue_.size() >= max_queue_size_) {
      ESP_LOGW(TAG, "Command queue full (size=%d), dropping command for light %d (sent %d of %d)",
               (int)queue_.size(), (int)light_id_, (int)i, (int)repeat);
      return;
    }
    Command cmd;
    cmd.data = data;
    cmd.target = light_id_;
    cmd.kind = kind;
    cmd.group_mask = group_mask;
    cmd.timestamp = millis();
    cmd.retries = 0;
    queue_.push_back(cmd);

    // Configurable pause between repeats of a membership write (2026-09-07, per direct
    // request "can i increase the resend time between the 3x membership writes?") - only
    // between repeats (not after the last one), and only for GROUP_MEMBERSHIP, not every
    // repeated command generically. Pushed as an inline settle entry rather than calling
    // queue_settle() - that method takes queue_mutex_ itself, which is already held here.
    if (kind == CommandKind::GROUP_MEMBERSHIP && membership_repeat_gap_ms_ > 0 && i + 1 < repeat) {
      if (queue_.size() >= max_queue_size_)
        break;
      Command pause;
      pause.settle_ms = membership_repeat_gap_ms_;  // data stays empty - marks it a pause
      pause.timestamp = millis();
      queue_.push_back(pause);
    }
  }
  ESP_LOGV(TAG, "Command queued x%d, queue size: %d", (int)repeat, (int)queue_.size());
}

void FastconController::queue_settle(uint16_t ms) {
  if (ms == 0)
    return;
  std::lock_guard<std::mutex> lock(queue_mutex_);
  if (queue_.size() >= max_queue_size_)
    return;
  Command cmd;
  cmd.settle_ms = ms;  // data stays empty - that is what marks it a pause
  cmd.timestamp = millis();
  queue_.push_back(cmd);
}

void FastconController::clear_queue() {
  std::lock_guard<std::mutex> lock(queue_mutex_);
  this->queue_.clear();
}

void FastconController::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Fastcon BLE Controller...");
  ESP_LOGCONFIG(TAG, "  Advertisement interval: %d-%d", this->adv_interval_min_, this->adv_interval_max_);
  ESP_LOGCONFIG(TAG, "  Advertisement duration: %dms", this->adv_duration_);
  ESP_LOGCONFIG(TAG, "  Advertisement gap: %dms", this->adv_gap_);
}

// esp_reset_reason() names, for the heartbeat log below - esp_err_to_name()-style helper
// doesn't exist for this enum, and the numeric value alone means nothing without the
// ESP-IDF header open next to it.
static const char *reset_reason_name(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_UNKNOWN: return "UNKNOWN";
    case ESP_RST_POWERON: return "POWERON";
    case ESP_RST_EXT: return "EXT";
    case ESP_RST_SW: return "SW";
    case ESP_RST_PANIC: return "PANIC";
    case ESP_RST_INT_WDT: return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT: return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SDIO: return "SDIO";
    default: return "?";
  }
}

void FastconController::log_heartbeat_() {
  // Low-frequency, deliberately cheap - just a visibility floor so a silent reboot or a
  // resource squeeze mid-session shows up on its own timer instead of requiring a human
  // to notice a boot banner by eye while scanning a long log. esp_reset_reason() reads a
  // register set once at boot and is safe to call repeatedly - it reports the SAME reason
  // for the device's current run every time, not "what just happened".
  ESP_LOGI(TAG, "Heartbeat: uptime=%us free_heap=%u reset_reason=%s queue_size=%zu "
                "pending_retransmits=%zu sniff_drops(no_marker=%u mesh_key=%u checksum=%u)",
           (unsigned) (millis() / 1000), (unsigned) esp_get_free_heap_size(),
           reset_reason_name(esp_reset_reason()), this->get_queue_size(),
           this->pending_retransmits_.size(), (unsigned) this->drop_no_marker_,
           (unsigned) this->drop_mesh_key_mismatch_, (unsigned) this->drop_checksum_);
}

void FastconController::schedule_retransmits(uint16_t target_key, std::function<void()> redo) {
  // A fresh command for this target supersedes anything still pending for it - direct
  // request: "any new state changes supersede any pending 'retransmits'". Resending a now-
  // stale value after the mesh has already been told something new would fight the new
  // command instead of reinforcing it.
  this->pending_retransmits_.erase(
      std::remove_if(this->pending_retransmits_.begin(), this->pending_retransmits_.end(),
                      [target_key](const PendingRetransmit &p) { return p.target_key == target_key; }),
      this->pending_retransmits_.end());

  // Configurable (2026-09-07, per direct request): OFF disables the whole safety net for
  // this target - the erase above already ran, so any now-stale entries are still cleared,
  // there is just nothing new scheduled to replace them.
  if (!this->retransmit_enabled_) return;

  // Left unanchored - loop() anchors each entry's own fire_at the next time it observes the
  // queue idle, rather than fixing it to now + delay_ms here. See this method's own header
  // comment (fastcon_controller.h) for why.
  for (uint32_t delay_ms : this->retransmit_delays_) {
    this->pending_retransmits_.push_back(PendingRetransmit{target_key, delay_ms, false, 0, redo});
  }
}

void FastconController::loop() {
  const uint32_t now = millis();

  if (now - this->last_heartbeat_ms_ >= HEARTBEAT_INTERVAL_MS) {
    this->last_heartbeat_ms_ = now;
    this->log_heartbeat_();
  }

  // Fire any due retransmits first. Each `redo` just re-runs a normal dispatch (queueCommand()
  // and, for a group, ensure_group()), which is safe to call from here - it only ever queues,
  // never blocks. Copy the due ones out before invoking any of them, since invoking one can
  // itself call schedule_retransmits() again (nothing currently does, but a callback re-
  // entering this same vector while it is being iterated would be a use-after-free waiting to
  // happen).
  //
  // Anchored, and gated, on the queue actually being idle (2026-09-07, per direct request:
  // "the retry mechanism should not queue any commands unless the queue is empty" - "the 1 and
  // 5 second retries fire based on the queue empty time") - see schedule_retransmits()'s own
  // header comment (fastcon_controller.h) for why a timer fixed at schedule time can fire
  // while the burst it's protecting is still stuck behind a backlog. "Idle" here means the
  // queue is empty AND nothing is currently mid-transmission (adv_state_ == IDLE) - not merely
  // that nothing is left WAITING, which queue_.empty() alone would report a frame early (the
  // instant the last queued frame is popped into the advertise/gap cycle below, before it has
  // actually gone out over the air).
  if (!this->pending_retransmits_.empty()) {
    bool queue_idle;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      queue_idle = this->queue_.empty();
    }
    queue_idle = queue_idle && this->adv_state_ == AdvertiseState::IDLE;

    // Anchor any not-yet-anchored entry's own deadline from THIS idle moment - the first one
    // observed since it was scheduled, not from schedule time.
    if (queue_idle) {
      for (auto &p : this->pending_retransmits_) {
        if (!p.anchored) {
          p.anchored = true;
          p.fire_at = now + p.delay_ms;
        }
      }
    }

    std::vector<std::function<void()>> due;
    for (auto it = this->pending_retransmits_.begin(); it != this->pending_retransmits_.end();) {
      // Only ever fires when the queue is idle right now too - never piles a retry onto a
      // backlog that built up again in the meantime.
      if (it->anchored && it->fire_at <= now && queue_idle) {
        due.push_back(std::move(it->redo));
        it = this->pending_retransmits_.erase(it);
      } else {
        ++it;
      }
    }
    for (auto &fn : due) fn();
  }

  switch (adv_state_) {
    case AdvertiseState::IDLE: {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (queue_.empty()) return;
      Command cmd = queue_.front();
      queue_.pop_front();

      // A settling pause carries no frame: idle through the gap state instead of
      // advertising, so the bulbs get time to act on what was already sent.
      if (cmd.data.empty()) {
        pending_settle_ = cmd.settle_ms;
        adv_state_ = AdvertiseState::GAP;
        state_start_time_ = now;
        ESP_LOGV(TAG, "Settling for %ums", (unsigned) cmd.settle_ms);
        break;
      }

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
      if (now - state_start_time_ >= (uint32_t) adv_gap_ + pending_settle_) {
        pending_settle_ = 0;
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

// Does this color mode carry this channel at all? Same test as LightColorValues::as_rgb()
// and as_cwww() make before filling their outputs. Spelled with explicit casts rather than
// `mode & cap` so it does not depend on which operator& overload color_mode.h provides.
static inline bool has_cap(light::ColorMode mode, light::ColorCapability cap) {
  return (static_cast<uint8_t>(mode) & static_cast<uint8_t>(cap)) != 0;
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

  // Channel RATIOS, deliberately not state->current_values_as_rgbww(). That helper folds the
  // master brightness into every channel it returns, and gamma-corrects the result - but byte
  // [0] below already carries the brightness, so using it applied the dimmer twice. Confirmed
  // live 2026-09-03: HA brightness 15% on light_id 1 produced 92 00 00 00 01 01, i.e. level 18
  // of 127 (correct) alongside warm=1 cold=1 instead of 128/127, and the bulb read that as off.
  // The double-scaling is (0.149)^2.8 = 0.0048, which lands on 1 after rounding.
  //
  // Gamma goes with it, for two reasons beyond the double-apply. These bytes are nominal
  // channel levels the bulb's own firmware curves, not PWM duty, so ESPHome's 2.8 default is
  // correcting for hardware that is not on this side of the radio. And it is what put the
  // warm/cold pair out of spec at every brightness: gamma(0.3)+gamma(0.7) is 104, where every
  // captured app frame holds warm+cold to 255. Raw ratios sum to 255 by construction.
  //
  // Cost: RGB colors shift, since mid-scale components are no longer pulled down by the curve
  // (green 0.5 was 37, is now 128). That is the same correction, not a separate regression -
  // the bulb applies its own curve to what it receives.
  const auto mode = values.get_color_mode();
  float r = 0, g = 0, b = 0, cw = 0, ww = 0;
  if (has_cap(mode, light::ColorCapability::RGB)) {
    const float cb = values.get_color_brightness();
    r = values.get_red() * cb;
    g = values.get_green() * cb;
    b = values.get_blue() * cb;
  }
  if (has_cap(mode, light::ColorCapability::COLD_WARM_WHITE)) {
    cw = values.get_cold_white();
    ww = values.get_warm_white();
  }

  // If color mode is WHITE on RGBW fixtures (no CW/WW), map to RGB white.
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
      // Rounded, not truncated: fill_call_() decodes this as n/127, so truncation made the
      // round trip lossy (18 decodes to 0.1417, which re-encoded to 17) and every overheard
      // frame looked one step off from what we would have sent - which is exactly what the
      // sniffer's value-matched echo check treats as a genuine new command worth rebroadcasting.
      static_cast<uint8_t>(0x80 | static_cast<uint8_t>(blevel + 0.5f)),
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
      // Rounded, not truncated: fill_call_() decodes this as n/127, so truncation made the
      // round trip lossy (18 decodes to 0.1417, which re-encoded to 17) and every overheard
      // frame looked one step off from what we would have sent - which is exactly what the
      // sniffer's value-matched echo check treats as a genuine new command worth rebroadcasting.
      static_cast<uint8_t>(0x80 | static_cast<uint8_t>(blevel + 0.5f)),
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

  this->note_sent_(result_data);
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

  this->note_sent_(result_data);
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

  this->note_sent_(result_data);
  return this->generate_command(5, 0, result_data, true, /*membership_framing=*/true);
}

void FastconController::ensure_group(uint8_t group_id, const std::vector<uint8_t> &mask) {
  if (mask.empty())
    return;  // group is managed elsewhere (id 0, or defined in the app)

  // Narrow the write to only members NOT already tracked as being in THIS group
  // (2026-09-07, per direct request: "the only purpose to tracking the groupId was to
  // avoid membership writes for a light that was already in a group... if a group set
  // command is received for a different group then the incoming group's id needs to be
  // applied to the light's groupId"). This replaces the unconditional-rewrite rule added
  // 2026-09-03 after a confirmed live incident: a TTL-based freshness cache (membership_ttl_/
  // group_masks_) skipped a rewrite based on elapsed time alone, with no way to know a
  // DIFFERENT group_id's dispatch had, in the meantime, silently reclaimed a shared bulb
  // (each bulb holds exactly one group assignment). This is NOT that cache reborn - it
  // tracks "which group is this light currently in" (observed_light_group_,
  // fastcon_controller.h), and EVERY group dispatch (any group_id, any script) updates that
  // same map for every member it touches, below. So the moment group 21 claims light 4,
  // group 23's own next dispatch immediately sees light 4 is no longer tracked as 23's (its
  // own bit stays set in write_mask) - the exact cross-group-eviction case the 2026-09-03
  // fix was for, still caught, just without paying for a rewrite when nothing has actually
  // changed.
  std::vector<uint8_t> write_mask = mask;
  bool needs_write = false;
  // Configurable (2026-09-07, per direct request) - when skip_tracked_membership_ is
  // false, every member is treated as needing a write, full stop, restoring the
  // pre-2026-09-07 unconditional-rewrite behavior. See set_skip_tracked_membership()'s
  // own comment for why: live testing that night repeatedly showed the skip path (bare
  // control frame, no membership write) reaching 0-2 of 3 members while a forced write
  // reached all 3 every time - the narrowing check below is the thing to disable first
  // when chasing that, before touching anything else.
  if (!this->skip_tracked_membership_) {
    // mask is guaranteed non-empty here (see the early return above) - write_mask stays
    // the full, unnarrowed mask and every member is written, unconditionally.
    needs_write = true;
  } else {
    for (size_t byte = 0; byte < mask.size(); byte++) {
      for (int bit = 0; bit < 8; bit++) {
        if (!(mask[byte] & (1 << bit)))
          continue;
        const uint8_t id = (uint8_t) (byte * 8 + bit + 1);
        if (this->observed_group_of(id) == (int) group_id) {
          write_mask[byte] &= ~(1 << bit);  // already tracked as this group - skip it
        } else {
          needs_write = true;
        }
      }
    }
  }

  if (needs_write) {
    ESP_LOGD(TAG, "Defining group %u (%zu mask byte(s), narrowed from %zu members)",
             (unsigned) group_id, write_mask.size(), mask.size());

    // Lights self-select from this broadcast, and a miss silently drops a light from the
    // group, so repeat it the way the app does.
    auto adv_data = this->set_group_members(group_id, write_mask);

    // Bracket the membership write with settling pauses. Before, so a control frame already
    // in flight to these bulbs is acted on before their group assignment moves under it;
    // after, so the membership has landed before the control frame that follows addresses
    // the group. Without the gaps a two-group split queues membership 22, control 22,
    // membership 23, control 23 back to back at ~60ms a frame, and a bulb that is still
    // chewing on one frame gets its group reassigned before the next arrives. Skipped
    // entirely below when no write is needed - there is nothing to protect the timing of.
    this->queue_settle(this->group_settle_ms_);
    // Pass the membership retry count explicitly: queueCommand defaults to
    // command_retries_, and letting both apply would send this nine times. `mask` here
    // (the group's full intended membership), not `write_mask` (the narrowed wire
    // payload) - the queue's own cross-group-supersede rule (queueCommand()'s own comment)
    // reasons about this group's whole membership, not which subset happened to need an
    // actual wire write this time.
    this->queueCommand(group_id, adv_data, this->membership_retries_, CommandKind::GROUP_MEMBERSHIP, mask);
    this->queue_settle(this->group_settle_ms_);
  } else {
    ESP_LOGD(TAG, "Group %u: every member already tracked as this group - skipping membership write",
             (unsigned) group_id);
  }

  // Record the FULL intended membership as this dispatch's own belief, regardless of
  // whether a write was actually sent for every member - this is what lets a LATER,
  // different group_id's dispatch detect these members need reclaiming (see this method's
  // own comment above).
  for (size_t byte = 0; byte < mask.size(); byte++) {
    for (int bit = 0; bit < 8; bit++) {
      if (mask[byte] & (1 << bit))
        this->observed_light_group_[(uint8_t) (byte * 8 + bit + 1)] = group_id;
    }
  }

  this->group_masks_[group_id] = GroupState{mask, millis()};
}

std::vector<uint8_t> FastconController::queueGroupCommand(uint8_t group_id, const std::vector<uint8_t> &mask,
                                                             const std::vector<uint8_t> &light_data) {
  // Membership half of the set - no-ops on its own (empty mask: group_id 0, or a group_id
  // with no known members yet) exactly as it always has.
  this->ensure_group(group_id, mask);
  // Control half - queued as CommandKind::GROUP_CONTROL, linked to the membership half above
  // by sharing the same group_id target. See queueCommand()'s own comment for how the two
  // are kept superseded together once queued.
  auto payload = this->group_control(group_id, light_data);
  this->queueCommand(group_id, payload, /*repeat=*/0, CommandKind::GROUP_CONTROL, mask);
  return payload;
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

  std::vector<uint8_t> light_data;
  if (!state) {
    light_data = {0x00};
  } else {
    light_data = {
        static_cast<uint8_t>(0x80 | (brightness & 0x7F)),
        blue, red, green, warm, cold,
    };
  }

  // Same time-sync bracketing + queueGroupCommand() order as FastconLight::write_state()'s
  // own group path - see that method's own comments (fastcon_light.cpp) for why.
  // queueGroupCommand() itself already no-ops its membership half on an empty mask (group
  // 0's own case, per the guard above, and any group_id passed with no members).
  //
  // Captured by value, not by reference: this same lambda is also handed to
  // schedule_retransmits() below and can run again 1-5 seconds from now, well after this
  // call's own arguments have gone out of scope.
  auto send_group = [this, group_id, mask, members, light_data, state, brightness]() {
    // Queue depth BEFORE this dispatch's own frames go in (2026-09-07) - reveals a
    // backlog this dispatch is landing behind, not just what it adds. See this file's
    // own header note on last_heartbeat_ms_/log_heartbeat_() for why: at 150ms/frame
    // (adv_duration_+adv_gap_), a queue that's already deep when a new dispatch starts
    // means its own frames sit far longer than this log line's own timestamp implies.
    const size_t queue_size_before = this->get_queue_size();
    this->send_time_sync();
    auto payload = this->queueGroupCommand(group_id, mask, light_data);
    this->send_time_sync();

    // The group frame is on its way; now make the individual entities agree with it, so a
    // reconciler sees per-light state rather than lights it believes are unchanged.
    //
    // Group 0 is the hardwired all-lights group: it needs no membership write and it
    // commands every bulb on the mesh, including any the caller did not list. Publishing
    // only the listed members would leave the rest showing stale state while physically
    // having changed - so publish onto every single-light entity instead, via the 0
    // sentinel. This matters as soon as lights exist that the presets do not enumerate.
    if (group_id == 0)
      this->publish_group_members({0}, light_data);
    else
      this->publish_group_members(members, light_data);

    ESP_LOGD(TAG, "Dynamic group command: group=%u members=%zu state=%d brightness=%u payload_len=%d "
                  "queue_before=%zu queue_after=%zu",
             (unsigned) group_id, members.size(), (int) state, (unsigned) brightness, (int) payload.size(),
             queue_size_before, this->get_queue_size());
  };

  send_group();

  // +1s/+5s retransmit fallback (confirmed sequence, direct request: group command issued
  // -> group transmit -> 1s -> foreach(individual) transmit -> 5s -> foreach(individual)
  // transmit): addresses each member INDIVIDUALLY instead of repeating the group command.
  // Resending the same group command would have the same chance of failing again for the
  // same reason if the original miss was a lost/evicted group membership (ensure_group()
  // now always rewrites it, but that write is itself just another unacknowledged broadcast
  // that can be missed) - single_control() addresses a bulb directly and needs no group
  // membership at all, so it sidesteps that failure mode entirely rather than repeating it.
  // No entity publish here - the group send above already published the correct values;
  // this only concerns whether the bulbs themselves received them.
  //
  // Scheduled ONE PER MEMBER, keyed by that member's own light_id - deliberately NOT keyed
  // by this group_id. This is what makes "last command wins, per bulb" correct even across
  // two DIFFERENT group_ids that happen to share a bulb (e.g. group 50 then group 60,
  // issued back to back, both including the same physical light): schedule_retransmits()
  // already supersedes any existing entry with the same target_key, and here the
  // target_key IS the physical bulb, not whichever group most recently addressed it - so a
  // later command (group OR individual) naturally overwrites an earlier one's pending
  // retransmit for a shared bulb, with no cross-group bookkeeping needed at all. A bulb
  // that group 60 does NOT include keeps group 50's own still-pending retransmit
  // untouched, which is correct too - nothing has told that bulb anything different since.
  for (uint8_t id : members) {
    this->schedule_retransmits((uint16_t) id, [this, id, light_data]() {
      auto payload = this->single_control(id, light_data);
      this->queueCommand(id, payload);
      ESP_LOGD(TAG, "Retransmitting light %u individually", (unsigned) id);
    });
  }
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

  // Once, not command_retries_ times: this carries no state worth re-asserting, and it is
  // already queued twice around every group action.
  //
  // CommandKind::TIME_SYNC, not the INDIVIDUAL default: this frame is a component of
  // whichever group dispatch queued it, not a queue entry standing on its own, so it must
  // neither supersede nor be superseded. As INDIVIDUAL it was both - see that enumerator's
  // own comment (fastcon_controller.h) for the two ways that erased it before it transmitted.
  this->note_sent_(data);
  this->queueCommand(TIME_SYNC_TARGET, this->generate_command(5, 0, data, true), 1,
                     CommandKind::TIME_SYNC);
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

// ---------------------------------------------------------------------------
// Passive sniffer - see fastcon_controller.h for what this can and cannot do.
// ---------------------------------------------------------------------------

void FastconController::publish_group_members(const std::vector<uint8_t> &members,
                                              const std::vector<uint8_t> &light_data) {
  for (uint8_t id : members) {
    for (auto *l : this->lights_)
      l->publish_group_state(id, light_data);
  }
}

int FastconController::observed_group_of(uint8_t light_id) const {
  auto it = this->observed_light_group_.find(light_id);
  return it == this->observed_light_group_.end() ? -1 : (int) it->second;
}

/// Local hex formatter - vector_to_hex_string() takes a non-const reference, which is
/// awkward for the read-only buffers all over the sniffer path.
static std::string sniff_hex(const std::vector<uint8_t> &v) {
  static const char *const H = "0123456789abcdef";
  std::string s;
  s.reserve(v.size() * 2);
  for (uint8_t b : v) {
    s.push_back(H[b >> 4]);
    s.push_back(H[b & 0x0f]);
  }
  return s;
}

#ifdef USE_ESP32_BLE_TRACKER
bool FastconController::parse_device(const ble_device_base::ESPBTDevice &device) {
  if (!this->sniffer_enabled_)
    return false;

  for (auto &md : device.get_manufacturer_datas()) {
    if (md.uuid.get_uuid().len != ESP_UUID_LEN_16)
      continue;
    if (md.uuid.get_uuid().uuid.uuid16 != MANUFACTURER_DATA_ID)
      continue;

    // Raw, before anything is done to it. This is the exact equivalent of the app's own
    // `calculatedPayload` line, so a capture from here can be decoded the same way.
    // address_str() is deprecated (removed in ESPHome 2027.2.0) in favour of writing
    // into a caller-supplied buffer.
    char addr_buf[ble_device_base::ESPBTDevice::MAC_ADDRESS_PRETTY_BUFFER_SIZE];
    ESP_LOGD(TAG, "SNIFF raw  from=%s rssi=%d len=%u wire=%s", device.address_str_to(addr_buf),
             device.get_rssi(), (unsigned) md.data.size(), sniff_hex(md.data).c_str());

    this->handle_sniffed_payload_(md.data);
  }
  return false;  // never claim the device - bluetooth_proxy still wants to see it
}
#endif

void FastconController::handle_heartbeat_(const std::vector<uint8_t> &hb) {
  const uint8_t light_id = hb[5];
  const uint8_t group_id = hb[6];

  const int had = this->observed_group_of(light_id);
  // Ground truth, straight from the bulb. Everything else that writes this map is recording
  // what we ASKED for; this records what actually stuck. Where they disagree the bulb wins,
  // which is the entire point - a membership write that was never applied used to leave the
  // map claiming success with nothing able to contradict it.
  this->observed_light_group_[light_id] = group_id;

  if (had != (int) group_id) {
    ESP_LOGI(TAG, "HEARTBEAT light %u is in group %u (was %s)", (unsigned) light_id,
             (unsigned) group_id, had < 0 ? "unknown" : std::to_string(had).c_str());
  } else {
    ESP_LOGD(TAG, "HEARTBEAT light %u is in group %u", (unsigned) light_id, (unsigned) group_id);
  }
}

void FastconController::handle_sniffed_payload_(const std::vector<uint8_t> &payload) {
  // Bulb status broadcast - test first, because it is none of the things the rest of this
  // function knows how to take apart: 16 bytes, mesh-key XOR only, no whitening and no
  // framing marker, so un-whitening it first would turn it into noise that falls out of the
  // bottom as "no framing marker". Keyed on the decoded marker rather than the raw byte so
  // it does not depend on this mesh's particular key. See handle_heartbeat_()'s own comment
  // (fastcon_controller.h) for the layout and where it came from.
  if (payload.size() == HEARTBEAT_LEN) {
    std::vector<uint8_t> hb(HEARTBEAT_LEN);
    for (size_t i = 0; i < HEARTBEAT_LEN; i++)
      hb[i] = payload[i] ^ this->mesh_key_[i % 4];
    if (hb[0] == HEARTBEAT_MARKER) {
      ESP_LOGD(TAG, "SNIFF heartbeat %s", sniff_hex(hb).c_str());
      this->handle_heartbeat_(hb);
      return;
    }
  }

  // Whitening XORs against a position-keyed stream and is therefore its own inverse,
  // but that stream starts 0xf bytes before the part which goes on the air. Rebuild the
  // offset so the keystream lines up, then take the padding back off.
  // Frames normally arrive whitened, so un-whiten first. But some arrive already plain -
  // membership frames have been seen on air with the a5 5a marker in the clear, which the
  // transmit path cannot produce (verified: whitening our own membership payload yields
  // 461c67..., matching the app's captured wire form). Accept either, by testing for the
  // plaintext markers before spending the transform on it.
  const bool already_plain =
      (payload.size() >= 2 && payload[0] == 0xa5 && payload[1] == 0x5a) ||
      (payload.size() >= 3 && payload[0] == 0x8e && payload[1] == 0xf0 && payload[2] == 0xaa);

  std::vector<uint8_t> pre;
  if (already_plain) {
    pre = payload;
    ESP_LOGD(TAG, "SNIFF plain (arrived un-whitened)");
  } else {
    std::vector<uint8_t> buf(0xf, 0);
    buf.insert(buf.end(), payload.begin(), payload.end());
    WhiteningContext ctx;
    whitening_init(0x25, ctx);
    whitening_encode(buf, ctx);
    pre.assign(buf.begin() + 0xf, buf.end());
  }

  // Un-whitened but still wrapped and still encrypted. Equivalent to nothing the app
  // logs directly, but it is where framing is decided, so log it before deciding.
  ESP_LOGD(TAG, "SNIFF pre  %s", sniff_hex(pre).c_str());

  std::vector<uint8_t> body;
  const char *framing;
  if (pre.size() >= 8 && pre[0] == 0xa5 && pre[1] == 0x5a) {
    // Membership framing: marker then body, no address and no CRC.
    body.assign(pre.begin() + 2, pre.end());
    framing = "membership";
  } else if (pre.size() >= 12 && pre[0] == 0x8e && pre[1] == 0xf0 && pre[2] == 0xaa) {
    // Control framing. 0x8e/0xf0/0xaa are reverse_8() of 0x71/0x0f/0x55; three address
    // bytes follow it and a CRC16 trails.
    body.assign(pre.begin() + 6, pre.end() - 2);
    framing = "control";
  } else {
    this->drop_no_marker_++;
    ESP_LOGD(TAG, "SNIFF drop no framing marker (want a55a or 8ef0aa, got %02x%02x%02x, len %u)",
             pre.size() > 0 ? pre[0] : 0, pre.size() > 1 ? pre[1] : 0, pre.size() > 2 ? pre[2] : 0,
             (unsigned) pre.size());
    return;
  }

  if (body.size() < 5) {
    this->drop_no_marker_++;
    ESP_LOGD(TAG, "SNIFF drop %s body too short (%u bytes)", framing, (unsigned) body.size());
    return;
  }

  for (size_t i = 0; i < 4; i++)
    body[i] ^= DEFAULT_ENCRYPT_KEY[i & 3];
  for (size_t i = 4; i < body.size(); i++)
    body[i] ^= this->mesh_key_[(i - 4) & 3];

  // Equivalent to the app's `send--->` line once decrypted: header then inner payload.
  ESP_LOGD(TAG, "SNIFF body %s framing=%s", sniff_hex(body).c_str(), framing);

  // Two independent checks that this decoded cleanly and belongs to our mesh. A frame
  // from a neighbour's mesh decrypts to noise and fails both.
  if (body[2] != this->mesh_key_[3]) {
    this->drop_mesh_key_mismatch_++;
    ESP_LOGD(TAG, "SNIFF drop mesh key mismatch (body[2]=%02x, ours=%02x)", body[2],
             this->mesh_key_[3]);
    return;
  }
  uint8_t sum = 0;
  for (size_t i = 0; i < body.size(); i++) {
    if (i != 3)
      sum += body[i];
  }
  if (sum != body[3]) {
    this->drop_checksum_++;
    ESP_LOGD(TAG, "SNIFF drop checksum %02x != %02x", sum, body[3]);
    return;
  }

  const std::vector<uint8_t> inner(body.begin() + 4, body.end());
  // Equivalent to the app's `getPayloadWithInnerRetry---> payload:` line.
  ESP_LOGD(TAG, "SNIFF inner %s  n=%u seq=%u fwd=%u", sniff_hex(inner).c_str(),
           (unsigned) ((body[0] >> 4) & 7), (unsigned) body[1],
           (unsigned) ((body[0] & 0x80) ? 1 : 0));

  this->dispatch_observed_(inner);
}

void FastconController::note_sent_(const std::vector<uint8_t> &inner) {
  const uint32_t now = millis();
  this->recent_sent_.push_back(SentFrame{inner, now});
  while (this->recent_sent_.size() > SENT_ECHO_MAX ||
         (!this->recent_sent_.empty() && now - this->recent_sent_.front().at > SENT_ECHO_WINDOW_MS))
    this->recent_sent_.pop_front();
}

bool FastconController::was_sent_by_us_(const std::vector<uint8_t> &inner) {
  const uint32_t now = millis();
  for (auto it = this->recent_sent_.begin(); it != this->recent_sent_.end(); ++it) {
    if (now - it->at <= SENT_ECHO_WINDOW_MS && it->inner == inner)
      return true;
  }
  return false;
}

void FastconController::dispatch_observed_(const std::vector<uint8_t> &inner) {
  if (inner.empty())
    return;

  // The bulbs relay. Every frame this controller sends comes back off six different BLE
  // addresses, one per bulb, for a second or two afterwards. Publishing our own command
  // back onto the entity makes ESPHome re-encode it - and the round trip through its
  // colour model is not lossless, so it lands a step away, transmits, gets relayed, and
  // walks brightness and colour temperature down a rounding staircase. Drop our own.
  if (this->was_sent_by_us_(inner)) {
    ESP_LOGV(TAG, "SNIFF skip our own transmission");
    return;
  }

  const uint8_t cmd = inner[0] & 0x0f;
  const size_t declared = (inner[0] >> 4) & 0x0f;  // count of bytes following inner[0]

  if (cmd == 1 && inner.size() >= 3) {
    // Group assignment. Not a state change in itself, but it records which group a bulb
    // is in, which is what lets a sniffed group command reach individual entities.
    //
    // Membership-hijack alert (2026-09-07, added while chasing physical bulb flicker
    // that left no other trace): if this is one of OUR registered lights (not some
    // other bulb entirely) and its observed group is changing to something other than
    // what we last saw, log it at WARN - a bulb's own membership assignment changing
    // out from under us, from a source other than our own last write, is exactly the
    // shared-group-id hijack scripts.yaml's own state-management gate was fixed for
    // (see apply_living_room_light_targets's "Shared-group override" comment) - this
    // makes it visible in real time instead of requiring after-the-fact log
    // reconstruction like the Day Light incident that fix was built from.
    auto prior = this->observed_light_group_.find(inner[1]);
    if (prior != this->observed_light_group_.end() && prior->second != inner[2]) {
      for (auto *l : this->lights_) {
        if (l->owns_mesh_id(inner[1])) {
          ESP_LOGW(TAG, "Membership change: light %u moved from group %u to group %u",
                   (unsigned) inner[1], (unsigned) prior->second, (unsigned) inner[2]);
          break;
        }
      }
    }
    this->observed_light_group_[inner[1]] = inner[2];
    ESP_LOGD(TAG, "Observed: light %u assigned to group %u", (unsigned) inner[1], (unsigned) inner[2]);
    return;
  }

  // The declared length matters: light_data is padded with zeros out to 12 bytes, and
  // reading the padding would turn a plain on/off into a full colour command.
  if (cmd == 2 && declared >= 1 && inner.size() >= 1 + declared) {
    std::vector<uint8_t> ld(inner.begin() + 2, inner.begin() + 1 + declared);
    ESP_LOGD(TAG, "Observed: light %u, %u data byte(s)", (unsigned) inner[1], (unsigned) ld.size());
    for (auto *l : this->lights_)
      l->apply_observed(false, inner[1], ld);
    return;
  }

  if (cmd == 3 && declared >= 3 && inner.size() >= 1 + declared && inner[1] == GROUP_MARKER_HI &&
      inner[2] == GROUP_MARKER_LO) {
    std::vector<uint8_t> ld(inner.begin() + 4, inner.begin() + 1 + declared);
    ESP_LOGD(TAG, "Observed: group %u, %u data byte(s)", (unsigned) inner[3], (unsigned) ld.size());
    for (auto *l : this->lights_)
      l->apply_observed(true, inner[3], ld);
    return;
  }

  // cmd 5 (membership bitmask) and cmd 9 (time sync) carry no state worth publishing.
}

} // namespace fastcon
} // namespace esphome
