
#include <algorithm>
#include "esphome/core/log.h"
#include "esphome/components/light/light_state.h"
#include "fastcon_controller.h"
#include "fastcon_light.h"

#ifndef FASTCON_VERSION
#define FASTCON_VERSION "0.3.3-dev"
#endif

namespace esphome {
namespace fastcon {

static const char *const TAG = "fastcon.light";

light::LightTraits FastconLight::get_traits() {
  light::LightTraits t;
  if (this->color_interlock_) {
    if (this->supports_cwww_) {
      t.set_supported_color_modes({light::ColorMode::RGB, light::ColorMode::COLD_WARM_WHITE});
    } else {
      t.set_supported_color_modes({light::ColorMode::RGB, light::ColorMode::WHITE});
    }
  } else {
    if (this->supports_cwww_) {
      t.set_supported_color_modes({light::ColorMode::RGB_COLD_WARM_WHITE});
    } else {
      t.set_supported_color_modes({light::ColorMode::RGB_WHITE});
    }
  }

  if (this->supports_cwww_) {
    t.set_min_mireds(153.0f);
    t.set_max_mireds(500.0f);
  }
  
  return t;
}

void FastconLight::set_member_ids(const std::vector<int32_t> &ids) {
  if (this->mode_ == FASTCON_SINGLE) {
    ESP_LOGW(TAG, "set_members called on light_id %u, which addresses a single light - ignored",
             (unsigned) this->light_id_);
    return;
  }

  std::vector<uint8_t> mask;
  for (int32_t id : ids) {
    if (id < 1 || id > 255) {
      ESP_LOGW(TAG, "Ignoring out-of-range light id %d (valid range 1-255)", (int) id);
      continue;
    }
    const size_t byte = (size_t) (id - 1) / 8;
    if (mask.size() <= byte)
      mask.resize(byte + 1, 0);
    mask[byte] |= 1 << ((id - 1) % 8);
  }

  if (mask == this->members_) {
    ESP_LOGD(TAG, "Membership unchanged, nothing to do");
    return;
  }
  this->members_ = mask;

  // Re-apply the current state to the new set straight away. Without this the change
  // would not take effect until Home Assistant next changed something, and a repeat of
  // an identical command does not reach write_state() at all.
  if (this->last_state_ != nullptr)
    this->write_state(this->last_state_);
}

void FastconLight::apply_observed(bool is_group, uint8_t addr, const std::vector<uint8_t> &ld) {
  if (this->light_state_ == nullptr || ld.empty())
    return;

  // Does this frame address us?
  if (is_group) {
    if (this->mode_ == FASTCON_SINGLE) {
      // Group 0 is the firmware "all lights" group, so it always includes us. For any
      // other group we only know we are a member if we overheard the assignment.
      if (addr != 0 && this->controller_->observed_group_of(this->light_id_) != (int) addr)
        return;
    } else if (this->group_addr_() != addr) {
      return;
    }
  } else {
    if (this->mode_ != FASTCON_SINGLE || this->light_id_ != addr)
      return;
  }

  auto call = this->light_state_->make_call();
  this->fill_call_(call, ld);

  // Remember what this will make write_state() compute, so the resulting call does not
  // put a frame back on the air for a command we merely overheard.
  this->suppress_echo_ = ld;
  call.perform();
}

void FastconLight::fill_call_(light::LightCall &call, const std::vector<uint8_t> &ld) {
  call.set_transition_length(0);

  if (ld.empty() || ld[0] == 0x00) {
    call.set_state(false);
    return;
  }

  call.set_state(true);
  call.set_brightness((ld[0] & 0x7f) / 127.0f);

  if (ld.size() >= 6) {
    const float b = ld[1] / 255.0f, r = ld[2] / 255.0f, g = ld[3] / 255.0f;
    const uint16_t ww = ld[4], cw = ld[5];
    if (ld[1] || ld[2] || ld[3]) {
      call.set_color_mode_if_supported(light::ColorMode::RGB);
      call.set_rgb(r, g, b);
    } else if (ww || cw) {
      // Inverse of get_light_data()'s encoding: byte 4 is warm white, byte 5 cold,
      // and the pair spans 153-500 mireds linearly.
      call.set_color_mode_if_supported(light::ColorMode::COLD_WARM_WHITE);
      // Clamp to the traits range. Floating point puts the ww==0 case a hair under
      // 153, which ESPHome rejects with "Color temperature value 153.00 is out of
      // range [153.0 - 500.0]" - a warning whose numbers look identical because the
      // log rounds what the comparison does not.
      float mireds = 153.0f + (500.0f - 153.0f) * ww / (float) (ww + cw);
      mireds = std::max(153.0f, std::min(500.0f, mireds));
      call.set_color_temperature(mireds);
    }
  }
}

void FastconLight::publish_group_state(uint8_t light_id, const std::vector<uint8_t> &ld) {
  // Only individual entities: a group entity's own state is set by the command that
  // produced this.
  if (this->mode_ != FASTCON_SINGLE)
    return;
  // light_id 0 means "every single light on this mesh". Valid mesh ids start at 1, so 0
  // is free as a sentinel. Used for the hardwired all-lights group, which commands every
  // bulb whether or not the caller happened to list it.
  if (light_id != 0 && this->light_id_ != light_id)
    return;
  if (this->light_state_ == nullptr || ld.empty())
    return;

  auto call = this->light_state_->make_call();
  this->fill_call_(call, ld);

  // Unconditional, not value-matched: the group frame has already gone out, so any
  // transmit this triggers would be a duplicate. A value check would let a rounding
  // step through and put an individual frame on air per member.
  this->suppress_next_write_ = true;
  call.perform();

  ESP_LOGD(TAG, "Group state published onto light %u", (unsigned) light_id);
}

uint8_t FastconLight::group_addr_() const {
  return this->mode_ == FASTCON_SHARED_GROUP ? this->controller_->get_group_slot() : this->light_id_;
}

void FastconLight::write_state(light::LightState *state) {
  if (this->controller_ == nullptr) {
    ESP_LOGW(TAG, "No controller bound; dropping command");
    return;
  }

  this->last_state_ = state;

  const bool is_group = this->mode_ != FASTCON_SINGLE;
  const unsigned addr = is_group ? (unsigned) this->group_addr_() : (unsigned) this->light_id_;

  std::vector<uint8_t> light_bytes;
  auto &values = state->current_values;

  // Determine if it's a "white-only" command
  bool is_white_only = values.get_color_mode() == light::ColorMode::WHITE;

  if (is_white_only) {
    ESP_LOGD(TAG, "Sending white-only command for %s %u", is_group ? "group" : "light", addr);
    light_bytes = this->controller_->get_white_light_data(state);
  } else {
    ESP_LOGD(TAG, "Sending RGB/color command for %s %u", is_group ? "group" : "light", addr);
    light_bytes = this->controller_->get_light_data(state);
  }

  // State we published ourselves after a group command already went out. Transmitting
  // again would just duplicate it, once per member.
  if (this->suppress_next_write_) {
    this->suppress_next_write_ = false;
    ESP_LOGD(TAG, "Not retransmitting group state for %s %u", is_group ? "group" : "light", addr);
    return;
  }

  // If this write is only the echo of a command we overheard from another controller,
  // publishing it was the whole point - putting it back on the air is not.
  if (!this->suppress_echo_.empty()) {
    const bool echo = (light_bytes == this->suppress_echo_);
    this->suppress_echo_.clear();
    if (echo) {
      ESP_LOGD(TAG, "Not rebroadcasting an overheard command for %s %u",
               is_group ? "group" : "light", addr);
      return;
    }
  }

  // Wrap into the inner payload for this address
  std::vector<uint8_t> payload;
  if (is_group) {
    // The app is observed to send a cmd-9 time-sync frame right before and right after
    // a group action - live A/B test of whether that primes the mesh into a more
    // receptive state for the membership-write/group-control frames that follow. No-op
    // if no time_id is configured on the controller.
    this->controller_->send_time_sync();

    // Claim the slot before addressing it - unconditionally, every call (2026-09-03: these
    // bulbs hold exactly one group assignment each, so a bulb shared with any OTHER group_id
    // this controller has since addressed may already have been silently evicted from this
    // one; see FastconController::ensure_group()'s own comment for the confirmed-live
    // incident this fixed).
    this->controller_->ensure_group((uint8_t) addr, this->members_);
    payload = this->controller_->group_control((uint8_t) addr, light_bytes);
  } else {
    payload = this->controller_->single_control(this->light_id_, light_bytes);
  }

  // Queue it for advertisement
  this->controller_->queueCommand(addr, payload);

  if (is_group)
    this->controller_->send_time_sync();

  ESP_LOGD(TAG, "Queued state v%s: %s=%u, payload_len=%d",
           FASTCON_VERSION, is_group ? "group" : "light_id", addr, (int)payload.size());
}

}  // namespace fastcon
}  // namespace esphome
