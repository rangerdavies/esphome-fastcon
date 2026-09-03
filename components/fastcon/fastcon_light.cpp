
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

uint8_t FastconLight::group_addr_() const {
  return this->mode_ == FASTCON_SHARED_GROUP ? this->controller_->get_group_slot() : this->light_id_;
}

void FastconLight::write_state(light::LightState *state) {
  if (this->controller_ == nullptr) {
    ESP_LOGW(TAG, "No controller bound; dropping command");
    return;
  }

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

  // Wrap into the inner payload for this address
  std::vector<uint8_t> payload;
  if (is_group) {
    // The app is observed to send a cmd-9 time-sync frame right before and right after
    // a group action - live A/B test of whether that primes the mesh into a more
    // receptive state for the membership-write/group-control frames that follow. No-op
    // if no time_id is configured on the controller.
    this->controller_->send_time_sync();

    // Claim the slot before addressing it. No-ops while the mesh already holds this
    // membership, so a repeated command costs one frame instead of four.
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
