#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace fastcon {

/// Reports which mesh group one bulb says it is in.
///
/// Not derived from anything this component sent - the value comes from that bulb's own
/// heartbeat broadcast (FastconController::handle_heartbeat_), so it is the mesh's answer
/// rather than our record of what we asked for. Those two disagreeing is the entire class of
/// bug this exists to make visible: a membership write that never applied used to leave every
/// piece of state in the system claiming success.
///
/// Publishes on change only, and stays unpublished until the bulb is first heard from - an
/// unknown group reads as "unavailable" in Home Assistant rather than as a plausible-looking
/// 0, which really would mean "in no group".
///
/// Deliberately has no link back to the controller: the Python side registers it, which keeps
/// the header dependency one-way.
class FastconGroupSensor : public sensor::Sensor, public Component {
 public:
  void set_light_id(uint8_t light_id) { this->light_id_ = light_id; }
  uint8_t get_light_id() const { return this->light_id_; }

  void publish_group(uint8_t group_id) {
    if (this->has_state() && (uint8_t) this->get_state() == group_id)
      return;
    this->publish_state(group_id);
  }

  void dump_config() override {
    ESP_LOGCONFIG("fastcon.sensor", "Fastcon group sensor for light %u", (unsigned) this->light_id_);
  }

 protected:
  uint8_t light_id_{0};
};

}  // namespace fastcon
}  // namespace esphome
