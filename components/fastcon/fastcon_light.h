
#pragma once

#include <vector>
#include "esphome/core/component.h"
#include "esphome/components/light/light_output.h"

namespace esphome {
namespace fastcon {

class FastconController; // fwd decl

/// How an entity addresses the mesh.
enum FastconAddressMode : uint8_t {
  /// One light, by its mesh id.
  FASTCON_SINGLE = 0,
  /// A group id pinned in the config. Membership is left alone unless `members` is given.
  FASTCON_GROUP = 1,
  /// A named group that borrows the controller's shared slot and defines itself on use.
  FASTCON_SHARED_GROUP = 2,
};

class FastconLight : public Component, public light::LightOutput {
 public:
  FastconLight() = default;
  explicit FastconLight(int light_id) { this->light_id_ = static_cast<uint8_t>(light_id); }

  void set_controller(FastconController *c) { controller_ = c; }
  void set_light_id(uint8_t id) { light_id_ = id; }
  void set_supports_cwww(bool v) { supports_cwww_ = v; }
  void set_color_interlock(bool v) { color_interlock_ = v; }
  void set_mode(FastconAddressMode m) { mode_ = m; }
  void set_members(const std::vector<uint8_t> &mask) { members_ = mask; }

  // LightOutput interface
  light::LightTraits get_traits() override;
  void write_state(light::LightState *state) override;

 protected:
  /// The group id this entity addresses, resolved against the controller when shared.
  uint8_t group_addr_() const;

  FastconController *controller_{nullptr};
  uint8_t light_id_{0};
  bool supports_cwww_{false};
  bool color_interlock_{false};

  FastconAddressMode mode_{FASTCON_SINGLE};

  /// Membership bitmask. Empty when the group is not ours to define.
  std::vector<uint8_t> members_;
};

}  // namespace fastcon
}  // namespace esphome
