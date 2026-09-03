
#pragma once

#include <vector>
#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/light/light_output.h"
#include "esphome/components/light/light_state.h"

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

  /// Membership: a list of mesh light ids, packed here and applied at once. This is the
  /// only way to set it - the entity declares itself a group in YAML but never its
  /// members, which arrive from Home Assistant via the fastcon.set_members action.
  /// Ids outside 1..255 are skipped with a warning. An empty list leaves the group
  /// undefined, which makes the entity a no-op rather than addressing a stale set.
  void set_member_ids(const std::vector<int32_t> &ids);

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

  /// Last state written, so a membership change can be re-applied to the new set
  /// immediately instead of waiting for the next command.
  light::LightState *last_state_{nullptr};
};

/// Redefine a group entity's membership at runtime, e.g. from a Home Assistant service.
template<typename... Ts> class SetMembersAction : public Action<Ts...> {
 public:
  explicit SetMembersAction(FastconLight *parent) : parent_(parent) {}
  TEMPLATABLE_VALUE(std::vector<int32_t>, members)

  void play(Ts... x) override { this->parent_->set_member_ids(this->members_.value(x...)); }

 protected:
  FastconLight *parent_;
};

}  // namespace fastcon
}  // namespace esphome
