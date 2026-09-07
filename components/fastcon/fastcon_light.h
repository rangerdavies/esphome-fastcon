
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

  /// The LightState this output backs, wired up in light.py so the sniffer can publish
  /// onto it before any command has ever been sent through write_state().
  void set_light_state(light::LightState *s) { light_state_ = s; }

  /// Publish the state a group command just set onto this entity, WITHOUT transmitting
  /// anything. Called for each member after a group frame goes out, so the individual
  /// entities in Home Assistant match what the group was told - a reconciler can then act
  /// on them per-light. No-op unless this entity is that single light.
  void publish_group_state(uint8_t light_id, const std::vector<uint8_t> &light_data);

  /// A command from another controller (the phone app, a scene switch) was overheard.
  /// Publishes it onto this entity if it addresses us. `addr` is a light_id when
  /// `is_group` is false, otherwise a group id.
  void apply_observed(bool is_group, uint8_t addr, const std::vector<uint8_t> &light_data);

  /// True if this is a single-light entity (not a group) whose own mesh light_id is
  /// `id`. Used by the controller's membership-hijack alert (2026-09-07) to check a
  /// sniffed group-assignment frame against our own registered lights without exposing
  /// light_id_/mode_ themselves - those stay protected.
  bool owns_mesh_id(uint8_t id) const { return this->mode_ == FASTCON_SINGLE && this->light_id_ == id; }

  // LightOutput interface
  light::LightTraits get_traits() override;
  void write_state(light::LightState *state) override;

 protected:
  /// Set the values in `light_data` on a pending LightCall. Shared by the sniffer and by
  /// group-member propagation, so both interpret the wire bytes identically.
  void fill_call_(light::LightCall &call, const std::vector<uint8_t> &ld);

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

  light::LightState *light_state_{nullptr};

  /// Unconditional one-shot suppression for a write_state() we know is not a new
  /// command: either state we published ourselves after a group command, or state
  /// applied from a sniffed frame (apply_observed()) - own delayed self-relay or a
  /// genuinely foreign controller. Deliberately not value-matched (2026-09-07, was
  /// briefly split into a value-matched suppress_echo_ for the apply_observed() case
  /// specifically): the round trip through ESPHome's colour model is not lossless, so a
  /// value check lets a rounding-step difference through and re-transmits it for real -
  /// confirmed live as a slow color_temp_kelvin drift, one rounding step per sniffed
  /// self-relay, compounding fastest during TV Low's high dispatch frequency.
  bool suppress_next_write_{false};
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
