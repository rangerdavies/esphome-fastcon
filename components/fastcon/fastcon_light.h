#pragma once

#include "esphome/core/component.h"
#include "esphome/components/light/light_output.h"

namespace esphome {
namespace fastcon {

enum class DefaultFirstOnMode {
  RGB_WHITE,
  WARM_WHITE
};

struct LastLevels {
  float r{0}, g{0}, b{0}, cw{0}, ww{0};
};

class FastconController; // fwd decl

class FastconLight : public light::LightOutput {
 public:
  void set_controller(FastconController *c) { controller_ = c; }
  void set_light_id(uint8_t id) { light_id_ = id; }
  void set_supports_cwww(bool v) { supports_cwww_ = v; }
  void set_default_first_on(DefaultFirstOnMode m) { default_first_on_ = m; }

  // LightOutput overrides
  light::LightTraits get_traits() override;
  void write_state(light::LightState *state) override;

 protected:
  FastconController *controller_{nullptr};
  uint8_t light_id_{0};
  bool supports_cwww_{false};
  DefaultFirstOnMode default_first_on_{DefaultFirstOnMode::WARM_WHITE};
  bool has_last_{false};
  LastLevels last_{};
};

} // namespace fastcon
} // namespace esphome
