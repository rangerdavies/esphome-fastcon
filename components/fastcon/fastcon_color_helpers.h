#pragma once
#include "esphome/components/light/light_state.h"
#include "version.h"

namespace esphome { namespace fastcon {

// Resolve final channel levels (0..1) from LightState current values.
// - Uses current_values helpers so brightness/color_brightness are applied
// - If color mode is UNKNOWN, falls back to the last non-zero cached values (if provided)
//   or to a sensible default: Warm White for RGBCW, RGB white for RGBW.
// Parameters:
//   state: LightState*
//   supports_cwww: true for RGBCW, false for RGBW
//   first_on_warm_white: if true and supports_cwww, first ON defaults to warm white
//   last: optional pointer to 5 floats storing last r,g,b,cw,ww (can be null)
//   out: rf,gf,bf,cwf,wwf
inline void resolve_fastcon_channels(
  esphome::light::LightState &st,
  bool supports_cwww,
  bool first_on_warm_white,
  const float *last_levels, // nullptr or array[5]
  float &rf, float &gf, float &bf, float &cwf, float &wwf
) {
  using namespace esphome::light;
  rf=gf=bf=cwf=wwf=0.0f;
  const auto &cv = st.current_values;
  const auto mode = cv.get_color_mode();

  if (mode == ColorMode::RGB || mode == ColorMode::RGB_WHITE || mode == ColorMode::RGB_COLD_WARM_WHITE) {
    st.current_values_as_rgbww(&rf, &gf, &bf, &cwf, &wwf, /*constant_brightness=*/false);
    if (!supports_cwww) { cwf = 0.0f; wwf = 0.0f; }
  } else if (mode == ColorMode::COLD_WARM_WHITE || mode == ColorMode::WHITE) {
    st.current_values_as_rgbww(&rf, &gf, &bf, &cwf, &wwf, /*constant_brightness=*/false);
    rf = gf = bf = 0.0f;
    if (!supports_cwww) {
      // Map WHITE to RGB white intensity for RGBW devices
      rf = gf = bf = (wwf > 0 ? wwf : cwf);
      cwf = wwf = 0.0f;
    }
  } else {
    // UNKNOWN
    if (last_levels != nullptr) {
      rf = last_levels[0]; gf = last_levels[1]; bf = last_levels[2]; cwf = last_levels[3]; wwf = last_levels[4];
    }
    if (rf==0 && gf==0 && bf==0 && cwf==0 && wwf==0) {
      if (supports_cwww && first_on_warm_white) { wwf = 1.0f; }
      else { rf = gf = bf = 1.0f; }
    }
  }
}

inline uint8_t to8(float v) {
  if (v < 0.0f) v = 0.0f; if (v > 1.0f) v = 1.0f; return static_cast<uint8_t>(v * 255.0f + 0.5f);
}

}} // namespace
