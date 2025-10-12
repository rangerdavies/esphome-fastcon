
#include "esphome/core/log.h"
#include "esphome/components/light/light_state.h"
#include "fastcon_light.h"
#include "fastcon_controller.h"
#include "version.h"

namespace esphome {
namespace fastcon {

static const char *const TAG = "fastcon.light";

light::LightTraits FastconLight::get_traits() {
  light::LightTraits t;
  // Always support brightness (master)
  t.set_supported_color_modes({
    supports_cwww_ ? light::ColorMode::RGB_COLD_WARM_WHITE : light::ColorMode::RGB_WHITE
  });
  // Expose min/max mireds if CW/WW present; typical defaults
  if (supports_cwww_) {
    t.set_min_mireds(153.0f); // ~6500K
    t.set_max_mireds(500.0f); // ~2000K-2700K
  }
  return t;
}

static inline uint8_t to8(float v) {
  if (v < 0.0f) v = 0.0f; if (v > 1.0f) v = 1.0f; return static_cast<uint8_t>(v * 255.0f + 0.5f);
}

void FastconLight::write_state(light::LightState *state) {
  const bool on = state->current_values.is_on();

  float r_f=0, g_f=0, b_f=0, cw_f=0, ww_f=0;

  const auto mode = state->current_values.get_color_mode();

  if (mode == light::ColorMode::RGB || mode == light::ColorMode::RGB_WHITE || mode == light::ColorMode::RGB_COLD_WARM_WHITE) {
    // Use helper with WW/CW slots so constant_brightness path matches ESPHome behavior
    state->current_values_as_rgbww(&r_f, &g_f, &b_f, &cw_f, &ww_f, /*constant_brightness=*/false);
    // If device doesn't support CW/WW, zero them; UI may still send some due to traits variant
    if (!supports_cwww_) { cw_f = 0.0f; ww_f = 0.0f; }
  } else if (mode == light::ColorMode::COLD_WARM_WHITE || mode == light::ColorMode::WHITE) {
    // For white-only mode prefer WW over CW for warmer feel when brightness>0
    state->current_values_as_rgbww(&r_f, &g_f, &b_f, &cw_f, &ww_f, /*constant_brightness=*/false);
    r_f = g_f = b_f = 0.0f;
    if (!supports_cwww_) {
      // Map WHITE to plain white via RGB for RGBW devices
      r_f = g_f = b_f = ww_f > 0 ? ww_f : cw_f; // pick some intensity
      cw_f = ww_f = 0.0f;
    }
  } else {
    // Color mode is UNKNOWN. Use last seen non-zero or warm white by default.
    if (has_last_) {
      r_f = last_.r; g_f = last_.g; b_f = last_.b; cw_f = last_.cw; ww_f = last_.ww;
    } else {
      if (supports_cwww_ && default_first_on_ == DefaultFirstOnMode::WARM_WHITE) {
        ww_f = 1.0f; cw_f = 0.0f; r_f = g_f = b_f = 0.0f;
      } else {
        // Fallback default: RGB white
        r_f = g_f = b_f = 1.0f; cw_f = ww_f = 0.0f;
      }
    }
  }

  if (!on) {
    r_f = g_f = b_f = cw_f = ww_f = 0.0f;
  }

  // Cache last for future UNKNOWN cases
  if (on && (r_f>0 || g_f>0 || b_f>0 || cw_f>0 || ww_f>0)) {
    last_ = {r_f, g_f, b_f, cw_f, ww_f};
    has_last_ = true;
  }

  const uint8_t r = to8(r_f), g = to8(g_f), b = to8(b_f);
  const uint8_t cw = to8(cw_f), ww = to8(ww_f);

  auto payload_mode = FastconController::Mode::RGB;
  if ((cw + ww) > 0 && (r + g + b) == 0) payload_mode = FastconController::Mode::WHITE;

  ESP_LOGD(TAG, "Writing state v%s: light_id=%u, on=%d, rgb=(%u,%u,%u), warm=%u, cold=%u, esph_mode=%d",
           FASTCON_VERSION, (unsigned)light_id_, on, r, g, b, (unsigned)ww, (unsigned)cw, (int)mode);

  if (controller_ != nullptr) {
    controller_->queue_light_command(light_id_, on, r, g, b, ww, cw, payload_mode);
  } else {
    ESP_LOGW(TAG, "No controller bound; dropping command");
  }
}

} // namespace fastcon
} // namespace esphome
