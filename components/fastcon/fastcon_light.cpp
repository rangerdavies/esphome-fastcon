
#include "esphome/core/log.h"
#include "esphome/components/light/light_state.h"
#include "fastcon_controller.h"
#include "fastcon_light.h"
#include "version.h"

namespace esphome {
namespace fastcon {

static const char *const TAG = "fastcon.light";

void FastconLight::write_state(light::LightState *state) {
  // Delegate channel resolution & payload building to the controller.
  // This avoids coupling the light output to any controller-specific enums/APIs.

  // Build manufacturer payload for this light state
  std::vector<uint8_t> light_bytes = this->controller_->get_light_data(state);
  std::vector<uint8_t> payload      = this->controller_->single_control(this->light_id_, light_bytes);

  // Queue it for advertisement
  this->controller_->queueCommand(this->light_id_, payload);

  // Optional: lightweight log with version
  ESP_LOGD(TAG, "Queued state v%s: light_id=%u, payload_len=%d", FASTCON_VERSION,
           (unsigned)this->light_id_, (int)payload.size());
}

}  // namespace fastcon
}  // namespace esphome
