#pragma once

#include <queue>
#include <map>
#include <mutex>
#include <vector>
#include "esphome/core/component.h"
#include "esphome/components/esp32_ble_server/ble_server.h"

namespace esphome
{
    namespace time
    {
        class RealTimeClock;
    }

    namespace fastcon
    {

        class FastconController : public Component
        {
        public:
            FastconController() = default;

            void setup() override;
            void loop() override;

            std::vector<uint8_t> get_light_data(light::LightState *state);
            std::vector<uint8_t> get_white_light_data(light::LightState *state);
            std::vector<uint8_t> single_control(uint32_t addr, const std::vector<uint8_t> &light_data);
            std::vector<uint8_t> group_control(uint8_t group_id, const std::vector<uint8_t> &light_data);
            std::vector<uint8_t> set_group_members(uint8_t group_id, const std::vector<uint8_t> &mask);

            /// Write the membership of `group_id` unless the mesh already holds it and it is fresh.
            void ensure_group(uint8_t group_id, const std::vector<uint8_t> &mask);

            /// Queue a cmd-9 time-sync frame, matching the app's own habit of sending one right
            /// before and right after a group action - a live A/B test for whether that primes
            /// the mesh into a more receptive state. No-op (logs at debug) if no time source is
            /// configured or the clock hasn't synced yet.
            void send_time_sync();

            /// Define (or redefine) an arbitrary group's membership and command it directly, with
            /// no backing `platform: fastcon` light entity and no light::LightState* - the caller
            /// (an `api: actions:` lambda, see brmesh-bridge.yaml) supplies every light_data byte
            /// already computed. `members` are raw mesh light_ids (1-based, matching
            /// set_group_members()'s own numbering - NOT the HA-facing "Light N" numbering used
            /// in scripts.yaml, same caveat as every other members: list in this repo).
            /// `group_id` must not collide with a group_id already owned by a static entity (0 is
            /// firmware-owned "all"; any group_id used by a `platform: fastcon` entity in YAML is
            /// that entity's own) or the shared group_slot - pick an unused id per ad-hoc group.
            /// `brightness` is 0-127 (the wire scale, already divided down from HA's 0-255 - see
            /// this method's own .cpp comment for why no scaling happens here). `blue`/`red`/
            /// `green`/`warm`/`cold` are each 0-255, matching get_light_data()'s own wire format.
            /// Same ensure_group()+group_control()+queueCommand()+send_time_sync() sequence as
            /// FastconLight::write_state()'s own group path (fastcon_light.cpp) - deliberately not
            /// factored into a shared helper, to keep that entity-bound path untouched by this one.
            void dynamic_group_command(uint8_t group_id, const std::vector<uint8_t> &members,
                                        bool state, uint8_t brightness,
                                        uint8_t blue, uint8_t red, uint8_t green,
                                        uint8_t warm, uint8_t cold);

            void queueCommand(uint32_t light_id_, const std::vector<uint8_t> &data);

            void clear_queue();
            bool is_queue_empty() const
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                return queue_.empty();
            }
            size_t get_queue_size() const
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                return queue_.size();
            }
            void set_max_queue_size(size_t size) { max_queue_size_ = size; }
            void set_membership_retries(uint8_t n) { membership_retries_ = n; }
            void set_membership_ttl(uint32_t ms) { membership_ttl_ = ms; }
            void set_group_slot(uint8_t id) { group_slot_ = id; }

            /// The group id used by entities that do not pin one of their own.
            uint8_t get_group_slot() const { return group_slot_; }

            void set_mesh_key(std::array<uint8_t, 4> key) { mesh_key_ = key; }
            void set_adv_interval_min(uint16_t val) { adv_interval_min_ = val; }
            void set_adv_interval_max(uint16_t val)
            {
                adv_interval_max_ = val;
                if (adv_interval_max_ < adv_interval_min_)
                {
                    adv_interval_max_ = adv_interval_min_;
                }
            }
            void set_adv_duration(uint16_t val) { adv_duration_ = val; }
            void set_adv_gap(uint16_t val) { adv_gap_ = val; }
            void set_time_source(time::RealTimeClock *time_source) { time_source_ = time_source; }

        protected:
            struct Command
            {
                std::vector<uint8_t> data;
                uint32_t timestamp;
                uint8_t retries{0};
                static constexpr uint8_t MAX_RETRIES = 3;
            };

            std::queue<Command> queue_;
            mutable std::mutex queue_mutex_;
            size_t max_queue_size_{100};

            enum class AdvertiseState
            {
                IDLE,
                ADVERTISING,
                GAP
            };

            AdvertiseState adv_state_{AdvertiseState::IDLE};
            uint32_t state_start_time_{0};

            // Protocol implementation
            std::vector<uint8_t> generate_command(uint8_t n, uint32_t light_id_, const std::vector<uint8_t> &data, bool forward = true,
                                                    bool membership_framing = false);

            struct GroupState
            {
                std::vector<uint8_t> mask;
                uint32_t written_at;
            };

            /// What we last wrote to each group id, so a repeat costs no frames until it ages out.
            std::map<uint8_t, GroupState> group_masks_;
            uint8_t membership_retries_{3};
            uint32_t membership_ttl_{30000};
            uint8_t group_slot_{0xfd};

            std::array<uint8_t, 4> mesh_key_{};

            uint16_t adv_interval_min_{0x20};
            uint16_t adv_interval_max_{0x40};
            uint16_t adv_duration_{50};
            uint16_t adv_gap_{10};

            /// Optional - unset unless `time_id` is configured. See send_time_sync().
            time::RealTimeClock *time_source_{nullptr};

            static const uint16_t MANUFACTURER_DATA_ID = 0xfff0;
            static const uint8_t GROUP_MARKER_HI = 0x2a;
            static const uint8_t GROUP_MARKER_LO = 0xa8;
        };

    } // namespace fastcon
} // namespace esphome
