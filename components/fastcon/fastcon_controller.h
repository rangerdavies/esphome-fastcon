#pragma once

#include <queue>
#include <deque>
#include <map>
#include <mutex>
#include <vector>
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/components/esp32_ble_server/ble_server.h"
#ifdef USE_ESP32_BLE_TRACKER
#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"
#endif

namespace esphome
{
    namespace time
    {
        class RealTimeClock;
    }

    namespace fastcon
    {
        class FastconLight;

        class FastconController : public Component
#ifdef USE_ESP32_BLE_TRACKER
            ,
                                  public esp32_ble_tracker::ESPBTDeviceListener
#endif
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

            /// Write the membership of `group_id`, unconditionally, every time this is called.
            /// (2026-09-03 - no longer skips on a "still fresh" cache hit: these bulbs hold
            /// exactly one group assignment each, so a different group_id's membership write
            /// silently evicts a shared bulb with no way for this controller to detect it - see
            /// this method's own .cpp comment for the confirmed-live incident that established
            /// this.) membership_ttl_/group_masks_'s written_at are no longer read for that
            /// purpose - group_masks_ is kept only as a last-written record.
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
            /// `group_id == 0` is explicitly safe to pass, though: it's treated as the same
            /// firmware-owned "all" group static entities use, and `members` is silently ignored
            /// for it (no membership write is ever attempted) - a caller wanting group 0 can still
            /// pass every member id, purely for its own target_state/believed_state bookkeeping.
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
            /// No longer affects behavior (2026-09-03) - ensure_group() rewrites membership
            /// unconditionally on every call now, see its own header comment. Kept only so the
            /// `fastcon: membership_ttl:` YAML option (fastcon_controller.py) still compiles for
            /// anyone with it set.
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

            // ---------------------------------------------------------------------------
            // Passive sniffer
            //
            // These bulbs never report anything: the protocol is one-way non-connectable
            // advertising, there is no GATT to read, no query command exists in any of the
            // five observed command types, and the phone app itself only tracks what it
            // sent. So this cannot ask a bulb its state.
            //
            // What it CAN do is hear every OTHER controller on the mesh. The phone app and
            // any FastCon scene switch broadcast their commands in the clear on the same
            // manufacturer id, under the same mesh key. Decoding those keeps Home Assistant
            // in step when somebody picks up their phone - the case that actually makes
            // state go stale in practice.
            //
            // Limits, stated plainly: this observes what was COMMANDED, not what a bulb is
            // doing. A bulb that missed the frame still reports wrong, and one switched off
            // at the wall is invisible. It narrows the drift, it does not remove it.
            // ---------------------------------------------------------------------------
            void set_sniffer_enabled(bool b) { sniffer_enabled_ = b; }
            void register_light(FastconLight *light) { lights_.push_back(light); }

            /// Group id a bulb was last observed being assigned to, from a sniffed cmd-1
            /// frame. Returns -1 if we have never seen one for this light.
            int observed_group_of(uint8_t light_id) const;

#ifdef USE_ESP32_BLE_TRACKER
            // ble_device_base::ESPBTDevice, not esp32_ble_tracker::ESPBTDevice - the latter is only
            // a `using` alias, and only exists when USE_ESP32_BLE_DEVICE happens to be defined.
            bool parse_device(const ble_device_base::ESPBTDevice &device) override;
#endif

        protected:
            /// Un-whiten, unwrap and decrypt one 0xfff0 manufacturer payload, then dispatch
            /// it. Silently drops anything that is not a well-formed frame for our mesh.
            void handle_sniffed_payload_(const std::vector<uint8_t> &payload);
            void dispatch_observed_(const std::vector<uint8_t> &inner);

            /// Remember an inner payload we are about to transmit. The bulbs RELAY every
            /// frame (confirmed live 2026-09-03: six distinct BLE addresses, one per bulb,
            /// rebroadcast each command), so the sniffer hears everything this controller
            /// sends. Without this the decode publishes our own command straight back onto
            /// the entity, which re-encodes it a rounding step away from where it started and
            /// transmits again - a loop that walks brightness and colour temperature down.
            void note_sent_(const std::vector<uint8_t> &inner);
            bool was_sent_by_us_(const std::vector<uint8_t> &inner);

            struct SentFrame
            {
                std::vector<uint8_t> inner;
                uint32_t at;
            };
            std::deque<SentFrame> recent_sent_;
            static const uint32_t SENT_ECHO_WINDOW_MS = 5000;
            static const size_t SENT_ECHO_MAX = 48;

            bool sniffer_enabled_{false};
            std::vector<FastconLight *> lights_;

            /// light_id -> group_id, learned from sniffed cmd-1 assignment frames. Lets a
            /// sniffed group command update the individual entities that group contains.
            std::map<uint8_t, uint8_t> observed_light_group_;

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

            /// What we last wrote to each group id - kept only as a last-written record
            /// (2026-09-03: no longer used to skip a rewrite, see ensure_group()'s own
            /// comment - these bulbs hold one group assignment each, so "we wrote it recently"
            /// says nothing about whether a DIFFERENT group_id's write has since evicted it).
            std::map<uint8_t, GroupState> group_masks_;
            uint8_t membership_retries_{3};
            uint32_t membership_ttl_{30000};  // unused - see set_membership_ttl()'s own comment
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
