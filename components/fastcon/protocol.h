#pragma once

#include <vector>
#include <array>
#include <cstdint>
#include "utils.h"

namespace esphome
{
    namespace fastcon
    {
        static const std::array<uint8_t, 4> DEFAULT_ENCRYPT_KEY = {0x5e, 0x36, 0x7b, 0xc4};
        static const std::array<uint8_t, 3> DEFAULT_BLE_FASTCON_ADDRESS = {0xC1, 0xC2, 0xC3};

        std::vector<uint8_t> get_rf_payload(const std::vector<uint8_t> &addr, const std::vector<uint8_t> &data);
        std::vector<uint8_t> prepare_payload(const std::vector<uint8_t> &addr, const std::vector<uint8_t> &data);

        // Membership-write (cmd 5) frames use a different wire envelope from every other
        // command: no embedded/reversed BLE address, no separate CRC16 - just a fixed 2-byte
        // marker (0xA5, 0x5A) followed by the full encrypted body, whitened the same way.
        // Reverse-engineered from real BRMesh app captures (0x25 seed, same whitening_encode);
        // see set_group_members() in fastcon_controller.cpp for the derivation notes.
        std::vector<uint8_t> prepare_membership_payload(const std::vector<uint8_t> &body);
    } // namespace fastcon
} // namespace esphome