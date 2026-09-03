# ESPHome Fastcon BLE Light Component

This is a custom component for ESPHome that allows you to control Broadlink Fastcon BLE lights, also known as brMesh. It should work with any light that can be controlled by brMesh or Broadlink BLE mobile apps.

Be warned - there is also a brLight app, which might look like brMesh, but the protocol is different.

**This is the `group-support` branch** — a fork of [`scross01/esphome-fastcon`](https://github.com/scross01/esphome-fastcon) that adds BLE-mesh group addressing (control several lights with one advertisement instead of one per light) on top of that base's `color_interlock`/`supports_cwww` work. See "Group addressing" below.

## Requirements

- ESP32 board
- ESPHome 2023.12.0 or newer

## Supported Features

- On/Off control
- Brightness control
- RGB color control
- White mode
- Group addressing: address several lights with a single BLE advertisement, either a named group defined on demand (`members:`) or an existing mesh group id (`group_id:`)

## Configuration

Add the following to your ESPHome configuration:

```yaml
# ESP32 is required
esp32:
  board: esp32-s3-devkitc-1
  framework:
    type: arduino

esp32_ble_tracker:
esp32_ble_server:

# Source configuration
external_components:
  - source: github://rangerdavies/esphome-fastcon@group-support

# Optional - lets the controller bracket group actions with a time-sync frame,
# matching a habit observed in the phone app. See "Group addressing" below.
time:
  - platform: homeassistant
    id: ha_time

# Controller configuration
fastcon:
  mesh_key: "12345678"    # Your mesh key in hex format

  # Optional parameters to control the advertisement protocol with their defaults:
  adv_interval_min: 0x20  # Minimum advertisement interval
  adv_interval_max: 0x40  # Maximum advertisement interval
  adv_duration: 50        # Advertisement duration in milliseconds
  adv_gap: 10             # Gap between advertisements in milliseconds
  max_queue_size: 100     # Maximum number of queued commands

  # Optional - group addressing, see below
  membership_retries: 3   # Repeats of a membership write (the app does 3)
  membership_ttl: 30s     # How long a written membership is trusted before rewriting
  group_slot: 0xFD        # Shared group id used by named (members:) groups
  time_id: ha_time        # Time source for time-sync frames around group actions

# Light configuration (add an entry for each light)
light:
  # Single light, addressed by its own mesh id
  - platform: fastcon
    id: living_room_light
    name: "Living Room Light"
    light_id: 1           # ID of the light (1-255)
    supports_cwww: true   # Optional: Set to true if the light supports cold/warm white
    color_interlock: true # Optional: Set to true to prevent RGB and white LEDs from being on at the same time
    default_transition_length: 0s  # Recommended for every fastcon entity - see below

  # Named group, defined on demand on the shared group_slot
  - platform: fastcon
    name: "Kitchen Group"
    members: [1, 2, 4]     # Mesh ids of the lights in this group
    supports_cwww: true
    color_interlock: true
    default_transition_length: 0s

  # An existing mesh group id - group_id: 0 is the built-in "all lights" group
  # and needs no membership write at all
  - platform: fastcon
    name: "All Lights"
    group_id: 0
    supports_cwww: true
    color_interlock: true
    default_transition_length: 0s
```

### Configuration Variables

#### Fastcon Controller

- **mesh_key** (*Required*, string): The mesh key for your Fastcon lights in hexadecimal format (8 characters/4 bytes)
- **id** (*Optional*, ID): The ID to use for this controller component. Defaults to "fastcon_controller"
- **adv_interval_min** (*Optional*, int): Minimum advertisement interval. Defaults to 0x20
- **adv_interval_max** (*Optional*, int): Maximum advertisement interval. Defaults to 0x40
- **adv_duration** (*Optional*, int): Duration of each advertisement in milliseconds. Defaults to 50
- **adv_gap** (*Optional*, int): Gap between advertisements in milliseconds. Defaults to 10
- **max_queue_size** (*Optional*, int): Maximum number of commands that can be queued. Defaults to 100
- **membership_retries** (*Optional*, int, 1-10): How many times a group membership write is repeated - a missed write silently drops a light from the group, so it's repeated the way the app does. Defaults to `3`
- **membership_ttl** (*Optional*, time period): How long a written membership is trusted before it's rewritten. `0s` disables the cache entirely (correct, but costs a rewrite on every command). Defaults to `30s`
- **group_slot** (*Optional*, int, 1-255): The group id used by named (`members:`) groups that don't pin their own `group_id`. `0xFD` is the id the BRMesh app itself uses for its own ad-hoc multi-selections, and the only non-zero id confirmed to work on real hardware. Defaults to `0xFD`
- **time_id** (*Optional*, ID of a `time:` platform): If set, the controller sends a time-sync frame immediately before and after each group action, matching a pattern observed in real captures of the phone app. No effect on single-light entities; a no-op if the clock hasn't synced yet.

#### Fastcon Light

Exactly one of `light_id`, `members`, or `group_id` is required per light.

- **light_id** (*Optional*, int, 1-255): Address one light directly, by its own mesh id
- **members** (*Optional*, list of int): A named group, defined on demand on the controller's `group_slot`. Membership is (re)written automatically whenever it's stale or has changed. **Unreliable in real-hardware testing so far - see "Group addressing" below; prefer `group_id:` where a suitable group id already exists.**
- **group_id** (*Optional*, int, 0-255): Address a group id that already exists in the mesh. `group_id: 0` is the "all lights" group built into the firmware and needs no membership write - it can't be combined with `members:`
- **name** (*Required*, string): The name for the light entity
- **id** (*Optional*, ID): The ID to use for this light component
- **controller_id** (*Optional*, ID): The ID of the controller to use. Defaults to "fastcon_controller"
- **supports_cwww** (*Optional*, boolean): Set to `true` if the light supports cold/warm white channels. Defaults to `false`.
- **color_interlock** (*Optional*, boolean): Set to `true` to prevent RGB and white LEDs from being on at the same time. Defaults to `false`.
- **default_transition_length** (*Optional*, time period): Recommended `0s` for every fastcon entity - without it, ESPHome interpolates transitions into a burst of intermediate advertisements, which throws away the benefit of the group frame.

## Group addressing

Stock behaviour is one BLE advertisement per light per state change, so N lights cost N frames per state change and colour transitions can overflow the command queue. A group command costs a single frame regardless of how many lights are in the group.

> **`members:` (named/dynamic groups) has been unreliable in real-hardware testing.** Even
> with correct, verified-against-the-app membership-write bytes, repeated tests have shown
> some intended member lights failing to join and/or stale members from a *previous*
> group's `members:` list still responding - most likely because several physically
> separate bulbs all have to independently receive the same single membership-write
> broadcast, and any one of them missing it leaves the mesh's idea of that group's
> membership wrong. **Prefer `group_id:` to an existing, already-defined mesh group where
> you can** (`group_id: 0`, the built-in "all lights" group, needs no membership write at
> all and has been reliable) - reserve `members:` for cases where no suitable group id
> already exists.

- A **named group** (`members:`) is defined on demand: the controller writes membership for its light ids to the shared `group_slot` the first time it's needed, or whenever the membership has changed or aged past `membership_ttl`, before sending the group command itself.
- An **existing group id** (`group_id:`) addresses a group id that's already meaningful on the mesh. `group_id: 0` is the firmware's built-in "all lights" group and needs no membership write at all. `group_id:` can also be combined with `members:` to pin a named group to its own dedicated id instead of sharing `group_slot` with every other named group and the app's own ad-hoc multi-select.
- **State is optimistic.** These lights never report back, so driving a group does not update the individual member entities in Home Assistant - they'll show stale state.
- **The membership cache is RAM-only** and per group id. A reboot forces one rewrite; two different named groups sharing the same `group_slot` will each force a rewrite when switched between.

## Finding Your Mesh Key

The mesh key is crucial for controlling your Fastcon BLE lights. To find your light's mesh key, you first need to setup your devices using an Android device. The app generates a unique mesh key that will be used with all lights that are set up in the app.

Once the lights are setup, you can use ADB to connect to your phone and you may use the following command to extract the mesh key.

```bash
adb logcat | { grep -m 1 -o 'jyq_helper: .* payload:.\{24\},[[:space:]]*key:[[:space:]]*.\{8\}' | awk '{print $NF}'; kill -2 $(pgrep -P $$ adb); }
```

While running the above, open the app and toggle a light on and off. The command should then output your mesh key.

## Acknowledgments

This component builds upon the reverse engineering and hard work of several others who must be acknowledged and thanked:

### Protocol Reverse Engineering

The foundational protocol reverse engineering work was done by [Mooody](https://mooody.me/posts/2023-04/reverse-the-fastcon-ble-protocol/), who provided detailed analysis of the Fastcon BLE protocol, including packet structure and encryption methods. https://mooody.me/posts/2023-04/reverse-the-fastcon-ble-protocol/

### Implementation References

- [ArcadeMachinist's brMeshMQTT](https://github.com/ArcadeMachinist/brMeshMQTT) - This work was crucial in helping me understand the practical implementation details of the protocol. https://github.com/ArcadeMachinist/brMeshMQTT

### Community Resources

- [Home Assistant Community Thread](https://community.home-assistant.io/t/brmesh-app-bluetooth-lights/473486/102)

This ESPHome component adapts and/or takes heavy inspiration from all of these works to run directly on ESP32 devices, allowing for native integration with Home Assistant without requiring additional bridges or MQTT brokers. A huge thank you to all those who contributed to my understanding of the Fastcon BLE protocol.

## License

This project is licensed under the MIT License - see the LICENSE file for details.
