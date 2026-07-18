# Fire Guardian: Fire Eyes for the Edge

> An embedded-systems contest prototype for vision-based laboratory fire early warning.

`OpenMV` `Embedded Systems` `Computer Vision` `IoT` `MicroPython` `ESP8266` `Flutter` `Fire Detection` `Student Project`

Fire Guardian explores a practical edge-device loop: detect suspicious flames, combine temperature information, trigger local alarms, and optionally send a remote notification. It is a contest prototype and learning record, not a certified fire-alarm product.

## Highlights

- Flame candidates detected with color, area, and shape features
- Temperature-assisted decisions to reduce false alarms
- Local OLED, LED, and buzzer feedback
- ESP8266 LAN connectivity and optional remote notification
- A Flutter companion app experiment in `v2`

## Start here

1. Open `v1/main.py` in OpenMV IDE.
2. Run the camera, display, LED, and buzzer first.
3. Adjust pin mappings and thresholds for your own hardware.
4. Add the ESP8266 workflow only after the local detection loop works.

## Project map

| Path | What it contains |
| --- | --- |
| `v1/` | Baseline OpenMV detection and local/LAN alerts |
| `v2/` | Extended detection, remote notification, and Flutter app |
| `failure-trys/` | Experiments retained for retrospective learning |

## Configuration safety

Wi-Fi and notification values in `v2` use placeholders. Keep your real credentials only in a local copy and never commit them.

## Design report

See the [redacted public report](v2/Fire-Guardian-public-report.docx) for the project design and iteration notes.

> Safety note: This is an embedded-systems contest/teaching prototype and must not replace a standards-compliant fire alarm system.
