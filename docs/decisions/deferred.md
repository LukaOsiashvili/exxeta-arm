# Deferred items

SHOULD / MAY items from the spec that are postponed so that the core v1 gets built first, plus two MUST items that are not finished. Each entry says what exists and what is needed.

| Item | Spec | Status | What is needed |
|---|---|---|---|
| **Base interior: module mounts, rear panel with switch / jack / USB-C cut-outs, LED light pipe** | 4.6 (MUST) | **Not done.** The base shell has the opening for a rear panel, vents and a 5 mm LED hole, but there is no rear-panel part and there are no mounting points. | The real modules in hand (sizes differ between sellers). Until then: foam tape on the bottom panel. This is the main gap in "physically buildable". |
| Feature-parametric Fusion 360 model | 4.8, acceptance 8 | The Fusion model is full-detail but built from exact imported geometry, not Fusion features (ADR-0009). Built and saved. A simplified feature-based script exists but was not run. | Owner decision whether the feature-based version is still wanted. |
| USB-C power-bank charge input | 6.2 (SHOULD consider) | Optional module in the BOM and the wiring diagram; no panel cut-out yet | Decide when the base interior is designed. Only usable with the arm parked. |
| J5 (wrist roll) pot mount | 4.5, D3 | Dropped for v1 by owner decision | Keeps mass off the wrist. The ADC1 pin stays free; see ADR-0002. |
| KiCad schematic, perfboard layout | 5.3 | SVG wiring diagram only | The design uses ready-made modules on a screw-terminal board; a perfboard layout follows once the modules are chosen. |
| Pot calibration wizard in the app | 7.4, 8.3 | Firmware, protocol, estimator and 3D ghost exist (tried with simulated pots only); the app has no pot wizard | A second section in the calibration panel that records ADC readings at the two reference points. |
| "Limp teach" mode | 8.4 (MAY) | Not done | Needs pots and a safe-pose check. |
| Cartesian waypoints in the program editor | 8.4 | The recorder and planner accept Cartesian steps; the app's editor only creates joint waypoints | A type switch per waypoint. |
| Wi-Fi provisioning helper in the app | 8.3 | Basic: the SETUP tab can send SSID and password to a connected arm | The hotspot page on the arm is the main route. |
| Limits editing in an "advanced" mode | 8.3 | Limits are shown read-only | Change them in `arm_params.json` and regenerate. |
| MuJoCo simulation, joint torque display | 8.2, 9 (MAY) | Not done | The simple servo model is used. |
| Rerun visualisation node | 8.2 (MAY) | Not done | |
| ADS1115 external ADC | 5.1 (MAY) | Not done; the I2C pins are reserved | |
| PWA offline support | 8.3 | Manifest and icons only | Needs HTTPS, which a LAN address does not have. |
| Heat-set insert option | 4.4 (MAY) | The parameter exists (`options.fastener_type`) but the CAD only implements captive nuts | |
| Hard stops on J4, J5, J6 | 4.4 ("where practical") | J1, J2 and J3 have printed stops. J4 to J6 rely on the servo's internal stop and the software limits | Loads there are small. |
| Second-colour "EXXETA" text | 4.6 | Engraved 0.8 mm deep, same colour | Fill with paint, or print the text as an inlay. |
| GLB meshes | 4.8 | STL meshes only | The web app loads STL directly. |
| Assembly guide with step renders | 12.1 | Text steps with whole-part renders | Exploded step renders once the first arm is built. |
