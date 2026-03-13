# Catheter BeamAdapter Simulation (SOFA + SofaPython3)

This workspace now includes:

- `catheter_simulation.py`: complete SOFA scene.
- `catheter_actions.py`: action enums and keyboard/control mappings.
- `catheter_config.py`: CLI + catheter YAML profile loading (`SimulationConfig`).
- `catheter_controls.py`: playback parser and runtime control/logging controller.
- `catheter_control_window.py`: interactive control panel used in GUI mode.
- `catheters/catheter_default.yaml`: default catheter definition (auto-loaded).
- `catheters/catheter_alt_example.yaml`: example alternate catheter definition.
- `vasculature/vascular_network.stl`: vessel wall geometry (triangulated STL).
- `controls_example.txt`: sample prerecorded deployment controls.

## Implemented Features

- STL vessel import (`vasculature/vascular_network.stl`)
- Beam-like catheter mechanics (`WireBeamInterpolation` + `AdaptiveBeamForceFieldAndMass`)
- Per-section stiffness/radius (`RodStraightSection` + `RodSpireSection`)
- Distal pre-shaped hook tip (`RodSpireSection`), defaulting to a semicircle (`hook_radius = 1.5 * shaft_radius`)
- Catheter parameters are loaded from YAML (default profile auto-loaded, switchable via `--catheter`)
- Collision + Coulomb friction (`CollisionResponse` with `FrictionContactConstraint`)
- Robust double-sided vessel wall collision (always enabled)
- Catheter tube visualization (cylindrical segment rendering via `Edge2QuadTopologicalMapping`)
- Sequential collision pipeline by default (`BruteForceBroadPhase` + `BVHNarrowPhase`)
- Interactive controls:
  - `Up`: push catheter forward
  - `Down`: retract catheter
  - `Left` / `Right`: rotate catheter
  - Fallback: `W/S/A/D` map to the same actions (useful if arrow key mapping differs by GUI backend)
- On-screen control window:
  - A separate control panel window opens in interactive mode
  - Buttons: `PUSH [W / Up]`, `PULL [S / Down]`, `ROTATE LEFT [A / Left]`, `ROTATE RIGHT [D / Right]`
  - Press-and-hold for continuous action, or use `Tap` for a single step
  - Keyboard in that window: `Up/Down/Left/Right` and `W/A/S/D`
  - `RECORD [R]` toggle starts/stops recording to timestamped files in `recordings/` (e.g. `deployment_20260313_143025.txt`)
  - Click the control window once to give it keyboard focus
- Playback from text control sequence (`--controls`)
- Headless playback mode with terminal logging of collision contact count, `xtip`, and catheter tip position (`--headless`)
- Optional recording of interactive actions to file (`--record`)

## Run (Interactive)

```bash
runSofa catheter_simulation.py
```
This auto-starts animation in normal GUI sessions (no `--controls`, no `--headless`), so no extra `-i` switch is needed.

## Run with a prerecorded deployment file

```bash
runSofa catheter_simulation.py --argv "--controls controls_example.txt"
```

## Run with another catheter profile

```bash
runSofa catheter_simulation.py --argv "--catheter catheter_alt_example.yaml"
```

## Run headless (no GUI) and print tip position

```bash
runSofa -g batch catheter_simulation.py --argv "--headless --controls controls_example.txt"
```

Optional log frequency:

```bash
runSofa -g batch catheter_simulation.py --argv "--headless --controls controls_example.txt --print-every 1"
```

Headless log format includes `contacts=...` so collision activity is directly visible.

## Record an interactive deployment

```bash
runSofa catheter_simulation.py --argv "--record deployment_recorded.txt"
```

Then replay:

```bash
runSofa catheter_simulation.py --argv "--controls deployment_recorded.txt"
```

## Important Scene Parameters

You can override these from CLI:

- `--vessel` STL path (searches `vasculature/` first for bare filenames)
- `--vessel-alpha` STL visual transparency (`0` fully transparent, `1` opaque)
- `--catheter` load a specific catheter YAML file (searches `catheters/` first; default: `catheters/catheter_default.yaml`)
- `--no-control-window` disable the external control panel window
- `--control-state-file` path for control-window command exchange (advanced/debug)
- `--dt` simulation timestep
- `--friction` Coulomb friction coefficient
- `--contact-distance` collision contact distance
- `--alarm-distance` collision alarm distance
- `--print-every` headless tip-print stride (steps)
- `--collision-radius` catheter radius contribution in contact detection
- `--insertion-step` insertion increment per push action
- `--rotation-step` rotation increment per rotate action
- `--parallel-collision` opt-in parallel broadphase/narrowphase

Core defaults in `catheter_simulation.py`:

- Start pose: position `(0,0,0)`, direction `(-1,0,0)`
- Per-section radius/stiffness:
  - Shaft: larger radius, higher Young's modulus
  - Distal tip: smaller radius, lower Young's modulus
- Defaults are read from `catheters/catheter_default.yaml`
- Distal hook defaults to a semicircle: `hook_radius=1.5*shaft_radius`, `tip_length=pi*hook_radius` (can be changed in YAML)
