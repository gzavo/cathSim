"""Runtime control helpers for playback, input handling, and logging."""

import atexit
import datetime
import math
import os
import subprocess
from pathlib import Path
from typing import List, Optional, Tuple

import Sofa
import Sofa.Core

from catheter_actions import (
    ACTION_CODE_TO_NAME,
    ACTION_MOVE_BACKWARD,
    ACTION_MOVE_FORWARD,
    ACTION_NAME_TO_CODE,
    ACTION_SPIN_LEFT,
    ACTION_SPIN_RIGHT,
    CONTROL_HOLD_TO_ACTION,
    CONTROL_STEP_TO_ACTION,
    CONTROL_TOGGLE_RECORD,
    KEY_TO_ACTION,
)
from catheter_config import scene_dir


def parse_controls_file(path: str, dt: float) -> Tuple[List[float], List[int]]:
    """Parse a push/pull/rotate control script into BeamAdapter action arrays."""
    times: List[float] = []
    actions: List[int] = []
    next_implicit_t = 0.0
    with open(path, "r", encoding="utf-8") as f:
        for line_number, raw in enumerate(f, start=1):
            line = raw.strip()
            if not line or line.startswith("#"):
                continue

            tokens = line.split()
            t: float
            action_token: str
            repeat = 1

            if len(tokens) >= 2:
                try:
                    t = float(tokens[0])
                    action_token = tokens[1]
                    if len(tokens) >= 3:
                        repeat = int(tokens[2])
                except ValueError:
                    t = next_implicit_t
                    action_token = tokens[0]
                    if len(tokens) >= 2:
                        repeat = int(tokens[1])
            else:
                t = next_implicit_t
                action_token = tokens[0]

            action_key = action_token.strip().upper()
            if action_key not in ACTION_NAME_TO_CODE:
                raise ValueError(
                    f"Unknown action '{action_token}' in {path}:{line_number}. "
                    f"Expected one of {sorted(ACTION_NAME_TO_CODE.keys())}."
                )

            for k in range(max(1, repeat)):
                tk = t + k * dt
                times.append(tk)
                actions.append(ACTION_NAME_TO_CODE[action_key])
                next_implicit_t = tk + dt

    paired = sorted(zip(times, actions), key=lambda ta: ta[0])
    if not paired:
        return [], []
    times_sorted, actions_sorted = zip(*paired)
    return list(times_sorted), list(actions_sorted)


class CatheterControlAndLoggingController(Sofa.Core.Controller):
    """SOFA controller that centralizes runtime control and debug logging.

    Responsibilities:
    - keyboard control (push/pull/rotate)
    - polling external control window commands
    - optional recording of user actions
    - periodic headless debug prints (contacts + tip pose)
    """

    def __init__(
        self,
        root: Sofa.Core.Node,
        action_controller,
        ircontroller,
        tip_collision_mo,
        collision_models,
        record_file: Optional[str],
        headless: bool,
        print_every_steps: int,
        head_direction: Tuple[float, float, float],
        enable_control_window: bool,
        control_state_file: Optional[str],
        control_window_script: Optional[str],
        hold_repeat_steps: int = 2,
        *args,
        **kwargs,
    ):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.root = root
        self.action_controller = action_controller
        self.ircontroller = ircontroller
        self.tip_collision_mo = tip_collision_mo
        self.collision_models = list(collision_models) if collision_models is not None else []
        self.record_file = record_file
        self.headless = headless
        self.print_every_steps = max(1, print_every_steps)
        self.enable_control_window = bool(enable_control_window) and not bool(headless)
        self.control_state_file = str(control_state_file) if control_state_file else None
        self.control_window_script = str(control_window_script) if control_window_script else None
        self.hold_repeat_steps = max(1, int(hold_repeat_steps))
        self._external_hold_action: Optional[int] = None
        self._last_external_command: str = ""
        self._control_window_proc = None
        self.step_count = 0
        self.listening = True
        try:
            self.findData("listening").value = True
        except Exception:
            pass
        dx, dy, dz = head_direction
        n = math.sqrt(dx * dx + dy * dy + dz * dz)
        if n < 1.0e-12:
            self.head_direction = (1.0, 0.0, 0.0)
        else:
            self.head_direction = (dx / n, dy / n, dz / n)

        self._record_handle = None
        self._recording_path: Optional[str] = None
        if self.record_file:
            self._start_recording(self.record_file)

        if self.enable_control_window and self.control_state_file and self.control_window_script:
            self._initialize_control_state_file()
            self._launch_control_window()

    def __del__(self):
        self._stop_recording()
        if self._control_window_proc is not None:
            try:
                if self._control_window_proc.poll() is None:
                    self._control_window_proc.terminate()
            except Exception:
                pass

    def _terminate_control_window(self) -> None:
        if self._control_window_proc is None:
            return
        try:
            if self._control_window_proc.poll() is None:
                self._control_window_proc.terminate()
        except Exception:
            pass

    def _initialize_control_state_file(self) -> None:
        try:
            Path(self.control_state_file).parent.mkdir(parents=True, exist_ok=True)
            Path(self.control_state_file).write_text("NONE\n", encoding="utf-8")
        except Exception as exc:
            print(f"[control-window] Failed to initialize control state file '{self.control_state_file}': {exc}")

    def _launch_control_window(self) -> None:
        cmd = ["python3", self.control_window_script, "--state-file", self.control_state_file]
        try:
            self._control_window_proc = subprocess.Popen(cmd)
            atexit.register(self._terminate_control_window)
            print(f"[control-window] Started: {' '.join(cmd)}")
        except Exception as exc:
            print(f"[control-window] Failed to start control window ({cmd}): {exc}")

    def _poll_external_control_window(self) -> None:
        if not self.enable_control_window or not self.control_state_file:
            return
        try:
            cmd = Path(self.control_state_file).read_text(encoding="utf-8").strip().upper()
        except Exception:
            return
        if not cmd:
            cmd = "NONE"

        is_step_cmd = cmd in CONTROL_STEP_TO_ACTION
        if (not is_step_cmd) and cmd == self._last_external_command:
            return
        self._last_external_command = cmd

        if cmd == "NONE":
            self._external_hold_action = None
            return
        if cmd == CONTROL_TOGGLE_RECORD:
            self._toggle_recording()
            try:
                Path(self.control_state_file).write_text("NONE\n", encoding="utf-8")
            except Exception:
                pass
            return
        if cmd in CONTROL_HOLD_TO_ACTION:
            self._external_hold_action = CONTROL_HOLD_TO_ACTION[cmd]
            return
        if cmd in CONTROL_STEP_TO_ACTION:
            action = CONTROL_STEP_TO_ACTION[cmd]
            self._apply_action_direct(action)
            self._record_action(action, float(self.root.time.value))
            try:
                Path(self.control_state_file).write_text("NONE\n", encoding="utf-8")
            except Exception:
                pass

    def _record_action(self, action_code: int, t_apply: float) -> None:
        if self._record_handle is None:
            return
        action_name = ACTION_CODE_TO_NAME.get(action_code, "UNKNOWN")
        self._record_handle.write(f"{t_apply:.6f} {action_name}\n")
        self._record_handle.flush()

    def _make_timestamp_record_path(self) -> str:
        recordings_dir = (scene_dir() / "recordings").resolve()
        recordings_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        base = recordings_dir / f"deployment_{stamp}.txt"
        if not base.exists():
            return str(base)
        for i in range(1, 1000):
            candidate = recordings_dir / f"deployment_{stamp}_{i:03d}.txt"
            if not candidate.exists():
                return str(candidate)
        return str(recordings_dir / f"deployment_{stamp}_{os.getpid()}.txt")

    def _start_recording(self, path: Optional[str] = None) -> None:
        self._stop_recording()
        target = path if path else self._make_timestamp_record_path()
        Path(target).parent.mkdir(parents=True, exist_ok=True)
        self._record_handle = open(target, "w", encoding="utf-8")
        self._record_handle.write("# time_s action\n")
        self._record_handle.flush()
        self._recording_path = target
        print(f"[record] Started: {target}")

    def _stop_recording(self) -> None:
        if self._record_handle is not None and not self._record_handle.closed:
            self._record_handle.flush()
            self._record_handle.close()
            if self._recording_path:
                print(f"[record] Stopped: {self._recording_path}")
        self._record_handle = None
        self._recording_path = None

    def _toggle_recording(self) -> None:
        if self._record_handle is not None:
            self._stop_recording()
        else:
            # Toggle from the control window always creates a new timestamped file.
            self._start_recording(None)

    def _apply_action_direct(self, action_code: int) -> None:
        step = float(self.ircontroller.findData("step").value)
        angular_step = float(self.ircontroller.findData("angularStep").value)
        xtip_data = self.ircontroller.findData("xtip")
        rot_data = self.ircontroller.findData("rotationInstrument")

        xtip = list(xtip_data.value)
        rot = list(rot_data.value)
        if not xtip:
            xtip = [0.0]
        if not rot:
            rot = [0.0]

        if action_code == ACTION_MOVE_FORWARD:
            xtip[0] = float(xtip[0]) + step
        elif action_code == ACTION_MOVE_BACKWARD:
            xtip[0] = max(0.0, float(xtip[0]) - step)
        elif action_code == ACTION_SPIN_RIGHT:
            rot[0] = float(rot[0]) + angular_step
        elif action_code == ACTION_SPIN_LEFT:
            rot[0] = float(rot[0]) - angular_step

        xtip_data.value = xtip
        rot_data.value = rot

    def _handle_key_action(self, key_value) -> None:
        if key_value is None:
            return
        key = key_value
        if isinstance(key, str):
            key = key.strip()
            upper = key.upper()
            if upper in KEY_TO_ACTION:
                action_code = KEY_TO_ACTION[upper]
            else:
                action_code = KEY_TO_ACTION.get(key, None)
        else:
            action_code = KEY_TO_ACTION.get(key, None)
        if action_code is None:
            return

        self._apply_action_direct(int(action_code))
        self._record_action(int(action_code), float(self.root.time.value))

    def onKeypressedEvent(self, event):
        key = event.get("key", None)
        self._handle_key_action(key)

    # Compatibility with legacy SOFA callback signature.
    def onKeypressed(self, key):
        self._handle_key_action(key)

    def onAnimateBeginEvent(self, _event):
        self.step_count += 1
        self._poll_external_control_window()
        if self._external_hold_action is not None and (self.step_count % self.hold_repeat_steps == 0):
            self._apply_action_direct(int(self._external_hold_action))
            self._record_action(int(self._external_hold_action), float(self.root.time.value))

        if not self.headless:
            return
        if self.step_count % self.print_every_steps != 0:
            return

        total_contacts = 0
        contact_breakdown: List[str] = []
        for label, model in self.collision_models:
            if model is None:
                continue
            n_contacts_data = model.findData("numberOfContacts")
            if n_contacts_data is None:
                continue
            try:
                n = int(n_contacts_data.value)
                total_contacts += n
                contact_breakdown.append(f"{label}:{n}")
            except Exception:
                pass
        contacts_str = ",".join(contact_breakdown) if contact_breakdown else "-"

        positions = list(self.tip_collision_mo.findData("position").value)
        if not positions:
            print(f"[t={float(self.root.time.value):.4f}] contacts={total_contacts} [{contacts_str}] tip=(nan, nan, nan)")
            return

        dx, dy, dz = self.head_direction
        tip = max(positions, key=lambda p: float(p[0]) * dx + float(p[1]) * dy + float(p[2]) * dz)
        xtip_values = list(self.ircontroller.findData("xtip").value)
        xtip_scalar = float(xtip_values[0]) if xtip_values else 0.0
        print(
            "[t={:.4f}] contacts={} [{}] xtip={:.3f} tip=({:.5f}, {:.5f}, {:.5f}) nNodes={}".format(
                float(self.root.time.value),
                total_contacts,
                contacts_str,
                xtip_scalar,
                float(tip[0]),
                float(tip[1]),
                float(tip[2]),
                len(positions),
            )
        )

