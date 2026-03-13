#!/usr/bin/env python3
import argparse
from pathlib import Path
import tkinter as tk


def _write_state(path: Path, command: str) -> None:
    path.write_text(f"{command}\n", encoding="utf-8")


def _build_ui(state_file: Path) -> None:
    state_file.parent.mkdir(parents=True, exist_ok=True)
    _write_state(state_file, "NONE")

    root = tk.Tk()
    root.title("Catheter Controls")
    root.geometry("340x360")
    root.resizable(False, False)
    try:
        root.attributes("-topmost", True)
    except Exception:
        pass

    title = tk.Label(root, text="Catheter Control Panel", font=("Helvetica", 14, "bold"))
    title.pack(pady=(10, 8))

    info = tk.Label(root, text="Press and hold for continuous motion")
    info.pack(pady=(0, 8))

    kb_info = tk.Label(root, text="Keyboard: arrows, W/A/S/D, R (record)")
    kb_info.pack(pady=(0, 6))

    frame = tk.Frame(root)
    frame.pack(fill=tk.BOTH, expand=True, padx=12, pady=4)

    key_to_hold_cmd = {
        "Up": "HOLD_PUSH",
        "w": "HOLD_PUSH",
        "W": "HOLD_PUSH",
        "Down": "HOLD_PULL",
        "s": "HOLD_PULL",
        "S": "HOLD_PULL",
        "Left": "HOLD_ROTATE_LEFT",
        "a": "HOLD_ROTATE_LEFT",
        "A": "HOLD_ROTATE_LEFT",
        "Right": "HOLD_ROTATE_RIGHT",
        "d": "HOLD_ROTATE_RIGHT",
        "D": "HOLD_ROTATE_RIGHT",
    }
    pressed_keys = []
    recording_enabled = False
    record_button_text = tk.StringVar(value="RECORD [R]: OFF")

    def make_hold_button(text: str, hold_cmd: str, step_cmd: str) -> None:
        row = tk.Frame(frame)
        row.pack(fill=tk.X, pady=4)

        hold_btn = tk.Button(row, text=text, width=18, height=2)
        hold_btn.pack(side=tk.LEFT)
        hold_btn.bind("<ButtonPress-1>", lambda _e: _write_state(state_file, hold_cmd))
        hold_btn.bind("<ButtonRelease-1>", lambda _e: _write_state(state_file, "NONE"))

        tap_btn = tk.Button(row, text="Tap", width=6, command=lambda: _write_state(state_file, step_cmd))
        tap_btn.pack(side=tk.RIGHT, padx=(8, 0))

    make_hold_button("PUSH [W / Up]", "HOLD_PUSH", "STEP_PUSH")
    make_hold_button("PULL [S / Down]", "HOLD_PULL", "STEP_PULL")
    make_hold_button("ROTATE LEFT [A / Left]", "HOLD_ROTATE_LEFT", "STEP_ROTATE_LEFT")
    make_hold_button("ROTATE RIGHT [D / Right]", "HOLD_ROTATE_RIGHT", "STEP_ROTATE_RIGHT")

    record_row = tk.Frame(frame)
    record_row.pack(fill=tk.X, pady=(10, 4))
    record_btn = tk.Button(record_row, textvariable=record_button_text, width=28, height=2)
    record_btn.pack(side=tk.LEFT)

    def toggle_recording() -> None:
        nonlocal recording_enabled
        recording_enabled = not recording_enabled
        record_button_text.set("RECORD [R]: ON" if recording_enabled else "RECORD [R]: OFF")
        _write_state(state_file, "TOGGLE_RECORD")

    record_btn.configure(command=toggle_recording)

    def apply_command_from_keys() -> None:
        for k in reversed(pressed_keys):
            cmd = key_to_hold_cmd.get(k, None)
            if cmd is not None:
                _write_state(state_file, cmd)
                return
        _write_state(state_file, "NONE")

    def on_key_press(event) -> None:
        k = event.keysym
        if k in ("r", "R"):
            return
        if k not in key_to_hold_cmd:
            return
        if k not in pressed_keys:
            pressed_keys.append(k)
        apply_command_from_keys()

    def on_key_release(event) -> None:
        k = event.keysym
        if k in ("r", "R"):
            toggle_recording()
            return
        if k not in key_to_hold_cmd:
            return
        pressed_keys[:] = [pk for pk in pressed_keys if pk != k]
        apply_command_from_keys()

    root.bind_all("<KeyPress>", on_key_press)
    root.bind_all("<KeyRelease>", on_key_release)
    root.focus_force()

    def on_close() -> None:
        try:
            _write_state(state_file, "NONE")
        finally:
            root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--state-file", required=True)
    args = parser.parse_args()

    _build_ui(Path(args.state_file).resolve())


if __name__ == "__main__":
    main()
