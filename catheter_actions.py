"""Action and input mapping constants for catheter control.

This module is intentionally small and data-only so both runtime control and
scene setup can reuse the same action vocabulary.
"""

from typing import Dict

from Sofa.constants import Key


# BeamAdapter action enum values:
# 0 NO_ACTION, 1 MOVE_FORWARD, 2 MOVE_BACKWARD, 3 SPIN_RIGHT, 4 SPIN_LEFT, ...
ACTION_MOVE_FORWARD = 1
ACTION_MOVE_BACKWARD = 2
ACTION_SPIN_RIGHT = 3
ACTION_SPIN_LEFT = 4


# Text action names accepted in playback files.
ACTION_NAME_TO_CODE: Dict[str, int] = {
    "PUSH": ACTION_MOVE_FORWARD,
    "IN": ACTION_MOVE_FORWARD,
    "FORWARD": ACTION_MOVE_FORWARD,
    "MOVE_FORWARD": ACTION_MOVE_FORWARD,
    "UP": ACTION_MOVE_FORWARD,
    "PULL": ACTION_MOVE_BACKWARD,
    "OUT": ACTION_MOVE_BACKWARD,
    "BACKWARD": ACTION_MOVE_BACKWARD,
    "MOVE_BACKWARD": ACTION_MOVE_BACKWARD,
    "DOWN": ACTION_MOVE_BACKWARD,
    "ROTATE_RIGHT": ACTION_SPIN_RIGHT,
    "RIGHT": ACTION_SPIN_RIGHT,
    "SPIN_RIGHT": ACTION_SPIN_RIGHT,
    "R": ACTION_SPIN_RIGHT,
    "ROTATE_LEFT": ACTION_SPIN_LEFT,
    "LEFT": ACTION_SPIN_LEFT,
    "SPIN_LEFT": ACTION_SPIN_LEFT,
    "L": ACTION_SPIN_LEFT,
}


# Canonical names used when recording actions back to text.
ACTION_CODE_TO_NAME: Dict[int, str] = {
    ACTION_MOVE_FORWARD: "PUSH",
    ACTION_MOVE_BACKWARD: "PULL",
    ACTION_SPIN_RIGHT: "ROTATE_RIGHT",
    ACTION_SPIN_LEFT: "ROTATE_LEFT",
}


# Keyboard bindings used by SOFA key events.
KEY_TO_ACTION: Dict[str, int] = {
    Key.uparrow: ACTION_MOVE_FORWARD,
    Key.downarrow: ACTION_MOVE_BACKWARD,
    Key.rightarrow: ACTION_SPIN_RIGHT,
    Key.leftarrow: ACTION_SPIN_LEFT,
    "UP": ACTION_MOVE_FORWARD,
    "DOWN": ACTION_MOVE_BACKWARD,
    "RIGHT": ACTION_SPIN_RIGHT,
    "LEFT": ACTION_SPIN_LEFT,
    "W": ACTION_MOVE_FORWARD,
    "S": ACTION_MOVE_BACKWARD,
    "D": ACTION_SPIN_RIGHT,
    "A": ACTION_SPIN_LEFT,
}


# Commands exchanged with the external control window state file.
CONTROL_HOLD_TO_ACTION: Dict[str, int] = {
    "HOLD_PUSH": ACTION_MOVE_FORWARD,
    "HOLD_PULL": ACTION_MOVE_BACKWARD,
    "HOLD_ROTATE_RIGHT": ACTION_SPIN_RIGHT,
    "HOLD_ROTATE_LEFT": ACTION_SPIN_LEFT,
}

CONTROL_STEP_TO_ACTION: Dict[str, int] = {
    "STEP_PUSH": ACTION_MOVE_FORWARD,
    "STEP_PULL": ACTION_MOVE_BACKWARD,
    "STEP_ROTATE_RIGHT": ACTION_SPIN_RIGHT,
    "STEP_ROTATE_LEFT": ACTION_SPIN_LEFT,
}

CONTROL_TOGGLE_RECORD = "TOGGLE_RECORD"

