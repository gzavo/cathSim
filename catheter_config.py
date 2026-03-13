"""Configuration loading for the catheter simulation.

Responsibilities of this module:
- define the simulation configuration dataclass
- parse CLI flags (including runSofa --argv forwarding behavior)
- load catheter geometry/material defaults from a simple YAML file
"""

import argparse
import math
import os
import shlex
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

DEFAULT_CATHETER_DIR = "catheters"
DEFAULT_VASCULATURE_DIR = "vasculature"
DEFAULT_CATHETER_FILE = "catheter_default.yaml"
DEFAULT_VESSEL_FILE = "vascular_network.stl"


@dataclass
class SimulationConfig:
    # Runtime/global simulation settings.
    dt: float = 0.01
    headless: bool = False
    controls_file: Optional[str] = None
    record_file: Optional[str] = None
    print_every_steps: int = 10
    use_parallel_collision: bool = False
    friction_mu: float = 0.2
    alarm_distance: float = 0.8
    contact_distance: float = 0.15
    collision_radius: float = 0.70
    vessel_file: str = f"{DEFAULT_VASCULATURE_DIR}/{DEFAULT_VESSEL_FILE}"
    vessel_alpha: float = 0.18
    show_control_window: bool = True
    control_state_file: Optional[str] = None

    # Catheter deployment start pose.
    starting_position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    starting_direction: Tuple[float, float, float] = (-1.0, 0.0, 0.0)

    # Shaft section.
    shaft_length: float = 980.0
    shaft_radius: float = 0.70
    shaft_young_modulus: float = 8000.0
    shaft_nb_beams: int = 220
    shaft_nb_edges_collis: int = 220
    shaft_nb_edges_visu: int = 180

    # Distal section/hook.
    tip_length: float = 3.30
    tip_radius: float = 0.35
    tip_young_modulus: float = 2500.0
    tip_nb_beams: int = 8
    tip_nb_edges_collis: int = 8
    tip_nb_edges_visu: int = 12
    hook_radius: float = 1.05
    tip_shape: str = "semicircle"
    tip_hook_radius_scale: Optional[float] = 1.5
    tip_length_auto: bool = True

    # Material and control increments.
    poisson_ratio: float = 0.49
    mass_density: float = 1.0e-6
    insertion_step: float = 0.5
    rotation_step: float = 0.035
    initial_xtip: float = 0.0

    # Derived metadata.
    auto_start_animation: bool = False
    catheter_profile_file: str = ""


def scene_dir() -> Path:
    """Return the directory where the simulation scene file lives."""
    return Path(__file__).resolve().parent


def to_abs_path(path_str: str) -> str:
    """Resolve a potentially relative path against the scene directory."""
    p = Path(path_str)
    if p.is_absolute():
        return str(p)
    return str((scene_dir() / p).resolve())


def resolve_with_default_search(path_str: str, default_subdir: str) -> str:
    """Resolve a path, searching default asset folder first for bare filenames.

    Search order for relative inputs:
    1) <scene_dir>/<default_subdir>/<filename>   (only when input has no folder)
    2) <scene_dir>/<input_path>
    """
    p = Path(path_str)
    if p.is_absolute():
        return str(p)
    candidates: List[Path] = []
    if p.parent == Path("."):
        candidates.append((scene_dir() / default_subdir / p.name).resolve())
    candidates.append((scene_dir() / p).resolve())
    for cand in candidates:
        if cand.exists():
            return str(cand)
    # Return preferred location for clear error messages by caller.
    return str(candidates[0])


def _strip_yaml_inline_comment(raw: str) -> str:
    out: List[str] = []
    quote: Optional[str] = None
    for ch in raw:
        if ch in ("'", '"'):
            if quote is None:
                quote = ch
            elif quote == ch:
                quote = None
            out.append(ch)
            continue
        if ch == "#" and quote is None:
            break
        out.append(ch)
    return "".join(out).rstrip()


def _parse_yaml_scalar(token: str) -> Any:
    t = token.strip()
    if not t:
        return ""
    if (t.startswith('"') and t.endswith('"')) or (t.startswith("'") and t.endswith("'")):
        return t[1:-1]
    low = t.lower()
    if low in ("null", "none", "~"):
        return None
    if low in ("true", "false"):
        return low == "true"
    try:
        if any(c in t for c in (".", "e", "E")):
            return float(t)
        return int(t)
    except ValueError:
        return t


def _load_simple_yaml(path: str) -> Dict[str, Any]:
    """Parse a minimal YAML subset (nested mappings with scalar values)."""
    root: Dict[str, Any] = {}
    stack: List[Tuple[int, Dict[str, Any]]] = [(-1, root)]
    with open(path, "r", encoding="utf-8") as f:
        for line_number, raw in enumerate(f, start=1):
            line = _strip_yaml_inline_comment(raw.rstrip("\n"))
            if not line.strip():
                continue
            if "\t" in line:
                raise ValueError(f"Invalid tab indentation in YAML at {path}:{line_number}")
            indent = len(line) - len(line.lstrip(" "))
            if indent % 2 != 0:
                raise ValueError(f"Indentation must use multiples of 2 spaces in {path}:{line_number}")
            content = line.lstrip(" ")
            if ":" not in content:
                raise ValueError(f"Expected 'key: value' YAML entry in {path}:{line_number}")

            key, _, remainder = content.partition(":")
            key = key.strip()
            if not key:
                raise ValueError(f"Empty key in YAML at {path}:{line_number}")
            value_text = remainder.strip()

            while stack and indent <= stack[-1][0]:
                stack.pop()
            if not stack:
                raise ValueError(f"Invalid indentation structure in YAML at {path}:{line_number}")
            parent = stack[-1][1]

            if value_text == "":
                child: Dict[str, Any] = {}
                parent[key] = child
                stack.append((indent, child))
            else:
                parent[key] = _parse_yaml_scalar(value_text)
    return root


def _lookup_first(data: Dict[str, Any], paths: List[Tuple[str, ...]]) -> Tuple[bool, Any]:
    for path in paths:
        node: Any = data
        found = True
        for key in path:
            if not isinstance(node, dict) or key not in node:
                found = False
                break
            node = node[key]
        if found:
            return True, node
    return False, None


def _to_float(value: Any, label: str, source_path: str) -> float:
    try:
        return float(value)
    except Exception as exc:
        raise ValueError(f"Invalid float for '{label}' in catheter file '{source_path}': {value!r}") from exc


def _to_int(value: Any, label: str, source_path: str, minimum: int = 1) -> int:
    try:
        out = int(value)
    except Exception as exc:
        raise ValueError(f"Invalid int for '{label}' in catheter file '{source_path}': {value!r}") from exc
    return max(minimum, out)


def _apply_catheter_profile(cfg: SimulationConfig, profile: Dict[str, Any], source_path: str) -> None:
    """Copy profile values into SimulationConfig with lightweight validation."""
    if not isinstance(profile, dict):
        raise ValueError(f"Catheter file '{source_path}' must contain a mapping/object at the root.")

    found, value = _lookup_first(profile, [("shaft", "length"), ("shaft_length",)])
    if found:
        cfg.shaft_length = _to_float(value, "shaft.length", source_path)
    found, value = _lookup_first(profile, [("shaft", "radius"), ("shaft_radius",)])
    if found:
        cfg.shaft_radius = _to_float(value, "shaft.radius", source_path)
    found, value = _lookup_first(profile, [("shaft", "young_modulus"), ("shaft_young_modulus",)])
    if found:
        cfg.shaft_young_modulus = _to_float(value, "shaft.young_modulus", source_path)
    found, value = _lookup_first(profile, [("shaft", "nb_beams"), ("shaft_nb_beams",)])
    if found:
        cfg.shaft_nb_beams = _to_int(value, "shaft.nb_beams", source_path)
    found, value = _lookup_first(profile, [("shaft", "nb_edges_collis"), ("shaft_nb_edges_collis",)])
    if found:
        cfg.shaft_nb_edges_collis = _to_int(value, "shaft.nb_edges_collis", source_path)
    found, value = _lookup_first(profile, [("shaft", "nb_edges_visu"), ("shaft_nb_edges_visu",)])
    if found:
        cfg.shaft_nb_edges_visu = _to_int(value, "shaft.nb_edges_visu", source_path)

    found_tip_beams, value = _lookup_first(profile, [("tip", "nb_beams"), ("tip_nb_beams",)])
    if found_tip_beams:
        cfg.tip_nb_beams = _to_int(value, "tip.nb_beams", source_path)
    found_tip_collis, value = _lookup_first(profile, [("tip", "nb_edges_collis"), ("tip_nb_edges_collis",)])
    if found_tip_collis:
        cfg.tip_nb_edges_collis = _to_int(value, "tip.nb_edges_collis", source_path)
    found_tip_visu, value = _lookup_first(profile, [("tip", "nb_edges_visu"), ("tip_nb_edges_visu",)])
    if found_tip_visu:
        cfg.tip_nb_edges_visu = _to_int(value, "tip.nb_edges_visu", source_path)
    if found_tip_beams and (not found_tip_collis):
        cfg.tip_nb_edges_collis = cfg.tip_nb_beams
    if found_tip_beams and (not found_tip_visu):
        cfg.tip_nb_edges_visu = max(cfg.tip_nb_beams, 12)

    found, value = _lookup_first(profile, [("tip", "radius"), ("tip_radius",)])
    if found:
        cfg.tip_radius = _to_float(value, "tip.radius", source_path)
    found, value = _lookup_first(profile, [("tip", "young_modulus"), ("tip_young_modulus",)])
    if found:
        cfg.tip_young_modulus = _to_float(value, "tip.young_modulus", source_path)
    found, value = _lookup_first(profile, [("tip", "shape"), ("tip_shape",)])
    if found:
        cfg.tip_shape = str(value).strip().lower()

    found, value = _lookup_first(profile, [("tip", "hook_radius_scale"), ("tip_hook_radius_scale",)])
    if found and value is not None:
        cfg.tip_hook_radius_scale = _to_float(value, "tip.hook_radius_scale", source_path)
    found, value = _lookup_first(profile, [("tip", "hook_radius"), ("hook_radius",)])
    if found:
        if value is None or (isinstance(value, str) and value.strip().lower() in ("auto", "none", "null")):
            pass
        else:
            cfg.hook_radius = _to_float(value, "tip.hook_radius", source_path)
            cfg.tip_hook_radius_scale = None

    found, value = _lookup_first(profile, [("tip", "length"), ("tip_length",)])
    if found:
        if isinstance(value, str) and value.strip().lower() in ("auto", "none", "null"):
            cfg.tip_length_auto = True
        else:
            cfg.tip_length = _to_float(value, "tip.length", source_path)
            cfg.tip_length_auto = False

    found, value = _lookup_first(profile, [("material", "poisson_ratio"), ("poisson_ratio",)])
    if found:
        cfg.poisson_ratio = _to_float(value, "material.poisson_ratio", source_path)
    found, value = _lookup_first(profile, [("material", "mass_density"), ("mass_density",)])
    if found:
        cfg.mass_density = _to_float(value, "material.mass_density", source_path)

    found, value = _lookup_first(profile, [("collision", "radius"), ("collision_radius",)])
    if found:
        cfg.collision_radius = _to_float(value, "collision.radius", source_path)


def parse_args(argv: Optional[List[str]] = None) -> SimulationConfig:
    """Parse simulation CLI arguments and return a fully-populated config."""
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--controls", type=str, default=None)
    parser.add_argument("--record", type=str, default=None)
    parser.add_argument("--catheter", type=str, default=None)
    parser.add_argument("--vessel", type=str, default=DEFAULT_VESSEL_FILE)
    parser.add_argument("--vessel-alpha", type=float, default=0.18)
    parser.add_argument("--no-control-window", action="store_true")
    parser.add_argument("--control-state-file", type=str, default=None)
    parser.add_argument("--no-mouse-ui", action="store_true")
    parser.add_argument("--dt", type=float, default=0.01)
    parser.add_argument("--friction", type=float, default=0.2)
    parser.add_argument("--contact-distance", type=float, default=0.15)
    parser.add_argument("--alarm-distance", type=float, default=0.8)
    parser.add_argument("--print-every", type=int, default=10)
    parser.add_argument("--parallel-collision", action="store_true")
    parser.add_argument("--collision-radius", type=float, default=None)
    parser.add_argument("--tip-length", type=float, default=None)
    parser.add_argument("--tip-radius", type=float, default=None)
    parser.add_argument("--tip-beams", type=int, default=None)
    parser.add_argument("--hook-radius", type=float, default=None)
    parser.add_argument("--insertion-step", type=float, default=None)
    parser.add_argument("--rotation-step", type=float, default=None)

    # runSofa forwards Python args as a single string via --argv.
    # Split each item so both direct Python runs and runSofa forwarding work.
    raw_args = sys.argv[1:] if argv is None else argv
    forwarded_args: List[str] = []
    for raw in raw_args:
        forwarded_args.extend(shlex.split(raw))
    args, _ = parser.parse_known_args(forwarded_args)

    cfg = SimulationConfig()

    # 1) Load catheter profile defaults first.
    default_catheter_file = str((scene_dir() / DEFAULT_CATHETER_DIR / DEFAULT_CATHETER_FILE).resolve())
    catheter_file = (
        resolve_with_default_search(args.catheter, DEFAULT_CATHETER_DIR)
        if args.catheter
        else default_catheter_file
    )
    if not os.path.exists(catheter_file):
        raise FileNotFoundError(f"Catheter file does not exist: {catheter_file}")
    catheter_profile = _load_simple_yaml(catheter_file)
    _apply_catheter_profile(cfg, catheter_profile, catheter_file)
    cfg.catheter_profile_file = catheter_file

    # 2) Apply CLI overrides.
    cfg.headless = bool(args.headless)
    cfg.controls_file = to_abs_path(args.controls) if args.controls else None
    cfg.record_file = to_abs_path(args.record) if args.record else None
    cfg.vessel_file = resolve_with_default_search(args.vessel, DEFAULT_VASCULATURE_DIR)
    cfg.vessel_alpha = max(0.0, min(1.0, float(args.vessel_alpha)))
    cfg.show_control_window = not (bool(args.no_control_window) or bool(args.no_mouse_ui))
    if args.control_state_file:
        cfg.control_state_file = to_abs_path(args.control_state_file)
    else:
        cfg.control_state_file = os.path.join("/tmp", f"catheter_controls_{os.getpid()}.txt")
    cfg.dt = float(args.dt)
    cfg.friction_mu = float(args.friction)
    cfg.contact_distance = float(args.contact_distance)
    cfg.alarm_distance = float(args.alarm_distance)
    cfg.print_every_steps = max(1, int(args.print_every))
    cfg.use_parallel_collision = bool(args.parallel_collision)
    if args.collision_radius is not None:
        cfg.collision_radius = float(args.collision_radius)

    user_set_tip_length = args.tip_length is not None
    if user_set_tip_length:
        cfg.tip_length = float(args.tip_length)
        cfg.tip_length_auto = False
    if args.tip_radius is not None:
        cfg.tip_radius = float(args.tip_radius)
    if args.tip_beams is not None:
        cfg.tip_nb_beams = max(1, int(args.tip_beams))
        cfg.tip_nb_edges_collis = cfg.tip_nb_beams
        cfg.tip_nb_edges_visu = max(cfg.tip_nb_beams, 12)

    user_set_hook_radius = args.hook_radius is not None
    if args.hook_radius is not None:
        cfg.hook_radius = float(args.hook_radius)
        cfg.tip_hook_radius_scale = None

    # Keep automatic semicircle behavior unless the user explicitly overrides it.
    if cfg.tip_shape == "semicircle":
        if (not user_set_hook_radius) and (cfg.tip_hook_radius_scale is not None):
            cfg.hook_radius = float(cfg.tip_hook_radius_scale) * cfg.shaft_radius
        if (not user_set_tip_length) and cfg.tip_length_auto:
            cfg.tip_length = math.pi * cfg.hook_radius

    if args.insertion_step is not None:
        cfg.insertion_step = float(args.insertion_step)
    if args.rotation_step is not None:
        cfg.rotation_step = float(args.rotation_step)

    # Match runSofa "-i" behavior for normal user sessions:
    # when not headless and not replaying prerecorded controls, start animating.
    cfg.auto_start_animation = (not cfg.headless) and (cfg.controls_file is None)
    return cfg
