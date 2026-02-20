from pathlib import Path
import xml.etree.ElementTree as ET
import random
import math
import re
import os

def get_base_world_name() -> str:
    """Get base world name passed by UI launch."""
    base_world_name = os.environ.get("DROBOT_BASE_WORLD_NAME")
    if not base_world_name:
        raise RuntimeError(
            "DROBOT_BASE_WORLD_NAME is not set. "
            "Run world_generator.py from ui.launch.py Create World flow."
        )
    
    print(base_world_name)
    return base_world_name

BASE_WORLD_NAME = get_base_world_name()

def get_worlds_root() -> Path:
    """Return worlds directory for this package."""
    return Path(__file__).resolve().parent.parent / "worlds"


def get_next_output_world_name(base_world_name: str, search_dir: Path | None = None) -> str:
    base = Path(base_world_name)
    stem = base.stem
    ext = base.suffix if base.suffix else ".sdf"

    root = Path(search_dir) if search_dir else get_worlds_root()
    if not root.exists():
        return f"{stem}_1{ext}"

    regex = re.compile(rf"^{re.escape(stem)}_(\d+){re.escape(ext)}$")
    max_index = 0

    for world_file in root.rglob(f"{stem}_*{ext}"):
        if not world_file.is_file():
            continue
        match = regex.match(world_file.name)
        if not match:
            continue
        max_index = max(max_index, int(match.group(1)))

    return f"{stem}_{max_index + 1}{ext}"


OUTPUT_WORLD_NAME = get_next_output_world_name(BASE_WORLD_NAME)


def get_base_goals() -> str:
    """Get base goals from base world."""
    base_goals = os.environ.get("DROBOT_BASE_GOALS")
    if not base_goals:
        raise RuntimeError(
            "DROBOT_BASE_GOALS is not set. "
            "Select a goal in ui.launch.py before Create World."
        )
    print(base_goals)
    return base_goals

BASE_GOALS = get_base_goals()

def get_base_options() -> str:
    """Get base options from base world."""
    base_options = os.environ.get("DROBOT_BASE_OPTIONS")
    if not base_options:
        raise RuntimeError(
            "DROBOT_BASE_OPTIONS is not set. "
            "Select an option in ui.launch.py before Create World."
        )
    print(base_options)
    return base_options


BASE_OPTIONS = get_base_options()
