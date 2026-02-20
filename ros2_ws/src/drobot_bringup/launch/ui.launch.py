#!/usr/bin/env python3
"""
UI launch file.

Usage:
  ros2 launch drobot_bringup ui.launch.py
"""

import sys
import os
import subprocess
from pathlib import Path
import tkinter as tk
from tkinter import ttk

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def load_world_files(worlds_root: Path):
    """Return sorted world file paths relative to worlds_root."""
    if not worlds_root.exists():
        return []

    world_files = []
    for pattern in ("**/*.sdf", "**/*.world"):
        world_files.extend(worlds_root.glob(pattern))

    return sorted(
        p.relative_to(worlds_root).as_posix()
        for p in world_files
        if p.is_file()
    )


def resolve_worlds_root() -> Path:
    """Resolve drobot worlds path in install/source environments."""
    try:
        return Path(get_package_share_directory("drobot_description")) / "worlds" / "original"
    except Exception:
        # Fallback for direct source execution
        return Path(__file__).resolve().parents[3] / "src" / "drobot_description" / "worlds" / "original"


def resolve_world_generator_script() -> Path:
    """Resolve world_generator.py in install/source environments."""
    try:
        return Path(get_package_share_directory("drobot_description")) / "scripts" / "world_generator.py"
    except Exception:
        return Path(__file__).resolve().parents[3] / "src" / "drobot_description" / "scripts" / "world_generator.py"


def run_ui():
    """Run Tkinter UI app."""
    window_width = 1320
    window_height = 560
    header_pady = (0, 10)
    value_pady = (0, 10)
    button_pady = 3

    root = tk.Tk()
    root.title("Start UI")
    root.geometry(f"{window_width}x{window_height}")
    root.resizable(False, False)

    root.grid_rowconfigure(0, weight=1)
    root.grid_columnconfigure(0, weight=0, minsize=300)  # worlds
    root.grid_columnconfigure(1, weight=0, minsize=8)    # sep1
    root.grid_columnconfigure(2, weight=0, minsize=0)    # unused spacer
    root.grid_columnconfigure(3, weight=0, minsize=200)  # goals
    root.grid_columnconfigure(4, weight=0, minsize=8)    # sep2
    root.grid_columnconfigure(5, weight=0, minsize=220)  # options
    root.grid_columnconfigure(6, weight=0, minsize=584)  # summary

    worlds_root = resolve_worlds_root()
    world_files = load_world_files(worlds_root)
    world_generator_script = resolve_world_generator_script()

    worlds1 = ttk.Frame(root, padding="10")
    worlds1.grid(row=0, column=0, sticky="nsew")
    worlds1.grid_propagate(False)
    ttk.Label(worlds1, text="Worlds").pack(anchor="w", pady=header_pady)

    summary_col = ttk.Frame(root)
    summary_col.grid(row=0, column=6, sticky="nsew")
    summary_col.grid_propagate(False)
    summary_col.grid_rowconfigure(0, weight=1)
    summary_col.grid_columnconfigure(1, weight=1)

    # Thicker divider between the last column and the rest of the UI.
    summary_sep = tk.Frame(summary_col, width=4, bg="#808080")
    summary_sep.grid(row=0, column=0, sticky="ns")

    summary_content = ttk.Frame(summary_col, padding="10")
    summary_content.grid(row=0, column=1, sticky="nsew")
    ttk.Label(summary_content, text="Selection").pack(anchor="w", pady=header_pady)

    selected_world = tk.StringVar(value="(none)")
    selected_goal = tk.StringVar(value="(none)")
    selected_option = tk.StringVar(value="(none)")
    ttk.Label(summary_content, text="Selected:").pack(anchor="w")
    ttk.Label(summary_content, textvariable=selected_world).pack(anchor="w", pady=value_pady)
    ttk.Label(summary_content, text="Goal:").pack(anchor="w")
    ttk.Label(summary_content, textvariable=selected_goal).pack(anchor="w", pady=value_pady)
    ttk.Label(summary_content, text="Option:").pack(anchor="w")
    ttk.Label(summary_content, textvariable=selected_option).pack(anchor="w", pady=value_pady)

    next_btn = ttk.Button(summary_content, text="Next")

    def create_world():
        base_world_name = selected_world.get()
        base_goal = selected_goal.get()
        base_option = selected_option.get()
        if base_world_name == "(none)":
            print("Create World skipped: no world selected")
            return
        if base_goal == "(none)":
            print("Create World skipped: no goal selected")
            return
        if base_option == "(none)":
            print("Create World skipped: no option selected")
            return
        if not world_generator_script.is_file():
            print(f"Create World skipped: script not found: {world_generator_script}")
            return

        env = os.environ.copy()
        env["DROBOT_BASE_WORLD_NAME"] = base_world_name
        env["DROBOT_BASE_GOALS"] = base_goal
        env["DROBOT_BASE_OPTIONS"] = base_option
        subprocess.Popen([sys.executable, str(world_generator_script)], env=env)
        print(
            "Running world_generator.py with "
            f"DROBOT_BASE_WORLD_NAME={base_world_name} "
            f"DROBOT_BASE_GOALS={base_goal} "
            f"DROBOT_BASE_OPTIONS={base_option}"
        )

    create_world_btn = ttk.Button(summary_content, command=create_world, text="Create World")



    goals_col = ttk.Frame(root, padding="10")
    goals_col.grid_propagate(False)
    ttk.Label(goals_col, text="Goals").pack(anchor="w", pady=header_pady)
    goals_buttons = ttk.Frame(goals_col)
    goals_buttons.pack(fill="both", expand=True)

    for goal_name in ["0", "1", "2", "3", "random"]:
        ttk.Button(
            goals_buttons,
            text=goal_name,
            command=lambda name=goal_name: on_select_goal(name)
        ).pack(fill="x", pady=button_pady)

    options_col = ttk.Frame(root, padding="10")
    options_col.grid_propagate(False)
    ttk.Label(options_col, text="Options").pack(anchor="w", pady=header_pady)
    options_buttons = ttk.Frame(options_col)
    options_buttons.pack(fill="both", expand=True)

    def on_select_option(option_name: str):
        selected_option.set(option_name)
        print(f"Selected option: {option_name}")
        next_btn.pack_forget()
        if not create_world_btn.winfo_ismapped():
            create_world_btn.pack(anchor="w", pady=(button_pady, 0))

    sep2 = ttk.Separator(root, orient="vertical")
    sep2.grid(row=0, column=4, sticky="ns")

    def reset_option_step():
        """Clear selected option and force option re-selection."""
        selected_option.set("(none)")
        create_world_btn.pack_forget()
        if options_col.winfo_ismapped():
            options_col.grid_remove()

    for option_name in ["moving", "stationary", "random"]:
        ttk.Button(
            options_buttons,
            text=option_name,
            command=lambda name=option_name: on_select_option(name)
        ).pack(fill="x", pady=button_pady)

    def on_next_pressed():
        # Step 1: show goals. Step 2: show options.
        if not goals_col.winfo_ismapped():
            goals_col.grid(row=0, column=3, sticky="nsew")
            next_btn.pack_forget()
            return

        if not options_col.winfo_ismapped():
            options_col.grid(row=0, column=5, sticky="nsew")
            next_btn.pack_forget()

    next_btn.configure(command=on_next_pressed)

    def on_select_world(world_name: str):
        selected_world.set(world_name)
        print(f"Selected world: {world_name}")
        selected_goal.set("(none)")
        reset_option_step()

        if goals_col.winfo_ismapped():
            goals_col.grid_remove()
        
        if not next_btn.winfo_ismapped() and not create_world_btn.winfo_ismapped():
            next_btn.pack(anchor="w", pady=(button_pady, 0))

    def on_select_goal(goal_name: str):
        selected_goal.set(goal_name)
        print(f"Selected goal: {goal_name}")
        reset_option_step()

        if not next_btn.winfo_ismapped():
            next_btn.pack(anchor="w", pady=(4, 0))
    
    worlds_list = ttk.Frame(worlds1)
    worlds_list.pack(fill="both", expand=True)

    if world_files:
        for world_name in world_files:
            ttk.Button(
                worlds_list,
                text=world_name,
                command=lambda name=world_name: on_select_world(name)
            ).pack(fill="x", pady=button_pady)
    else:
        ttk.Label(
            worlds_list,
            text=f"No world files found under:\n{worlds_root}",
            justify="left"
        ).pack(anchor="w")

    sep1 = ttk.Separator(root, orient="vertical")
    sep1.grid(row=0, column=1, sticky="ns")

    root.mainloop()


def generate_launch_description():
    """Launch UI in a child Python process."""
    this_file = str(Path(__file__).resolve())
    return LaunchDescription([
        ExecuteProcess(
            cmd=[sys.executable, this_file, "--run-ui"],
            output="screen",
        ),
    ])


if __name__ == "__main__":
    if "--run-ui" in sys.argv:
        run_ui()
