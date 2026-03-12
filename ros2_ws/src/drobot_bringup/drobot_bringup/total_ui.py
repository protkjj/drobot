"""
Drobot Total UI
PX4 SITL + DDS Agent + Navigation + Teleop + Perception 통합 실행 GUI
"""

import sys
import os
import signal
import subprocess
import shutil
import tempfile
import atexit
import yaml
from pathlib import Path
import tkinter as tk
from tkinter import ttk

from ament_index_python.packages import get_package_share_directory

_spawned_procs: list[subprocess.Popen] = []
_shutdown_requested = False

PX4_DIR = Path.home() / "Desktop" / "drobot" / "PX4-Autopilot"


# ============================================================
# Process Management
# ============================================================

def _kill_all_processes():
    patterns = [
        "gzserver", "gzclient", "gz sim", "ign gazebo",
        "parameter_bridge", "px4_sitl", "MicroXRCEAgent",
        "ros2 launch drobot_bringup navigation.launch.py",
        "ros2 launch drobot_bringup perception.launch.py",
        "ros2 run drobot_controller teleop_keyboard",
    ]
    for pattern in patterns:
        try:
            subprocess.run(["pkill", "-f", pattern], capture_output=True, timeout=5)
        except Exception:
            pass


def _cleanup():
    for proc in _spawned_procs:
        if proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except (ProcessLookupError, PermissionError, OSError):
                pass

    _kill_all_processes()

    import time
    time.sleep(2)

    for proc in _spawned_procs:
        if proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError, OSError):
                pass

    for pattern in ["gzserver", "gzclient", "gz sim", "ign gazebo", "px4", "MicroXRCEAgent"]:
        try:
            subprocess.run(["pkill", "-9", "-f", pattern], capture_output=True, timeout=5)
        except Exception:
            pass

    try:
        subprocess.run(
            ["pkill", "-9", "-f", "gz|rviz|nav2|slam|ekf|goal"],
            capture_output=True, timeout=5,
        )
    except Exception:
        pass

    _spawned_procs.clear()


# ============================================================
# Path Resolvers
# ============================================================

def get_spawn_position(bringup_pkg, world_name):
    spawn_file = os.path.join(bringup_pkg, 'config', 'spawn_positions.yaml')
    try:
        with open(spawn_file, 'r') as f:
            data = yaml.safe_load(f)
        worlds = data.get('worlds', {})
        pos = worlds.get(world_name, data.get('default', {'x': 0.0, 'y': 0.0}))
        return str(pos['x']), str(pos['y'])
    except Exception:
        return '0.0', '0.0'


def _resolve_worlds_root() -> Path:
    try:
        return Path(get_package_share_directory("drobot_description")) / "worlds" / "original"
    except Exception:
        return Path(__file__).resolve().parents[2] / "src" / "drobot_description" / "worlds" / "original"


def _resolve_world_generator_script() -> Path:
    try:
        return Path(get_package_share_directory("drobot_description")) / "scripts" / "world_generator.py"
    except Exception:
        return Path(__file__).resolve().parents[2] / "src" / "drobot_description" / "scripts" / "world_generator.py"


def _resolve_workspace_dir() -> Path:
    try:
        share_dir = Path(get_package_share_directory("drobot_bringup")).resolve()
        for parent in share_dir.parents:
            if parent.name == "install":
                return parent.parent
    except Exception:
        pass
    return Path(__file__).resolve().parents[2]


def _resolve_bringup_pkg() -> str:
    try:
        return get_package_share_directory('drobot_bringup')
    except Exception:
        return str(Path(__file__).resolve().parents[1])


def _load_world_files(worlds_root: Path) -> list[str]:
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


def _load_option_launch_files(bringup_pkg: str) -> list[str]:
    launch_dir = Path(bringup_pkg) / "launch"
    if not launch_dir.exists():
        return []
    return sorted(
        p.name for p in launch_dir.glob("*.launch.py")
        if p.is_file() and p.name == "perception.launch.py"
    )


# ============================================================
# Terminal Launchers
# ============================================================

def _launch_in_terminator(commands: list[tuple[str, str]]):
    n = len(commands)
    if n == 0:
        return

    script_paths = []
    for title, cmd in commands:
        fd, path = tempfile.mkstemp(suffix=".sh", prefix="drobot_")
        with os.fdopen(fd, "w") as f:
            f.write("#!/bin/bash -l\n")
            f.write(f"echo '=== {title} ==='\n")
            f.write(f"{cmd}\n")
            f.write("exec bash\n")
        os.chmod(path, 0o755)
        script_paths.append(path)

    cfg = [
        "[global_config]", "  dbus = False",
        "  suppress_multiple_term_dialog = True",
        "[keybindings]", "[profiles]",
    ]
    for i in range(n):
        profile_name = "default" if i == 0 else f"drobot_{i}"
        cfg += [f"  [[{profile_name}]]", "    use_custom_command = True",
                f"    custom_command = {script_paths[i]}"]

    cfg += ["[layouts]", "  [[default]]", "    [[[window0]]]",
            "      type = Window", '      parent = ""']

    if n == 1:
        cfg += ["    [[[term0]]]", "      type = Terminal",
                "      parent = window0", "      profile = default"]
    elif n == 2:
        cfg += ["    [[[child1]]]", "      type = HPaned", "      parent = window0"]
        for i in range(2):
            p = "default" if i == 0 else f"drobot_{i}"
            cfg += [f"    [[[term{i}]]]", "      type = Terminal",
                    "      parent = child1", f"      profile = {p}"]
    elif n == 3:
        cfg += ["    [[[child1]]]", "      type = VPaned", "      parent = window0",
                "    [[[child2]]]", "      type = HPaned", "      parent = child1"]
        for i in range(2):
            p = "default" if i == 0 else f"drobot_{i}"
            cfg += [f"    [[[term{i}]]]", "      type = Terminal",
                    "      parent = child2", f"      profile = {p}"]
        cfg += ["    [[[term2]]]", "      type = Terminal",
                "      parent = child1", "      profile = drobot_2"]
    else:  # 4+: 2x2 grid
        cfg += ["    [[[child1]]]", "      type = VPaned", "      parent = window0",
                "    [[[child2]]]", "      type = HPaned", "      parent = child1",
                "    [[[child3]]]", "      type = HPaned", "      parent = child1"]
        for i in range(min(n, 4)):
            parent = "child2" if i < 2 else "child3"
            p = "default" if i == 0 else f"drobot_{i}"
            cfg += [f"    [[[term{i}]]]", "      type = Terminal",
                    f"      parent = {parent}", f"      profile = {p}"]
        for i in range(4, n):
            cfg += [f"    [[[term{i}]]]", "      type = Terminal",
                    "      parent = child3", f"      profile = drobot_{i}"]

    cfg.append("[plugins]")

    fd, config_path = tempfile.mkstemp(suffix=".cfg", prefix="drobot_terminator_")
    with os.fdopen(fd, "w") as f:
        f.write("\n".join(cfg) + "\n")

    proc = subprocess.Popen(
        ["terminator", "-g", config_path, "--maximise"],
        start_new_session=True,
    )
    _spawned_procs.append(proc)


def _run_in_new_terminal(launch_cmd: str):
    terminal = None
    for candidate in ("gnome-terminal", "konsole", "xfce4-terminal", "xterm"):
        if shutil.which(candidate):
            terminal = candidate
            break

    if terminal == "gnome-terminal":
        proc = subprocess.Popen(
            [terminal, "--", "bash", "-lc", f"{launch_cmd}; exec bash"],
            start_new_session=True,
        )
    elif terminal:
        proc = subprocess.Popen(
            [terminal, "-e", "bash", "-lc", f"{launch_cmd}; exec bash"],
            start_new_session=True,
        )
    else:
        proc = subprocess.Popen(["bash", "-lc", launch_cmd], start_new_session=True)
    _spawned_procs.append(proc)


# ============================================================
# Launch Stack Builder
# ============================================================

def _build_commands(world_arg: str, model_pose: str,
                    workspace_dir: Path, selected_options: list[str]) -> list[tuple[str, str]]:
    commands = []

    # 1. PX4 SITL
    commands.append(("PX4 SITL", (
        f"cd {PX4_DIR} && "
        f"PX4_GZ_MODEL_POSE='{model_pose}' PX4_GZ_WORLD='{world_arg}' "
        f"make px4_sitl gz_drobot"
    )))

    # 2. DDS Agent
    commands.append(("DDS Agent", "sleep 5 && MicroXRCEAgent udp4 -p 8888"))

    # 3. Navigation
    commands.append(("Navigation", (
        f"cd {workspace_dir} && source install/setup.bash && "
        f"sleep 10 && ros2 launch drobot_bringup navigation.launch.py world:={world_arg}"
    )))

    # 4. Teleop
    commands.append(("Teleop", (
        f"cd {workspace_dir} && source install/setup.bash && "
        "sleep 12 && ros2 run drobot_controller teleop_keyboard"
    )))

    # 5. Perception (optional)
    if "perception.launch.py" in selected_options:
        commands.append(("Perception", (
            f"cd {workspace_dir} && source install/setup.bash && "
            "sleep 12 && ros2 launch drobot_bringup perception.launch.py"
        )))

    return commands


# ============================================================
# Tkinter GUI
# ============================================================

def run_ui():
    atexit.register(_cleanup)

    def _request_shutdown(signum=None, frame=None):
        global _shutdown_requested
        _shutdown_requested = True

    signal.signal(signal.SIGTERM, _request_shutdown)
    signal.signal(signal.SIGINT, _request_shutdown)

    # Resolve paths
    worlds_root = _resolve_worlds_root()
    world_files = _load_world_files(worlds_root)
    world_generator_script = _resolve_world_generator_script()
    workspace_dir = _resolve_workspace_dir()
    bringup_pkg = _resolve_bringup_pkg()
    option_files = _load_option_launch_files(bringup_pkg)

    # Window
    root = tk.Tk()
    root.title("Drobot Total Launch")
    root.geometry("1320x560")
    root.resizable(False, False)

    def _poll_shutdown():
        if _shutdown_requested:
            _cleanup()
            root.destroy()
            return
        root.after(200, _poll_shutdown)
    root.after(200, _poll_shutdown)

    root.grid_rowconfigure(0, weight=1)
    root.grid_columnconfigure(0, weight=0, minsize=300)
    root.grid_columnconfigure(1, weight=0, minsize=8)
    root.grid_columnconfigure(2, weight=0, minsize=0)
    root.grid_columnconfigure(3, weight=0, minsize=200)
    root.grid_columnconfigure(4, weight=0, minsize=8)
    root.grid_columnconfigure(5, weight=0, minsize=220)
    root.grid_columnconfigure(6, weight=0, minsize=584)

    hpad = (0, 10)
    vpad = (0, 10)
    bpad = 3

    # --- Worlds ---
    worlds_frame = ttk.Frame(root, padding="10")
    worlds_frame.grid(row=0, column=0, sticky="nsew")
    worlds_frame.grid_propagate(False)
    ttk.Label(worlds_frame, text="Worlds").pack(anchor="w", pady=hpad)

    # --- Summary ---
    summary_col = ttk.Frame(root)
    summary_col.grid(row=0, column=6, sticky="nsew")
    summary_col.grid_propagate(False)
    summary_col.grid_rowconfigure(0, weight=1)
    summary_col.grid_columnconfigure(1, weight=1)
    tk.Frame(summary_col, width=4, bg="#808080").grid(row=0, column=0, sticky="ns")

    summary = ttk.Frame(summary_col, padding="10")
    summary.grid(row=0, column=1, sticky="nsew")
    ttk.Label(summary, text="Selection").pack(anchor="w", pady=hpad)

    sel_world = tk.StringVar(value="(none)")
    sel_goal = tk.StringVar(value="(none)")
    sel_option = tk.StringVar(value="(none)")
    for label, var in [("Selected:", sel_world), ("Goal:", sel_goal), ("Option:", sel_option)]:
        ttk.Label(summary, text=label).pack(anchor="w")
        ttk.Label(summary, textvariable=var).pack(anchor="w", pady=vpad)

    next_btn = ttk.Button(summary, text="Next")

    # --- Launch handler ---
    def launch_stack(output_world_name: str, selected_options: list[str]):
        world_arg = Path(output_world_name).stem
        spawn_x, spawn_y = get_spawn_position(bringup_pkg, world_arg)
        model_pose = f"{spawn_x},{spawn_y},0.15,0,0,0"
        commands = _build_commands(world_arg, model_pose, workspace_dir, selected_options)
        print(f"Launching world: {world_arg} (pose: {model_pose})", flush=True)
        if shutil.which("terminator"):
            _launch_in_terminator(commands)
        else:
            for _, cmd in commands:
                _run_in_new_terminal(cmd)

    def parse_output_world_name(stdout_text: str) -> str | None:
        for line in stdout_text.splitlines():
            if line.startswith("OUTPUT_WORLD_NAME="):
                val = line.split("=", 1)[1].strip()
                if val:
                    return val
        return None

    def show_load_popup(base_name: str, output_name: str, opts: list[str]):
        popup = tk.Toplevel(root)
        popup.title("World Generated")
        popup.geometry("360x140")
        popup.resizable(False, False)
        popup.transient(root)
        popup.grab_set()
        ttk.Label(popup, text=f"Launching '{base_name}'.", padding=(12, 16, 12, 8)).pack(anchor="w")
        ttk.Button(popup, text="Load World",
                   command=lambda: (launch_stack(output_name, opts), popup.destroy())
                   ).pack(anchor="e", padx=12, pady=(0, 12))

    def create_world():
        world_name = sel_world.get()
        goal = sel_goal.get()
        opts = [name for name, var in option_vars.items() if var.get()]
        if "(none)" in (world_name, goal) or not opts:
            return
        if not world_generator_script.is_file():
            print(f"Script not found: {world_generator_script}")
            return

        env = os.environ.copy()
        env["DROBOT_WORKSPACE_DIR"] = str(workspace_dir)
        env["DROBOT_BASE_WORLD_NAME"] = world_name
        env["DROBOT_BASE_GOALS"] = goal
        env["DROBOT_BASE_OPTIONS"] = ",".join(opts)
        result = subprocess.run(
            [sys.executable, str(world_generator_script)],
            env=env, text=True, capture_output=True, check=False,
        )
        if result.returncode != 0:
            print("Create World failed")
            if result.stdout: print(result.stdout)
            if result.stderr: print(result.stderr)
            return

        output_name = parse_output_world_name(result.stdout)
        if not output_name:
            print("OUTPUT_WORLD_NAME not found")
            if result.stdout: print(result.stdout)
            return
        show_load_popup(world_name, output_name, opts)

    create_btn = ttk.Button(summary, command=create_world, text="Create World")

    # --- Goals ---
    goals_col = ttk.Frame(root, padding="10")
    goals_col.grid_propagate(False)
    ttk.Label(goals_col, text="Goals").pack(anchor="w", pady=hpad)
    goals_btns = ttk.Frame(goals_col)
    goals_btns.pack(fill="both", expand=True)
    for g in ["0", "1", "2", "3", "random"]:
        ttk.Button(goals_btns, text=g,
                   command=lambda name=g: on_select_goal(name)).pack(fill="x", pady=bpad)

    # --- Options ---
    options_col = ttk.Frame(root, padding="10")
    options_col.grid_propagate(False)
    ttk.Label(options_col, text="Options").pack(anchor="w", pady=hpad)
    options_btns = ttk.Frame(options_col)
    options_btns.pack(fill="both", expand=True)

    option_vars: dict[str, tk.BooleanVar] = {}

    def refresh_options():
        selected = [n for n, v in option_vars.items() if v.get()]
        if selected:
            sel_option.set(", ".join(selected))
            if not create_btn.winfo_ismapped():
                create_btn.pack(anchor="w", pady=(bpad, 0))
        else:
            sel_option.set("(none)")
            create_btn.pack_forget()
        next_btn.pack_forget()

    ttk.Separator(root, orient="vertical").grid(row=0, column=4, sticky="ns")

    def reset_options():
        sel_option.set("(none)")
        for v in option_vars.values():
            v.set(False)
        create_btn.pack_forget()
        if options_col.winfo_ismapped():
            options_col.grid_remove()

    for opt in option_files:
        var = tk.BooleanVar(value=False)
        option_vars[opt] = var
        ttk.Checkbutton(options_btns, text=opt, variable=var,
                        command=refresh_options).pack(fill="x", pady=bpad)

    def on_next():
        if not goals_col.winfo_ismapped():
            goals_col.grid(row=0, column=3, sticky="nsew")
            next_btn.pack_forget()
            return
        if not options_col.winfo_ismapped():
            options_col.grid(row=0, column=5, sticky="nsew")
            next_btn.pack_forget()

    next_btn.configure(command=on_next)

    def on_select_world(name: str):
        sel_world.set(name)
        sel_goal.set("(none)")
        reset_options()
        if goals_col.winfo_ismapped():
            goals_col.grid_remove()
        if not next_btn.winfo_ismapped() and not create_btn.winfo_ismapped():
            next_btn.pack(anchor="w", pady=(bpad, 0))

    def on_select_goal(name: str):
        sel_goal.set(name)
        reset_options()
        if not next_btn.winfo_ismapped():
            next_btn.pack(anchor="w", pady=(4, 0))

    worlds_list = ttk.Frame(worlds_frame)
    worlds_list.pack(fill="both", expand=True)
    for w in world_files:
        ttk.Button(worlds_list, text=w,
                   command=lambda name=w: on_select_world(name)).pack(fill="x", pady=bpad)

    ttk.Separator(root, orient="vertical").grid(row=0, column=1, sticky="ns")

    root.protocol("WM_DELETE_WINDOW", lambda: (_cleanup(), root.destroy()))
    root.mainloop()


def main():
    run_ui()
