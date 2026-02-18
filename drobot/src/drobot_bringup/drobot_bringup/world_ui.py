import subprocess
from pathlib import Path
import tkinter as tk
from tkinter import ttk, messagebox

from ament_index_python.packages import get_package_share_directory

from drobot_bringup.worldgen import write_world, default_output_path


class App:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Drobot World Launcher")
        self.root.geometry("720x420")

        # state
        self.mode = tk.StringVar(value="controller")
        self.num_obs = tk.IntVar(value=10)
        self.seed = tk.StringVar(value="")  # optional

        # world selection state
        self.world_source = tk.StringVar(value="generate")  # "generate" or "existing"
        self.selected_existing = tk.StringVar(value="")

        self.proc = None  # launch process handle

        # compute generated world directory in INSTALL share of drobot_description
        desc_share = Path(get_package_share_directory("drobot_description"))
        self.worlds_root = desc_share / "worlds"
        self.original_dir = self.worlds_root / "original"
        self.generated_dir = self.worlds_root / "generated"


        self._build_ui()

    # ---------- world list helpers ----------
    def _list_existing_worlds(self):
        # Existing maps live here
        self.original_dir.mkdir(parents=True, exist_ok=True)
        self.generated_dir.mkdir(parents=True, exist_ok=True)

        # Show original first, then generated (both .sdf and .world)
        originals = sorted(list(self.original_dir.glob("*.sdf")) + list(self.original_dir.glob("*.world")))
        generated = sorted(list(self.generated_dir.glob("*.sdf")) + list(self.generated_dir.glob("*.world")))

        # Return relative labels so UI shows where it came from
        out = []
        for p in originals:
            out.append(p.relative_to(self.worlds_root).as_posix())  # "original/foo.sdf"
        for p in generated:
            out.append(p.relative_to(self.worlds_root).as_posix())  # "generated/bar.sdf"
        return out


    def _refresh_world_dropdown(self):
        worlds = self._list_existing_worlds()
        self.existing_combo["values"] = worlds
        if worlds:
            if self.selected_existing.get() not in worlds:
                self.selected_existing.set(worlds[0])
        else:
            self.selected_existing.set("")

    # ---------- UI ----------
    def _build_ui(self):
        frm = ttk.Frame(self.root, padding=12)
        frm.pack(fill="both", expand=True)

        ttk.Label(frm, text="Mode").grid(row=0, column=0, sticky="w", pady=(0, 6))
        mode_combo = ttk.Combobox(
            frm,
            textvariable=self.mode,
            values=["controller", "nav2_2d", "nav2_3d"],
            state="readonly",
        )
        mode_combo.grid(row=0, column=1, sticky="ew", pady=(0, 6))

        ttk.Label(frm, text="Obstacles (boxes)").grid(row=1, column=0, sticky="w", pady=(0, 6))
        ttk.Spinbox(frm, from_=0, to=200, textvariable=self.num_obs).grid(row=1, column=1, sticky="ew", pady=(0, 6))

        ttk.Label(frm, text="Seed (optional)").grid(row=2, column=0, sticky="w", pady=(0, 6))
        ttk.Entry(frm, textvariable=self.seed).grid(row=2, column=1, sticky="ew", pady=(0, 6))

        # World source selection
        ttk.Label(frm, text="World source").grid(row=3, column=0, sticky="w", pady=(12, 6))
        src_row = ttk.Frame(frm)
        src_row.grid(row=3, column=1, sticky="ew", pady=(12, 6))
        ttk.Radiobutton(src_row, text="Generate new", value="generate", variable=self.world_source).pack(side="left", padx=(0, 12))
        ttk.Radiobutton(src_row, text="Use existing", value="existing", variable=self.world_source).pack(side="left")

        # Existing world dropdown
        ttk.Label(frm, text="Existing generated world").grid(row=4, column=0, sticky="w", pady=(0, 6))
        self.existing_combo = ttk.Combobox(frm, textvariable=self.selected_existing, state="readonly")
        self.existing_combo.grid(row=4, column=1, sticky="ew", pady=(0, 6))
        ttk.Button(frm, text="Refresh list", command=self._refresh_world_dropdown).grid(row=5, column=1, sticky="e", pady=(0, 6))

        # Path label
        ttk.Label(frm, text="Generated world path (install share)").grid(row=6, column=0, sticky="w", pady=(12, 6))
        self.world_path_lbl = ttk.Label(
            frm,
            text=f"original:  {self.original_dir}\n"
                f"generated: {self.generated_dir}",
            wraplength=520,
        )

        self.world_path_lbl.grid(row=6, column=1, sticky="w", pady=(12, 6))

        # Buttons
        btns = ttk.Frame(frm)
        btns.grid(row=7, column=0, columnspan=2, sticky="ew", pady=(18, 0))
        ttk.Button(btns, text="Generate / Launch", command=self.on_launch).pack(side="left", fill="x", expand=True, padx=(0, 6))
        ttk.Button(btns, text="Stop", command=self.on_stop).pack(side="left", fill="x", expand=True)

        frm.grid_columnconfigure(1, weight=1)

        # initial populate
        self._refresh_world_dropdown()

    # ---------- launch logic ----------
    def on_launch(self):
        # 1) Choose world path
        if self.world_source.get() == "existing":
            name = self.selected_existing.get().strip()
            if not name:
                messagebox.showerror(
                    "No world selected",
                    "No existing generated world found/selected.\nGenerate one first or click 'Refresh list'.",
                )
                return
            out_path = self.worlds_root / name
            if not out_path.is_file():
                messagebox.showerror("World missing", f"Selected world does not exist:\n{out_path}")
                return
        else:
            # Generate world
            try:
                n = int(self.num_obs.get())
                seed_txt = self.seed.get().strip()
                seed_val = int(seed_txt) if seed_txt else None

                out_path = default_output_path(self.generated_dir)
                out_path = write_world(out_path, num_obstacles=n, seed=seed_val)
                self._refresh_world_dropdown()
                self.selected_existing.set(out_path.name)
            except Exception as e:
                messagebox.showerror("Generate failed", str(e))
                return

        # 2) Map mode -> boolean flags for launch file
        mode = self.mode.get()
        use_controller = "true" if mode == "controller" else "false"
        use_nav2_2d = "true" if mode == "nav2_2d" else "false"
        use_nav2_3d = "true" if mode == "nav2_3d" else "false"

        # 3) Stop previous run if any
        self.on_stop()

        # 4) Launch orchestrator (assumes this UI was run from a sourced environment via ros2 run)
        cmd = [
            "ros2", "launch", "drobot_bringup", "bringup.launch.py",
            f"world:={str(out_path)}",
            f"mode:={mode}",
            f"use_controller:={use_controller}",
            f"use_nav2_2d:={use_nav2_2d}",
            f"use_nav2_3d:={use_nav2_3d}",
        ]

        try:
            self.proc = subprocess.Popen(cmd)
        except FileNotFoundError:
            messagebox.showerror(
                "ros2 not found",
                "ros2 command not found.\nRun this UI using:\n"
                "source /opt/ros/jazzy/setup.bash\n"
                "source ~/Desktop/drobot/install/setup.bash\n"
                "ros2 run drobot_bringup world_ui"
            )
            return

        messagebox.showinfo("Launched", f"World:\n{out_path}\n\nMode: {mode}")

    def on_stop(self):
        if self.proc and (self.proc.poll() is None):
            try:
                self.proc.terminate()
            except Exception:
                pass
        self.proc = None


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
