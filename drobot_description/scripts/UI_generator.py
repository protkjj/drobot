import tkinter as tk
from tkinter import ttk, messagebox

root = tk.Tk()
root.title("World Generator UI")
root.geometry("900x500")

root.grid_rowconfigure(0, weight=1)
root.grid_columnconfigure(0, weight=2)
root.grid_columnconfigure(1, weight=0)
root.grid_columnconfigure(2, weight=2)
root.grid_columnconfigure(3, weight=0)
root.grid_columnconfigure(4, weight=2)

left = ttk.Frame(root, padding=10)
left.grid(row=0, column=0, sticky="nsew")

sep = ttk.Separator(root, orient="vertical")
sep.grid(row=0, column=1, sticky="ns")

middle = ttk.Frame(root, padding=10)
middle.grid(row=0, column=2, sticky="nsew")

sep2 = ttk.Separator(root, orient="vertical")
sep2.grid(row=0, column=3, sticky="ns")

right = ttk.Frame(root, padding=10)
right.grid(row=0, column=4, sticky="nsew")

# Headers
ttk.Label(left, text="Select World").pack(anchor="w")
ttk.Label(middle, text="Create World").pack(anchor="w")
ttk.Label(right, text="Selection").pack(anchor="w")

saved_worlds = [
    "City/Urban 1", "City/Urban 2", "Nature", "Hospital",
    "Road 1", "Road 2", "Extraterrestrial"
]
create_worlds = ["Stationary", "Moving", "Random"]

selected_world = tk.StringVar(value="(none)")

ttk.Label(right, text="Selected:").pack(anchor="w", pady=(10, 0))
ttk.Label(right, textvariable=selected_world).pack(anchor="w", pady=(0, 10))

# Create ONE action button (disabled by default)
action_btn = ttk.Button(right, text="Select something first", state="disabled")
action_btn.pack(pady=5, fill="x")


def action_world():
    world = selected_world.get()
    if world in saved_worlds:
        print(f"Loading world: {world}")
        messagebox.showinfo("Load World", f"World '{world}' loaded successfully.")
    elif world in create_worlds:
        print(f"Creating world: {world}")
        messagebox.showinfo("Create World", f"World '{world}' created successfully.")


def on_select_world(world_name):
    selected_world.set(world_name)
    print(f"Selected world: {world_name}")

    # Update the ONE button instead of creating new ones
    if world_name in saved_worlds:
        action_btn.config(text="Load World", state="normal", command=action_world)
    elif world_name in create_worlds:
        action_btn.config(text="Create World", state="normal", command=action_world)
    else:
        action_btn.config(text="Unknown selection", state="disabled")


def button_creation():
    for w in saved_worlds:
        ttk.Button(left, text=w, command=lambda name=w: on_select_world(name)).pack(pady=5, fill="x")
    for w in create_worlds:
        ttk.Button(middle, text=w, command=lambda name=w: on_select_world(name)).pack(pady=5, fill="x")

button_creation()
root.mainloop()
