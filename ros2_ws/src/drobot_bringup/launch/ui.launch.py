## imports
import tkinter as tk
from tkinter import ttk, messagebox
from pathlib import Path
import shlex
import subprocess
import sys
import math
import xml.etree.ElementTree as ET


## UI Window

root = tk.Tk()
root.title("Start UI")
root.geometry("1200x500")

root.grid_rowconfigure(0, weight=1)
root.grid_columnconfigure(0, weight=2)
root.grid_columnconfigure(1, weight=0)
root.grid_columnconfigure(2, weight=2)
root.grid_columnconfigure(3, weight=0)
root.grid_columnconfigure(4, weight=2)
root.grid_columnconfigure(5, weight=0)
root.grid_columnconfigure(6, weight=2)

## Directories

world_dir = Path(__file__).parent.parent.parent.parent / "worlds"

## Frames

worlds1 = ttk.Frame(root, padding="10")
worlds1.grid(row=0, column=0, sticky="nsew")
ttk.Label(worlds1, text="Worlds").pack(pady=10)

def create_frame(title):
    return

## Separators

sep1 = ttk.Separator(root, orient="vertical")
sep1.grid(row=0, column=1, sticky="ns")

sep2 = ttk.Separator(root, orient="vertical")
sep2.grid(row=0, column=3, sticky="ns")

sep3 = ttk.Separator(root, orient="vertical")
sep3.grid(row=0, column=5, sticky="ns")

## Run
root.mainloop()