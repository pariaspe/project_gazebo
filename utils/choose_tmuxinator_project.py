#!/bin/python3

"""
choose_tmuxinator_project.py
"""

import argparse
import sys
from pathlib import Path


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("project_folder", nargs='?', default="tmuxinator",
                        help="Folder containing project config files")
    args = parser.parse_args()

    project_folder = Path(args.project_folder)

    files = sorted(project_folder.glob("aerostack2-*.yml"))
    print("Choose Aerostack2 tmuxinator project file to open:")
    i = 1
    for file in files:
        print(f"\t[{i}] {file.name}")
        i += 1

    try:
        s = input('')
        opt = int(s)
        chosen = files[opt - 1]
    except (EOFError, ValueError, IndexError):
        print("Invalid")
        sys.exit(1)

    # Return for use in bash scripts
    print(chosen)
