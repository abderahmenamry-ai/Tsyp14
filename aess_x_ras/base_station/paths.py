"""paths.py - make `nav` (living_map_nav) and `comms` importable regardless of the working directory."""
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
for p in (ROOT, os.path.join(ROOT, "living_map_nav"), os.path.join(ROOT, "comms")):
    if p not in sys.path:
        sys.path.insert(0, p)
