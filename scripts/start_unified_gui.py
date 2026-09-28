#!/usr/bin/env python3
"""Start the unified console from this checkout, including offline UI preview."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'aubo_control_gui'))
from aubo_control_gui.unified.window import main
if __name__ == '__main__':
    raise SystemExit(main())
