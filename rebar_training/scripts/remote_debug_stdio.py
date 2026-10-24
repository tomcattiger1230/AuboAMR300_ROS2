#!/usr/bin/env python3
"""Fixed SSH command target; can run with system Python without Isaac/CUDA imports."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from rebar_training.remote_debug import main

if __name__ == '__main__':
    raise SystemExit(main())
