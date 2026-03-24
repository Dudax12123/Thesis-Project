import sys
from pathlib import Path

# Ensure src/ is on path for tests executed from repository root.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
