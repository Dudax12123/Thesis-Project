"""``python -m Archive ...`` from Tese/src. From the repository root use
``python Tese/src/run_archive.py ...`` instead, which sets sys.path itself."""

import sys

from .cli import main

sys.exit(main() or 0)
