"""Entry point: `python3 tools/odgen [--check]` (runs with tools/odgen on sys.path)."""

import sys

from odgen.cli import main

sys.exit(main())
