#!/usr/bin/env python3
"""
Find and list all available robot models.

This script searches for robot models and displays their properties.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.utils import list_available_models


def main():
    print("Searching for robot models...")
    list_available_models()


if __name__ == "__main__":
    main()
