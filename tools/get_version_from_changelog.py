# tools/get_version_from_changelog.py
import re
import sys

CHANGELOG_PATH = "CHANGELOG.rst"

def get_latest_version(path=CHANGELOG_PATH):
    with open(path, "r") as f:
        content = f.read()

    # Matches version headers like "4.0.0 (2025-09-15)"
    match = re.search(r"^(\d+\.\d+\.\d+)\s*\(.*?\)", content, re.MULTILINE)
    if not match:
        raise ValueError(f"Could not find a version entry in {path}")
    return match.group(1)

if __name__ == "__main__":
    print(get_latest_version())