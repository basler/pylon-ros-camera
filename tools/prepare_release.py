# tools/prepare_release.py
import sys
import glob
import re
from datetime import date

if len(sys.argv) != 2:
    print("Usage: python3 tools/prepare_release.py <new_version>  (e.g. 4.1.0)")
    sys.exit(1)

new_version = sys.argv[1]
today = date.today().isoformat()
EXCLUDE_DIRS = ("/build/", "/install/", "/log/")

# --- 1. Update all package.xml files ---
updated_pkgs = []
for path in glob.glob("**/package.xml", recursive=True):
    if any(excl in path for excl in EXCLUDE_DIRS):
        continue
    with open(path, "r") as f:
        content = f.read()
    new_content = re.sub(
        r"<version>.*?</version>",
        f"<version>{new_version}</version>",
        content,
        count=1,
    )
    if new_content != content:
        with open(path, "w") as f:
            f.write(new_content)
        updated_pkgs.append(path)

# --- 2. Insert a new CHANGELOG.rst entry at the top ---
CHANGELOG_PATH = "CHANGELOG.rst"
with open(CHANGELOG_PATH, "r") as f:
    changelog = f.read()

new_header = f"{new_version} ({today})"
underline_length = max(len(new_header), 19)   # 19 matches this project's existing convention
new_entry = f"{new_header}\n{'-' * underline_length}\n* TODO: fill in release notes\n\n"

# Insert after the title block (assumes title + blank line at top)
lines = changelog.split("\n", 3)
changelog_updated = "\n".join(lines[:3]) + "\n\n" + new_entry + "\n".join(lines[3:]).lstrip("\n")

with open(CHANGELOG_PATH, "w") as f:
    f.write(changelog_updated)

print(f"Updated {len(updated_pkgs)} package.xml files to version {new_version}:")
for p in updated_pkgs:
    print(f"  - {p}")
print(f"Inserted new CHANGELOG.rst entry for {new_version} ({today}) — fill in the TODO before committing.")