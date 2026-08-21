# .github/scripts/ros2_dep_snapshot.py
import glob, json, os, sys, time, xml.etree.ElementTree as ET

manifests = {}
for path in glob.glob("**/package.xml", recursive=True):
    tree = ET.parse(path)
    root = tree.getroot()
    pkg_name = root.findtext("name")
    resolved = {}
    for tag in ("depend", "build_depend", "exec_depend", "test_depend"):
        for dep in root.findall(tag):
            dep_name = dep.text.strip()
            purl = f"pkg:generic/ros-rolling/{dep_name}"  # custom purl, since rosdep keys have no official purl type
            resolved[purl] = {
                "package_url": purl,
                "relationship": "direct",
                "scope": "runtime" if tag != "test_depend" else "development",
                "dependencies": []
            }
    manifests[path] = {
        "name": pkg_name,
        "file": {"source_location": path},
        "resolved": resolved
    }

snapshot = {
    "version": 0,
    "sha": os.environ.get("GITHUB_SHA", ""),
    "ref": os.environ.get("GITHUB_REF", ""),
    "job": {
        "correlator": "ros2-dependency-snapshot",
        "id": os.environ.get("GITHUB_RUN_ID", "local")
    },
    "detector": {
        "name": "ros2-package-xml-scanner",
        "version": "1.0.0",
        "url": "https://github.com/your-org/your-scanner"
    },
    "scanned": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
    "manifests": manifests
}

print(json.dumps(snapshot))
