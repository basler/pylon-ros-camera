import glob, json, os, time, uuid
import xml.etree.ElementTree as ET


def parse_package_xml(path):
    tree = ET.parse(path)
    root = tree.getroot()
    name = root.findtext("name")
    version = root.findtext("version") or "0.0.0"
    deps = []
    for tag in ("depend", "build_depend", "exec_depend", "test_depend"):
        for dep in root.findall(tag):
            deps.append(dep.text.strip())
    return name, version, deps


EXCLUDE_DIRS = ("/build/", "/install/", "/log/")

package_paths = [
    p for p in glob.glob("**/package.xml", recursive=True)
    if not any(excl in p for excl in EXCLUDE_DIRS)
]

pkg_info = {}
for path in package_paths:
    name, version, deps = parse_package_xml(path)
    pkg_info[path] = (name, version, deps)

components = {}
dependencies = {}

for path, (name, version, deps) in pkg_info.items():
    bom_ref = f"pkg:generic/{name}@{version}"
    components[bom_ref] = {
        "type": "library",
        "bom-ref": bom_ref,
        "name": name,
        "version": version,
        "purl": bom_ref,
    }

    dep_refs = []
    for dep_name in deps:
        dep_version = "unknown"
        for _, (n, v, _) in pkg_info.items():
            if n == dep_name:
                dep_version = v
                break

        dep_ref = f"pkg:generic/{dep_name}@{dep_version}"
        if dep_ref not in components:
            components[dep_ref] = {
                "type": "library",
                "bom-ref": dep_ref,
                "name": dep_name,
                "version": dep_version,
                "purl": dep_ref,
            }
        dep_refs.append(dep_ref)

    dependencies[bom_ref] = dep_refs

sbom = {
    "bomFormat": "CycloneDX",
    "specVersion": "1.6",
    "serialNumber": f"urn:uuid:{uuid.uuid4()}",
    "version": 1,
    "metadata": {
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "component": {
            "type": "application",
            "name": os.environ.get("GITHUB_REPOSITORY", "ros2-workspace"),
        },
    },
    "components": list(components.values()),
    "dependencies": [
        {"ref": ref, "dependsOn": deps} for ref, deps in dependencies.items()
    ],
}

print(json.dumps(sbom, indent=2))