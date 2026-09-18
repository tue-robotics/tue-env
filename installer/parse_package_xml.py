#! /usr/bin/env python3

from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Set
import os
import sys
import traceback
import xml.etree.ElementTree as ET

from catkin_pkg.condition import evaluate_condition


def main() -> int:
    if len(sys.argv) != 2:
        print("Usage: parse_package_xml PACKAGE.XML")
        return 1

    try:
        path = Path(sys.argv[1])
        result = package_xml_parser(path)
    except Exception as e:
        print(f"ERROR: Could not parse package.xml: {repr(e)}\n{traceback.format_exc()}")
        return 1

    print("\n".join(result["deps"]))
    return 0


def package_xml_parser(path: Path) -> Mapping:
    tree = ET.parse(path)
    doc = tree.getroot()

    dep_set: Set[str] = set()

    dep_types = []
    fields = ["name", "version", "description", "maintainer", "export"]
    # Values are sets of strings, except 'emails', which maps a maintainer to their address
    parsed: Dict[str, Any] = {}

    if os.getenv("TUE_ENV_INSTALL_SKIP_ROS_DEPS", "false") == "false":
        dep_types.extend(
            [
                "build_depend",
                "buildtool_depend",
                "build_export_depend",
                "buildtool_export_depend",
                "exec_depend",
                "depend",
                "run_depend",
            ]
        )

    if os.getenv("TUE_ENV_INSTALL_TEST_DEPEND", "false") == "true":
        dep_types.append("test_depend")

    if os.getenv("TUE_ENV_INSTALL_DOC_DEPEND", "false") == "true":
        dep_types.append("doc_depend")

    for types in fields + dep_types:
        parsed[types] = set()

    for dep_type in dep_types:
        deps = doc.findall(dep_type)
        parsed[dep_type] |= {
            dep.text for dep in deps if dep.text and evaluate_condition(dep.attrib.get("condition", None), os.environ)
        }
        dep_set |= parsed[dep_type]

    for field in fields:
        values = doc.findall(field)
        if field == "export":
            for exports in values:
                parsed["build_type"] = {bt.text for bt in exports if bt.tag == "build_type" and bt.text}
        else:
            parsed[field] |= {value.text for value in values if value.text}
            if field == "maintainer":
                emails: Dict[str, Optional[str]] = {}
                for value in values:
                    if value.text:
                        emails[value.text] = value.attrib.get("email")
                parsed["emails"] = emails

    return {"parser": parsed, "deps": dep_set}


if __name__ == "__main__":
    sys.exit(main())
