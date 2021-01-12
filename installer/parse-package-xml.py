#! /usr/bin/env python3

from __future__ import print_function

import os
import sys
import xml.etree.ElementTree as ET

from catkin_pkg.condition import evaluate_condition


def main():
    if len(sys.argv) != 2:
        print("Usage: parse-package-xml PACKAGE.XML")
        return 1

    tree = ET.parse(sys.argv[1])
    doc = tree.getroot()

    dep_set = set()

    dep_types = []

    if os.getenv("TUE_INSTALL_SKIP_ROS_DEPS", "false") == "false":
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

    if os.getenv("TUE_INSTALL_TEST_DEPEND", "false") == "true":
        dep_types.append("test_depend")

    if os.getenv("TUE_INSTALL_DOC_DEPEND", "false") == "true":
        dep_types.append("doc_depend")

    for dep_type in dep_types:
        deps = doc.findall(dep_type)
        dep_set |= {dep.text for dep in deps if evaluate_condition(dep.attrib.get("condition", None), os.environ)}

    print("\n".join(dep_set))


if __name__ == "__main__":
    sys.exit(main())
