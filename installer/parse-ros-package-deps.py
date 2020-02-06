#! /usr/bin/env python

from __future__ import print_function

import sys
import xml.etree.ElementTree as ET


def main():
    if len(sys.argv) != 2:
        print("Usage: parse-ros-package-deps PACKAGE.XML")
        return 1

    tree = ET.parse(sys.argv[1])
    doc = tree.getroot()

    dep_set = set()

    deps = doc.findall('build_depend')
    dep_set |= {dep.text for dep in deps}

    deps = doc.findall('run_depend')
    dep_set |= {dep.text for dep in deps}

    deps = doc.findall('depend')
    dep_set |= {dep.text for dep in deps}

    deps = doc.findall('build_export_depend')
    dep_set |= {dep.text for dep in deps}

    deps = doc.findall('exec_depend')
    dep_set |= {dep.text for dep in deps}

    deps = doc.findall('test_depend')
    dep_set |= {dep.text for dep in deps}

    deps = doc.findall('doc_depend')
    dep_set |= {dep.text for dep in deps}

    print('\n'.join(dep_set))


if __name__ == "__main__":
    sys.exit(main())
