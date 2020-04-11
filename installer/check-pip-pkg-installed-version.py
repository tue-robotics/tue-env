#! /usr/bin/env python

from __future__ import print_function

import sys
from pip._internal.req.constructors import install_req_from_line


def main():
    if len(sys.argv) < 2:
        print("Usage: check-pip-pkg-installed-version.py requirement [requirements]")
        return 2

    return_code = 0
    pkg_installed = []

    try:
        for arg in sys.argv[1:]:
            req = install_req_from_line(arg)

            req.check_if_exists(True)

            if req.satisfied_by:
                pkg_installed.append(str(req.satisfied_by).replace(" ", "^"))
            else:
                pkg_installed.append(str(None))
                return_code = 1

    except Exception as e:
        print("check-pip-pkg-installed-version.py:\n{}".format(e))
        return 2

    print(" ".join(pkg_installed))
    return return_code


if __name__ == "__main__":
    sys.exit(main())
