#! /usr/bin/env python3

import site
from typing import List
from pip._internal.req.constructors import install_req_from_line
from pip._internal.utils.virtualenv import running_under_virtualenv


def main(req_strs: List[str]) -> int:
    return_code = 0
    pkg_installed = []

    try:
        for req_str in req_strs:
            req = install_req_from_line(req_str)

            req.check_if_exists(not running_under_virtualenv())

            if req.satisfied_by:
                pkg_installed.append(str(req.satisfied_by).replace(" ", "^"))
            else:
                pkg_installed.append(str(None))
                return_code = 1

    except Exception as e:
        print(f"check-pip-pkg-installed-version.py:\n{e}")
        return 2

    print(" ".join(pkg_installed))
    return return_code


if __name__ == "__main__":
    import argparse
    import sys

    parser = argparse.ArgumentParser(
        description="Check if a set of pip package is installed, meeting a requirement string."
    )
    parser.add_argument("req_strs", nargs="+")

    args = parser.parse_args()

    sys.exit(main(**vars(args)))
