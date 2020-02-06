#! /usr/bin/env python

from __future__ import print_function

import sys
from pip._internal.req.constructors import install_req_from_req_string


def main():
    if len(sys.argv) != 2:
        print("Usage: pip-correct-installed.py requirement")
        return 1

    req = install_req_from_req_string(sys.argv[1])

    req.check_if_exists(True)

    if req.satisfied_by:
        print(str(req.satisfied_by))
        return 0
    else:
        return 1


if __name__ == "__main__":
    sys.exit(main())
