import argparse
from datetime import date
import sys


class VersionAction(argparse.Action):
    """
    Custom VersionAction that prints the version of this package
    """

    def __init__(
        self,
        option_strings,
        dest=argparse.SUPPRESS,
        default=argparse.SUPPRESS,
        help="print version of the installed package and exit",
    ):
        super().__init__(option_strings=option_strings, dest=dest, default=default, nargs=0, help=help)

    def __call__(self, parser, namespace, values, option_string=None):
        from .. import __version__ as version_str
        parent_package = __package__.split(".")[0]
        year = date.today().year

        python_version = ".".join(map(str, sys.version_info[:2]))
        formatter = parser._get_formatter()

        formatter.add_text(f"{parent_package} {version_str} (python {python_version}) (c) 2020-{year} TechUnited Eindhoven under BSD-3 License")

        parser.exit(message=formatter.format_help())
