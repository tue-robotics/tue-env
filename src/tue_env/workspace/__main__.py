#! /usr/bin/env python3
import os
import sys
import argparse
import colorlog
from argcomplete import autocomplete
from ..utils import VersionAction
from .environment import get_workspaces
from .config import CONFIG_COMMANDS


def parse_args(argv):
    help_formatter = lambda prog: argparse.HelpFormatter(prog, max_help_position=52)
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--verbose", action="store_true")
    parser.add_argument("--version", action=VersionAction, help="print version of the installed package and exit")

    subparsers = parser.add_subparsers(dest="command")
    subparsers.required = True

    subparser = subparsers.add_parser("init", help="initialize a new workspace", formatter_class=help_formatter)
    subparser.set_defaults(func="init")
    subparser.add_argument("name", metavar="NAME", help="name of workspace to create")
    subparser.add_argument("--dir", metavar="WS_DIR", default=os.getcwd(), help="The directory to create the workspace in. Defaults to the current working directory", required=False)
    subparser.add_argument("--targets-git-url", metavar="TARGETS_GIT_URL", help="git URL of the targets repository", required=False)

    subparser = subparsers.add_parser("list", help="List all available packages")
    subparser.set_defaults(func="list_workspaces")

    subparser = subparsers.add_parser("remove", help="remove workspaces")
    subparser.set_defaults(func="remove")
    subparser.add_argument("name", metavar="WS", choices=get_workspaces(), nargs="+", help="name(s) of the workspace(s)")
    subparser.add_argument("--purge", help="completely purge the selected workspace(s) WS", action="store_true")

    subparser = subparsers.add_parser("config", help="configure workspace variables")
    subparser.set_defaults(func="config")
    subparser.add_argument("ws", metavar="WS", choices=get_workspaces(), help="name of workspace")
    ssparser = subparser.add_subparsers(dest="cmd", metavar="CMD")
    for cmd in CONFIG_COMMANDS:
        p = ssparser.add_parser(cmd["cmd"], help=cmd["help"], description=cmd["help"])
        if cmd["args"]:
            p.add_argument("env", metavar="ENV", help="Name of environment variable")
            p.add_argument("value", metavar="VALUE", help="Value to set in the environment variable")

    subparser = subparsers.add_parser("set-default", help="sets the default workspace")
    subparser.set_defaults(func="set_default")
    subparser.add_argument("name", metavar="WS", choices=get_workspaces(), nargs="?", help="name of the workspace")

    subparser = subparsers.add_parser("get-default", help="prints the name of default workspace if set")
    subparser.set_defaults(func="get_default")

    subparser = subparsers.add_parser("setup-location", help="Prints the location of setup.bash file")
    subparser.set_defaults(func="get_setup_path")

    subparser = subparsers.add_parser("locate", help="prints the path to the current workspace")
    subparser.set_defaults(func="locate")

    subparser = subparsers.add_parser("name", help="prints the name of the current workspace")
    subparser.set_defaults(func="get_current")

    subparser = subparsers.add_parser("switch", help="switches current workspace to the specified workspace")
    subparser.set_defaults(func="switch_workspaces")
    subparser.add_argument("name", metavar="WS", choices=get_workspaces(), help="name of the workspace")

    subparser = subparsers.add_parser("targets-dir", help="prints the path to targets directory")
    subparser.set_defaults(func="targets")

    subparser = subparsers.add_parser("init-targets", help="(re-)initialize the targets list")
    subparser.set_defaults(func="init_targets")
    subparser.add_argument("url", metavar="TARGETS_URL", help="URL of the targets")
    subparser.add_argument("--workspace", metavar="WS", choices=get_workspaces(), nargs="?", help="name of the workspace")

    autocomplete(parser)

    # computation has to be avoided before the 'autocomplete(parser)' call
    args = parser.parse_args(argv)

    import logging.config

    logging.config.dictConfig(
        {
            "version": 1,
            "disable_existing_loggers": False,
            "formatters": {
                "colored": {
                    "()": "colorlog.ColoredFormatter",
                    "format": "%(log_color)s[%(levelname)s] %(name)s: %(message)s",
                }
            },
            "handlers": {
                "stream": {
                    "class": "logging.StreamHandler",
                    "formatter": "colored",
                },
            },
            "root": {
                "handlers": ["stream"],
                "level": "INFO",
            },
        }
    )

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    # remove func from the namespace
    func = args.func
    del args.func
    del args.command

    return func, args


def main() -> int:
    from . import get_setup_path, config, list_workspaces, init, set_default, get_default, locate, get_current, switch_workspaces, targets, init_targets, remove

    func, args = parse_args(sys.argv[1:])

    # execute the function given in 'func'
    func = {
        "init": init,
        "get_setup_path": get_setup_path,
        "config": config,
        "list_workspaces": list_workspaces,
        "set_default": set_default,
        "get_default": get_default,
        "locate": locate,
        "get_current": get_current,
        "switch_workspaces": switch_workspaces,
        "targets": targets,
        "init_targets": init_targets,
        "remove": remove,
    }[func]
    return func(**vars(args))


if __name__ == "__main__":
    sys.exit(main())
