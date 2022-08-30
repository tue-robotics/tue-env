#! /usr/bin/env python3
import os
from .environment import _locate

CONFIG_COMMANDS = [
    {
        "cmd": "git-use-ssh",
        "help": "Set all git remote URLs to use SSH",
        "args": 0,
        "env": "TUE_GIT_USE_SSH",
        "value": "true",
    },
    {
        "cmd": "git-use-https",
        "help": "Set all git remote URLs to use HTTPS",
        "args": 0,
        "env": "TUE_GIT_USE_SSH",
        "value": "false",
    },
    {
        "cmd": "github-use-ssh",
        "help": "Set all GitHub remote URLs to use SSH",
        "args": 0,
        "env": "TUE_GITHUB_USE_SSH",
        "value": "true",
    },
    {
        "cmd": "github-use-https",
        "help": "Set all GitHub remote URLs to use HTTPS",
        "args": 0,
        "env": "TUE_GITHUB_USE_SSH",
        "value": "false",
    },
    {
        "cmd": "gitlab-use-ssh",
        "help": "Set all GitLab remote URLs to use SSH",
        "args": 0,
        "env": "TUE_GITLAB_USE_SSH",
        "value": "true",
    },
    {
        "cmd": "gitlab-use-https",
        "help": "Set all GitLab remote URLs to use HTTPS",
        "args": 0,
        "env": "TUE_GITLAB_USE_SSH",
        "value": "false",
    },
    {
        "cmd": "install-test-depend",
        "help": "Set installation of test dependencies to be true",
        "args": 0,
        "env": "TUE_INSTALL_TEST_DEPEND",
        "value": "true",
    },
    {
        "cmd": "not-install-test-depend",
        "help": "Set installation of test dependencies to be false",
        "args": 0,
        "env": "TUE_INSTALL_TEST_DEPEND",
        "value": "false",
    },
    {
        "cmd": "install-doc-depend",
        "help": "Set installation of doc dependencies to be true",
        "args": 0,
        "env": "TUE_INSTALL_DOC_DEPEND",
        "value": "true",
    },
    {
        "cmd": "not-install-doc-depend",
        "help": "Set installation of doc dependencies to be false",
        "args": 0,
        "env": "TUE_INSTALL_DOC_DEPEND",
        "value": "false",
    },
    {
        "cmd": "set",
        "help": "Set a custom environment variable",
        "args": 2,
    },
]


def config(verbose, ws, cmd, **kwargs) -> None:

    tue_env_dir = _locate(ws)

    config_path = os.path.join(tue_env_dir, ".env", "setup", "user_setup.bash")

    if not cmd:
        print(f"Opening config file '{config_path}' in an editor...")
        os.system(f"edit {config_path}")
        return

    settings_current = {}

    with open(config_path, "r") as file:
        lines = [l for l in file.readlines() if l.strip()][1:]

        for line in lines:
            line = line.rstrip()
            kv = line.split(" ")[1]
            k, v = kv.split("=")
            settings_current[k] = v

    setting = [c for c in CONFIG_COMMANDS if cmd == c["cmd"]][0]

    if not setting["args"]:
        settings_current[setting["env"]] = setting["value"]
    else:
        settings_current[kwargs["env"]] = kwargs["value"]

    with open(config_path, "w") as file:
        header = "#! /usr/bin/env bash\n\n"

        content = ""
        for k, v in settings_current.items():
            content += f"export {k}={v}\n"

        file.writelines(header + content)
