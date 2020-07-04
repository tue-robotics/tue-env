#! /usr/bin/env python3

# Functions that configure an environment
# All functions should be called tue-env-XXX. The functions should be in
# this file for it to work and to be available in the auto complete

import os
import shlex
import subprocess as sub
import sys


# function _set_export_option
# {
#
#     # _set_export_option KEY VALUE FILE
#     # Add the following line: 'export KEY=VALUE' to FILE
#     # Or changes the VALUE to current value if line already in FILE.
#
#     local key=${1//\//\\/}
#     local value=${2//\//\\/}
#     sed -i \
#         -e '/^#\?\(\s*'"export ${key}"'\s*=\s*\).*/{s//\1'"${value}"'/;:a;n;ba;q}' \
#         -e '$a'"export ${key}"'='"${value}" "$3"
# }
#
# function tue-env-git-use-ssh
# {
#     local option="TUE_GIT_USE_SSH"
#     local value="true"
#     _set_export_option "$option" "$value" "$tue_env_dir"/.env/setup/user_setup.bash
#
#     echo -e "[tue-env](config) Environment '$env' set to use SSH for git as default"
# }
#
# function tue-env-git-use-https
# {
#     local option="TUE_GIT_USE_SSH"
#     local value="false"
#     _set_export_option "$option" "$value" "$tue_env_dir"/.env/setup/user_setup.bash
#
#     echo -e "[tue-env](config) Environment '$env' set to use HTTPS for git as default"
# }
#
# function tue-env-github-use-ssh
# {
#     local option="TUE_GITHUB_USE_SSH"
#     local value="true"
#     _set_export_option "$option" "$value" "$tue_env_dir"/.env/setup/user_setup.bash
#
#     echo -e "[tue-env](config) Environment '$env' set to use SSH for GitHub"
# }
#
# function tue-env-github-use-https
# {
#     local option="TUE_GITHUB_USE_SSH"
#     local value="false"
#     _set_export_option "$option" "$value" "$tue_env_dir"/.env/setup/user_setup.bash
#
#     echo -e "[tue-env](config) Environment '$env' set to use HTTPS for GitHub"
# }
#
# function tue-env-gitlab-use-ssh
# {
#     local option="TUE_GITLAB_USE_SSH"
#     local value="true"
#     _set_export_option "$option" "$value" "$tue_env_dir"/.env/setup/user_setup.bash
#
#     echo -e "[tue-env](config) Environment '$env' set to use SSH for GitLab"
# }
#
# function tue-env-gitlab-use-https
# {
#     local option="TUE_GITLAB_USE_SSH"
#     local value="false"
#     _set_export_option "$option" "$value" "$tue_env_dir"/.env/setup/user_setup.bash
#
#     echo -e "[tue-env](config) Environment '$env' set to use HTTPS for GitLab"
# }
#
# function tue-env-install-test-depend
# {
#     local option="TUE_INSTALL_TEST_DEPEND"
#     local value="true"
#     _set_export_option "$option" "$value" "$tue_env_dir"/.env/setup/user_setup.bash
#
#     echo -e "[tue-env](config) Environment '$env' set to install test dependencies"
# }
#
# function tue-env-not-install-test-depend
# {
#     local option="TUE_INSTALL_TEST_DEPEND"
#     local value="false"
#     _set_export_option "$option" "$value" "$tue_env_dir"/.env/setup/user_setup.bash
#
#     echo -e "[tue-env](config) Environment '$env' set to not install test dependencies"
# }
#
# function tue-env-install-doc-depend
# {
#     local option="TUE_INSTALL_DOC_DEPEND"
#     local value="true"
#     _set_export_option "$option" "$value" "$tue_env_dir"/.env/setup/user_setup.bash
#
#     echo -e "[tue-env](config) Environment '$env' set to install doc dependencies"
# }
#
# function tue-env-not-install-doc-depend
# {
#     local option="TUE_INSTALL_DOC_DEPEND"
#     local value="false"
#     _set_export_option "$option" "$value" "$tue_env_dir"/.env/setup/user_setup.bash
#
#     echo -e "[tue-env](config) Environment '$env' set to not install doc dependencies"
# }


def main(*args, **kwargs):
    env = kwargs["env"]
    if len(args) < 2:
        print("[tue-env](config) no environment set or provided")
        return 1
    else:
        tue_env = args[1]
        tue_dir = env["TUE_DIR"]
        tue_env_file = os.path.join(tue_dir, "user", "envs", tue_env)
        if not os.path.isfile(tue_env_file):
            print(f"[tue-env](config) environment: '{tue_env}' does not exist")
            return 1
        with open(tue_env_file) as f:
            tue_env_dir = f.readline().strip()

        if len(args) < 3:
            user_setup_path = os.path.join(tue_env_dir, ".env", "setup", "user_setup.bash")
            edit_args = f"vim {user_setup_path}"
            output = sub.run(args=shlex.split(edit_args), shell=False)
            if output.returncode:
                print(f"command: '{edit_args}' failed\nstdout: {output.stdout}\nstderr: {output.stderr}")
                return 1
        else:
            print("bla")

    #         functions=$(compgen -A function | grep "tue-env-")
    #         functions=${functions//tue-env-/}
    #         # shellcheck disable=SC2086
    #         functions=$(echo $functions | tr ' ' '|')
    #
    #         cmd=$1
    #         shift
    #
    #         eval "
    #             case $cmd in
    #                 $functions )
    #                         tue-env-$cmd $*;;
    #                 * )
    #                     echo -e '[tue-env](config) Unknown config command: $cmd'
    #                     exit 1 ;;
    #             esac"
    #     fi
    # fi


if __name__ == "__main__":
    kwargs = {"env": os.environ}
    sys.exit(main(*sys.argv, **kwargs))
