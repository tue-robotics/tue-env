#! /usr/bin/env bash

# Functions that configure an environment
# All functions should be called tue-env-XXX. The functions should be in
# this file for it to work and to be available in the auto complete

function _set_export_option
{

    # _set_export_option KEY VALUE FILE
    # Add the following line: 'export KEY=VALUE' to FILE
    # Or changes the VALUE to current value if line already in FILE.

    local key value
    key=${1//\//\\/}
    value=${2//\//\\/}
    sed -i \
        -e '/^#\?\(\s*'"export ${key}"'\s*=\s*\).*/{s//\1'"${value}"'/;:a;n;ba;q}' \
        -e '$a'"export ${key}"'='"${value}" "$3"
}

function tue-env-git-use-ssh
{
    local option value
    option="TUE_GIT_USE_SSH"
    value="true"
    _set_export_option "$option" "$value" "$tue_env_dir"/.env/setup/user_setup.bash

    echo -e "[tue-env](config) Environment '$env' set to use SSH for git as default"
}

function tue-env-git-use-https
{
    local option value
    option="TUE_GIT_USE_SSH"
    value="false"
    _set_export_option "$option" "$value" "$tue_env_dir"/.env/setup/user_setup.bash

    echo -e "[tue-env](config) Environment '$env' set to use HTTPS for git as default"
}

function tue-env-github-use-ssh
{
    local option value
    option="TUE_GITHUB_USE_SSH"
    value="true"
    _set_export_option "$option" "$value" "$tue_env_dir"/.env/setup/user_setup.bash

    echo -e "[tue-env](config) Environment '$env' set to use SSH for GitHub"
}

function tue-env-github-use-https
{
    local option value
    option="TUE_GITHUB_USE_SSH"
    value="false"
    _set_export_option "$option" "$value" "$tue_env_dir"/.env/setup/user_setup.bash

    echo -e "[tue-env](config) Environment '$env' set to use HTTPS for GitHub"
}

function tue-env-gitlab-use-ssh
{
    local option value
    option="TUE_GITLAB_USE_SSH"
    value="true"
    _set_export_option "$option" "$value" "$tue_env_dir"/.env/setup/user_setup.bash

    echo -e "[tue-env](config) Environment '$env' set to use SSH for GitLab"
}

function tue-env-gitlab-use-https
{
    local option value
    option="TUE_GITLAB_USE_SSH"
    value="false"
    _set_export_option "$option" "$value" "$tue_env_dir"/.env/setup/user_setup.bash

    echo -e "[tue-env](config) Environment '$env' set to use HTTPS for GitLab"
}

function tue-env-install-test-depend
{
    local option value
    option="TUE_INSTALL_TEST_DEPEND"
    value="true"
    _set_export_option "$option" "$value" "$tue_env_dir"/.env/setup/user_setup.bash

    echo -e "[tue-env](config) Environment '$env' set to install test dependencies"
}

function tue-env-not-install-test-depend
{
    local option value
    option="TUE_INSTALL_TEST_DEPEND"
    value="false"
    _set_export_option "$option" "$value" "$tue_env_dir"/.env/setup/user_setup.bash

    echo -e "[tue-env](config) Environment '$env' set to not install test dependencies"
}

function tue-env-install-doc-depend
{
    local option value
    option="TUE_INSTALL_DOC_DEPEND"
    value="true"
    _set_export_option "$option" "$value" "$tue_env_dir"/.env/setup/user_setup.bash

    echo -e "[tue-env](config) Environment '$env' set to install doc dependencies"
}

function tue-env-not-install-doc-depend
{
    local option value
    option="TUE_INSTALL_DOC_DEPEND"
    value="false"
    _set_export_option "$option" "$value" "$tue_env_dir"/.env/setup/user_setup.bash

    echo -e "[tue-env](config) Environment '$env' set to not install doc dependencies"
}

function tue-env-set
{
    local option value
    option="$1"
    value="$2"
    _set_export_option "$option" "$value" "$tue_env_dir"/.env/setup/user_setup.bash

    echo -e "[tue-env](config) Environment '$env' has '$option' set to '$value'"
}

function _main
{
    if [ -z "$1" ]
    then
        echo -e "[tue-env](config) no environment set or provided"
        exit 1
    else
        local env
        env=$1
        shift

        local tue_env_dir
        tue_env_dir="$(cat "$TUE_DIR"/user/envs/"$env")"

        if [ -z "$1" ]
        then
            edit "$(file --mime-type "${tue_env_dir}/.env/setup/user_setup.bash" | awk '{print $2}')":"${tue_env_dir}/.env/setup/user_setup.bash"
        else
            local functions
            functions=$(compgen -A function | grep "tue-env-")
            functions=${functions//tue-env-/}
            # shellcheck disable=SC2086
            functions=$(echo $functions | tr ' ' '|')

            local cmd
            cmd=$1
            shift

            eval "
                case $cmd in
                    $functions )
                            tue-env-$cmd $*;;
                    * )
                        echo -e '[tue-env](config) Unknown config command: $cmd'
                        exit 1 ;;
                esac"
        fi
    fi
}

_main "$@"
