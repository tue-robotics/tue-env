#! /usr/bin/env bash

# Functions that configure an environment

function _set_export_option
{

    # _set_export_option KEY VALUE FILE
    # Add the following line: 'export KEY=VALUE' to FILE
    # Or changes the VALUE to current value if line already in FILE.

    key=${1//\//\\/}
    value=${2//\//\\/}
    sed -i \
        -e '/^#\?\(\s*'"export ${key}"'\s*=\s*\).*/{s//\1'"${value}"'/;:a;n;ba;q}' \
        -e '$a'"export ${key}"'='"${value}" $3
}

function tue-env-use-ssh
{
    local option="TUE_USE_SSH"
    local value="true"
    _set_export_option "$option" "$value" $tue_env_dir/.env/setup/user_setup.bash

    if [ "$CURRENT_ENV" == "true" ]
    then
        source $tue_env_dir/.env/setup/user_setup.bash
    fi

    echo -e "[tue-env](config) Environment '$env' set to use SSH"
}

function tue-env-use-https
{
    local option="TUE_USE_SSH"
    local value="false"
    _set_export_option "$option" "$value" $tue_env_dir/.env/setup/user_setup.bash

    if [ "$CURRENT_ENV" == "true" ]
    then
        source $tue_env_dir/.env/setup/user_setup.bash
    fi

    echo -e "[tue-env](config) Environment '$env' set to use HTTPS"
}
