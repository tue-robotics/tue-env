#! /usr/bin/env bash
#
# Functions that configure the an environment

# ----------------------------------------------------------------------------------------------------
#                                        ADDITIONAL TOOLS
# ----------------------------------------------------------------------------------------------------

function _set_export_option {

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

    echo "[tue-env](config) Environment '$env' set to use SSH"
}

function tue-env-use-https
{
    tue_env_dir=$1
    local option="TUE_USE_SSH"
    local value="false"
    _set_export_option "$option" "$value" $tue_env_dir/.env/setup/user_setup.bash

    if [ "$CURRENT_ENV" == "true" ]
    then
        source $tue_env_dir/.env/setup/user_setup.bash
    fi

    echo "[tue-env](config) Environment '$env' set to use HTTPS"
}

if [ -z "$1" ]
then
    echo "[tue-env](config) no environment set or provided"
    exit 1
else
    env=$1
    shift

    if [ "$env" == "$TUE_ENV" ]
    then
        CURRENT_ENV=true
    else
        CURRENT_ENV=false
    fi

    # TODO Check if the cat command does not give errors
    tue_env_dir="$(cat $TUE_DIR/user/envs/$env)"

    if [ -z "$1" ]
    then
        vim $tue_env_dir/.env/setup/user_setup.bash
    else
        while [ "$1" != "" ]
        do
            case $1 in
                use-ssh )
                    tue-env-use-ssh ;;

                use-https )
                    tue-env-use-https ;;

                * )
                    echo "[tue-env](config) Unknown config command"
                    exit 1 ;;
            esac
            shift
        done
    fi
fi
