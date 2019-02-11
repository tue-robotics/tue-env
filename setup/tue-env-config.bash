#! /usr/bin/env bash

source $TUE_DIR/setup/tue-env-config-functions.bash

if [ -z "$1" ]
then
    echo -e "[tue-env](config) no environment set or provided"
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

    tue_env_dir="$(cat $TUE_DIR/user/envs/$env)"

    if [ -z "$1" ]
    then
        edit "${tue_env_dir}/.env/setup/user_setup.bash"
    else
        functions=$(compgen -A function | grep "tue-env-")
        functions=${functions//tue-env-/}
        functions=$(echo $functions | tr ' ' '|')
        while [ "$1" != "" ]
        do
            eval "
                case $1 in
                    $functions)
                        tue-env-$1 ;;
                    * )
                        echo -e '[tue-env](config) Unknown config command: $1'
                        exit 1 ;;
                esac"
            shift
        done
    fi
    unset CURRENT_ENV
fi
