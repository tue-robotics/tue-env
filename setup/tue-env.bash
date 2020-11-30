#! /usr/bin/env bash

# ----------------------------------------------------------------------------------------------------
#                                              TUE-ENV
# ----------------------------------------------------------------------------------------------------

function tue-env
{
    if [ -z "$1" ]
    then
        # shellcheck disable=SC1078,SC1079
        echo """tue-env is a tool for switching between different installation environments.

    Usage: tue-env COMMAND [ARG1 ARG2 ...]

    Possible commands:

        init           - Initializes new environment
        remove         - Removes an existing enviroment
        switch         - Switch to a different environment
        config         - Configures current environment
        set-default    - Set default environment
        init-targets   - (Re-)Initialize the target list
        targets        - Changes directory to targets directory
        list           - List all possible environments
        list-current   - Shows current environment
        cd             - Changes directory to environment directory
"""
        return 1
    fi

    cmd=$1
    shift

    # Make sure the correct directories are there
    mkdir -p "$TUE_DIR"/user/envs

    if [[ $cmd == "init" ]]
    then
        if [ -z "$1" ]
        then
            echo "Usage: tue-env init NAME [ DIRECTORY ] [ TARGETS GIT URL ]"
            return 1
        fi

        local dir=$PWD   # default directory is current directory
        [ -z "$2" ] || dir=$2
        dir="$( realpath "$dir" )"

        if [ -f "$TUE_DIR"/user/envs/"$1" ]
        then
            echo "[tue-env] Environment '$1' already exists"
            return 1
        fi

        if [ -d "$dir"/.env ]
        then
            echo "[tue-env] Directory '$dir' is already an environment directory."
            return 1
        fi

        echo "$dir" > "$TUE_DIR"/user/envs/"$1"
        # Create .env and .env/setup directories
        mkdir -p "$dir"/.env/setup
        echo -e "#! /usr/bin/env bash\n" > "$dir"/.env/setup/user_setup.bash
        echo "[tue-env] Created new environment $1"

        if [ -n "$3" ]
        then
            tue-env init-targets "$1" "$3"
        fi

    elif [[ $cmd == "remove" ]]
    then
        if [ -z "$1" ]
        then
            # shellcheck disable=SC1078,SC1079
            echo """Usage: tue-env remove [options] ENVIRONMENT
options:
    --purge
        Using this would completely remove the selected ENVIRONMENT if it exists"""
            return 1
        else
            # Set purge to be false by default
            PURGE=false
            env=""
            while test $# -gt 0
            do
                case "$1" in
                    --purge)
                        PURGE=true
                        ;;
                    --*)
                        echo "[tue-env] Unknown option $1"
                        ;;
                    *)
                        # Read only the first passed environment name and ignore
                        # the rest
                        if [ -z $env ]
                        then
                            env=$1
                        fi
                        ;;
                esac
                shift
            done
        fi

        if [ ! -f "$TUE_DIR"/user/envs/"$env" ]
        then
            echo "[tue-env] No such environment: '$env'."
            return 1
        fi

        dir=$(cat "$TUE_DIR"/user/envs/"$env")
        rm "$TUE_DIR"/user/envs/"$env"

        if [ $PURGE == "false" ]
        then
            dir_moved=$dir.$(date +%F_%R)
            mv "$dir" "$dir_moved"
            # shellcheck disable=SC1078,SC1079
            echo """[tue-env] Removed environment '$env'
Moved environment directory of '$env' to '$dir_moved'"""
        else
            rm -rf "$dir"
            # shellcheck disable=SC1078,SC1079
            echo """[tue-env] Removed environment '$env'
Purged environment directory of '$env'"""
        fi

    elif [[ $cmd == "switch" ]]
    then
        if [ -z "$1" ]
        then
            echo "Usage: tue-env switch ENVIRONMENT"
            return 1
        fi

        if [ ! -f "$TUE_DIR"/user/envs/"$1" ]
        then
            echo "[tue-env] No such environment: '$1'."
            return 1
        fi

        export TUE_ENV=$1
        TUE_ENV_DIR=$(cat "$TUE_DIR"/user/envs/"$1")
        export TUE_ENV_DIR

        # shellcheck disable=SC1090
        source "$TUE_DIR"/setup.bash

    elif [[ $cmd == "set-default" ]]
    then
        if [ -z "$1" ]
        then
            echo "Usage: tue-env set-default ENVIRONMENT"
            return 1
        fi

        mkdir -p "$TUE_DIR"/user/config
        echo "$1" > "$TUE_DIR"/user/config/default_env
        echo "[tue-env] Default environment set to $1"

    elif [[ $cmd == "init-targets" ]]
    then
        if [ -z "$1" ] || { [ -z "${TUE_ENV-}" ] && [ -z "$2" ]; }
        then
            echo "Usage: tue-env init-targets [ENVIRONMENT] TARGETS_GIT_URL"
            return 1
        fi

        local env=$1
        local url=$2
        if [ -z "$url" ]
        then
            env=$TUE_ENV
            if [ -z "$env" ]
            then
                # This shouldn't be possible logical, should have exited after printing usage
                echo "[tue-env](init-targets) no enviroment set or provided"
                return 1
            fi
            url=$1
        fi

        local tue_env_dir
        tue_env_dir=$(cat "$TUE_DIR"/user/envs/"$env")
        local tue_env_targets_dir=$tue_env_dir/.env/targets

        if [ -d "$tue_env_targets_dir" ]
        then
            local targets_dir_moved
            targets_dir_moved=$tue_env_targets_dir.$(date +%F_%R)
            mv -f "$tue_env_targets_dir" "$targets_dir_moved"
            echo "[tue-env] Moved old targets of environment '$env' to $targets_dir_moved"
        fi

        git clone --recursive "$url" "$tue_env_targets_dir"
        echo "[tue-env] cloned targets of environment '$env' from $url"

    elif [[ $cmd == "targets" ]]
    then
        local env=$1
        [ -n "$env" ] || env=$TUE_ENV

        if [ -n "$env" ]
        then
            local tue_env_dir
            tue_env_dir=$(cat "$TUE_DIR"/user/envs/"$env")
            cd "$tue_env_dir"/.env/targets || { echo -e "Targets directory '$tue_env_dir/.env/targets' (environment '$TUE_ENV') does not exist"; return 1; }
        fi

    elif [[ $cmd == "config" ]]
    then
        local env=$1
        shift
        [ -n "$env" ] || env=$TUE_ENV

        "$TUE_DIR"/setup/tue-env-config.bash "$env" "$@"

        if [ "$env" == "$TUE_ENV" ]
        then
            local tue_env_dir
            tue_env_dir=$(cat "$TUE_DIR"/user/envs/"$env")
            # shellcheck disable=SC1090
            source "$tue_env_dir"/.env/setup/user_setup.bash
        fi

    elif [[ $cmd == "cd" ]]
    then
        local env=$1
        [ -n "$env" ] || env=$TUE_ENV

        if [ -n "$env" ]
        then
            local tue_env_dir
            tue_env_dir=$(cat "$TUE_DIR"/user/envs/"$env")
            cd "$tue_env_dir" || { echo -e "Environment directory '$tue_env_dir' (environment '$TUE_ENV') does not exist"; return 1; }
        else
            echo "[tue-env](cd) no enviroment set or provided"
            return 1
        fi

    elif [[ $cmd == "list" ]]
    then
        [ -d "$TUE_DIR"/user/envs ] || return 0

        for env in "$TUE_DIR"/user/envs/*
        do
            basename "$env"
        done

    elif [[ $cmd == "list-current" ]]
    then
        if [[ -n $TUE_ENV ]]
        then
            echo "$TUE_ENV"
        else
            echo "[tue-env] no enviroment set"
        fi

    else
        echo "[tue-env] Unknown command: '$cmd'"
        return 1
    fi
}

# ----------------------------------------------------------------------------------------------------

function _tue-env
{
    local cur=${COMP_WORDS[COMP_CWORD]}

    if [ "$COMP_CWORD" -eq 1 ]
    then
        mapfile -t COMPREPLY < <(compgen -W "init list switch list-current remove cd set-default config init-targets targets" -- "$cur")
    else
        cmd=${COMP_WORDS[1]}
        if [[ $cmd == "switch" ]] || [[ $cmd == "remove" ]] || [[ $cmd == "cd" ]] || [[ $cmd == "set-default" ]] || [[ $cmd == "init-targets" ]] || [[ $cmd == "targets" ]]
        then
            if [ "$COMP_CWORD" -eq 2 ]
            then
                local envs
                [ -d "$TUE_DIR"/user/envs ] && envs=$(ls "$TUE_DIR"/user/envs)

                mapfile -t COMPREPLY < <(compgen -W "$envs" -- "$cur")

            elif [[ $cmd == "remove" ]] && [ "$COMP_CWORD" -eq 3 ]
            then
                local IFS=$'\n'
                mapfile -t COMPREPLY < <(compgen -W "'--purge'" -- "$cur")
            fi
        elif [[ $cmd == "config" ]]
        then
            if [ "$COMP_CWORD" -eq 2 ]
            then
                local envs
                [ -d "$TUE_DIR/user/envs" ] && envs=$(ls "$TUE_DIR"/user/envs)
                mapfile -t COMPREPLY < <(compgen -W "$envs" -- "$cur")
            fi
            if [ "$COMP_CWORD" -eq 3 ]
            then
                local functions
                functions=$(grep 'function ' "$TUE_DIR"/setup/tue-env-config.bash | awk '{print $2}' | grep "tue-env-")
                functions=${functions//tue-env-/}
                mapfile -t COMPREPLY < <(compgen -W "$functions" -- "$cur")
            fi
        fi
    fi
}
complete -F _tue-env tue-env
