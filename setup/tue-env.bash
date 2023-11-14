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
        remove         - Removes an existing environment
        switch         - Switch to a different environment
        config         - Configures current environment
        set-default    - Set default environment
        unset-default  - Unset default environment
        init-targets   - (Re-)Initialize the target list
        targets        - Changes directory to targets directory
        init-venv      - Initializes a virtualenv
        list           - List all possible environments
        current        - Shows current environment
        cd             - Changes directory to environment directory
"""
        return 1
    fi

    local cmd
    cmd=$1
    shift

    # Make sure the correct directories are there
    mkdir -p "$TUE_DIR"/user/envs

    local create_venv dir env_name targets_url show_help
    create_venv="false"
    show_help="false"

    if [[ $cmd == "init" ]]
    then
        if [[ -n "$1" ]]
        then
            env_name=$1
            shift
            for i in "$@"
            do
                case $i in
                    --targets-url=* )
                        targets_url="${i#*=}" ;;
                    --create-virtualenv=* )
                        create_venv="${i#*=}" ;;
                    --help )
                        show_help="true" ;;
                    * )
                        if [[ -z "${dir}" ]]
                        then
                            dir="$i"
                        else
                            tue-install-error "Unknown input variable $i"
                        fi
                        ;;
                esac
            done
        else
            show_help="true"
        fi

        if [[ "${show_help}" == "true" ]]
        then
            echo "Usage: tue-env init NAME [ DIRECTORY ] [--help] [--targets-url=TARGETS GIT URL] [--create-virtualenv=false|true]"
            return 1
        fi

        [[ -z "${dir}" ]] && dir=${PWD} # If no directory is given, use current directory
        dir="$( realpath "${dir}" )"

        if [ -f "${TUE_DIR}"/user/envs/"${env_name}" ]
        then
            echo "[tue-env] Environment '${env_name}' already exists"
            return 1
        fi

        if [ -d "$dir"/.env ]
        then
            echo "[tue-env] Directory '$dir' is already an environment directory."
            return 1
        fi

        echo "${dir}" > "${TUE_DIR}"/user/envs/"${env_name}"
        # Create .env and .env/setup directories
        mkdir -p "$dir"/.env/setup
        echo -e "#! /usr/bin/env bash\n" > "$dir"/.env/setup/user_setup.bash
        echo "[tue-env] Created new environment ${env_name}"

        if [[ -n "${targets_url}" ]]
        then
            tue-env init-targets "${env_name}" "${targets_url}"
        fi

        if [[ "${create_venv}" == "true" ]]
        then
            tue-env init-venv "${env_name}"
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
            local PURGE env
            PURGE=false
            env=
            for i in "$@"
            do
                case $i in
                    --purge)
                        PURGE=true
                        ;;
                    --*)
                        echo "[tue-env] Unknown option $i"
                        ;;
                    *)
                        # Read only the first passed environment name and ignore
                        # the rest
                        if [ -z "${env}" ]
                        then
                            env=$i
                        fi
                        ;;
                esac
            done
        fi

        if [ ! -f "$TUE_DIR"/user/envs/"$env" ]
        then
            echo "[tue-env] No such environment: '$env'."
            return 1
        fi

        local dir
        dir=$(cat "$TUE_DIR"/user/envs/"$env")
        rm "$TUE_DIR"/user/envs/"$env"

        if [[ -d ${dir} ]]
        then
            if [ $PURGE == "false" ]
            then
                dir_moved=$dir.$(date +%F_%R)
                mv "${dir}" "${dir_moved}"
                # shellcheck disable=SC1078,SC1079
                echo """[tue-env] Removed environment '${env}'
Moved environment directory from '${dir}' to '${dir_moved}'"""
            else
                rm -rf "${dir}"
                # shellcheck disable=SC1078,SC1079
                echo """[tue-env] Removed environment '$env'
Purged environment directory '${dir}'"""
            fi
        else
            # shellcheck disable=SC1078,SC1079
            echo """[tue-env] Removed environment '${env}'
Environment directory '${dir}' didn't exist (anymore)"""
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

        # shellcheck disable=SC1091
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

    elif [[ $cmd == "unset-default" ]]
    then
        if [ -n "$1" ]
        then
            echo "Usage: tue-env unset-default"
            echo "No arguments allowed"
        fi

        if [[ ! -f "${TUE_DIR}"/user/config/default_env ]]
        then
            echo "[tue-env] No default environment set, nothing to unset"
            return 1
        fi
        local default_env
        default_env=$(cat "${TUE_DIR}"/user/config/default_env)
        rm -f "${TUE_DIR}"/user/config/default_env
        echo "[tue-env] Default environment '${default_env}' unset"
        return 0

    elif [[ $cmd == "init-targets" ]]
    then
        if [ -z "$1" ] || { [ -z "$TUE_ENV" ] && [ -z "$2" ]; }
        then
            echo "Usage: tue-env init-targets [ENVIRONMENT] TARGETS_GIT_URL"
            return 1
        fi

        local env url
        env=$1
        url=$2
        if [ -z "$url" ]
        then
            env=$TUE_ENV
            if [ -z "$env" ]
            then
                # This shouldn't be possible logical, should have exited after printing usage
                echo "[tue-env](init-targets) no environment set or provided"
                return 1
            fi
            url=$1
        fi

        local tue_env_dir
        tue_env_dir=$(cat "$TUE_DIR"/user/envs/"$env")
        local tue_env_targets_dir
        tue_env_targets_dir=$tue_env_dir/.env/targets

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
        local env
        env=$1
        [ -n "$env" ] || env=$TUE_ENV

        if [ -n "$env" ]
        then
            local tue_env_dir
            tue_env_dir=$(cat "$TUE_DIR"/user/envs/"$env")
            cd "${tue_env_dir}"/.env/targets || { echo -e "Targets directory '${tue_env_dir}/.env/targets' (environment '${env}') does not exist"; return 1; }
        fi

    elif [[ $cmd == "init-venv" ]]
    then
        local env
        env=$1
        [ -n "${env}" ] || env=${TUE_ENV}

        if [[ -z "${env}" ]]
        then
            echo "[tue-env](init-venv) no environment set or provided"
            echo "Usage: tue-env init-venv [ NAME ]"
            return 1
        fi

        python3 -c "import virtualenv" 2>/dev/null ||
        { echo -e "[tue-env](init-venv) 'virtualenv' module is not found. Make sure you install it 'sudo apt-get install python3-virtualenv'"; return 1; }

        local tue_env_dir
        tue_env_dir=$(cat "${TUE_DIR}"/user/envs/"${env}")
        local venv_dir
        venv_dir=${tue_env_dir}/.venv/${env}

        if [ -d "$tue_env_targets_dir" ]
        then
            local targets_dir_moved
            targets_dir_moved=$tue_env_targets_dir.$(date +%F_%R)
            mv -f "$tue_env_targets_dir" "$targets_dir_moved"
            echo "[tue-env] Moved old targets of environment '$env' to $targets_dir_moved"
        fi

        if [[ -d "${venv_dir}" ]]
        then
            local venv_dir_moved
            venv_dir_moved=${venv_dir}.$(date +%F_%R)
            if [[ $(basename "${VIRTUAL_ENV}") == "${env}" ]]
            then
                echo "[tue-env](init-venv) deactivating currently active virtualenv of environment '${env}'"
                deactivate
            fi
            mv -f "${venv_dir}" "${venv_dir_moved}"
            echo "[tue-env] Moved old virtualenv of environment '${env}' to ${venv_dir_moved}"
            echo "Don't use it anymore as its old path is hardcoded in the virtualenv"
        fi

        python3 -m virtualenv "${venv_dir}" -q --system-site-packages --symlinks 2>/dev/null
        echo "[tue-env] Initialized virtualenv of environment '${env}'"

        if [ "${env}" == "${TUE_ENV}" ]
        then
            local tue_env_dir
            tue_env_dir=$(cat "${TUE_DIR}"/user/envs/"${env}")
            # shellcheck disable=SC1090
            source "${tue_env_dir}"/.venv/"${env}"/bin/activate
            echo "[tue-env] Activated new virtualenv of currently active environment '${env}'"
        fi

    elif [[ $cmd == "config" ]]
    then
        local env
        env=$1
        shift
        [ -n "$env" ] || env=$TUE_ENV

        "$TUE_DIR"/setup/tue-env-config.bash "$env" "$@"

        if [ "$env" == "$TUE_ENV" ]
        then
            local tue_env_dir
            tue_env_dir=$(cat "$TUE_DIR"/user/envs/"$env")
            # shellcheck disable=SC1091
            source "$tue_env_dir"/.env/setup/user_setup.bash
        fi

    elif [[ $cmd == "cd" ]]
    then
        local env
        env=$1
        [ -n "$env" ] || env=$TUE_ENV

        if [ -n "$env" ]
        then
            local tue_env_dir
            tue_env_dir=$(cat "$TUE_DIR"/user/envs/"$env")
            cd "${tue_env_dir}" || { echo -e "Environment directory '${tue_env_dir}' (environment '${env}') does not exist"; return 1; }
        else
            echo "[tue-env](cd) no environment set or provided"
            return 1
        fi

    elif [[ $cmd == "list" ]]
    then
        [ -d "$TUE_DIR"/user/envs ] || return 0

        for env in "$TUE_DIR"/user/envs/*
        do
            basename "$env"
        done

    elif [[ $cmd == "current" ]]
    then
        if [[ -n $TUE_ENV ]]
        then
            echo "$TUE_ENV"
        else
            echo "[tue-env] no environment set"
        fi

    else
        echo "[tue-env] Unknown command: '$cmd'"
        return 1
    fi
}

export -f tue-env

# ----------------------------------------------------------------------------------------------------

function _tue-env
{
    local cur
    cur=${COMP_WORDS[COMP_CWORD]}

    if [ "$COMP_CWORD" -eq 1 ]
    then
        mapfile -t COMPREPLY < <(compgen -W "init list switch current remove cd set-default config init-targets targets init-venv" -- "$cur")
    else
        local cmd
        cmd=${COMP_WORDS[1]}
        if [[ $cmd == "switch" ]] || [[ $cmd == "remove" ]] || [[ $cmd == "cd" ]] || [[ $cmd == "set-default" ]] || [[ $cmd == "init-targets" ]] || [[ $cmd == "targets" ]] || [[ $cmd == "init-venv" ]]
        then
            if [ "$COMP_CWORD" -eq 2 ]
            then
                local envs
                [ -d "$TUE_DIR"/user/envs ] && envs=$(ls "$TUE_DIR"/user/envs)

                mapfile -t COMPREPLY < <(compgen -W "$envs" -- "$cur")

            elif [[ $cmd == "remove" ]] && [ "$COMP_CWORD" -eq 3 ]
            then
                local IFS
                IFS=$'\n'
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
