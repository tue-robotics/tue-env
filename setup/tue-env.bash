#! /usr/bin/env bash

# ----------------------------------------------------------------------------------------------------
#                                              TUE-ENV
# ----------------------------------------------------------------------------------------------------

function tue-env
{
    local show_help
    show_help="false" # Default value
    if [[ -z "$1" ]]
    then
        show_help="true"
    else
        case $1 in
            --help | -h )
                show_help="true"
                ;;
        esac
    fi

    if [[ "${show_help}" == "true" ]]
    then
        # shellcheck disable=SC1078,SC1079
        echo """tue-env is a tool for switching between different installation environments.

    Usage: tue-env COMMAND [ARG1 ARG2 ...]

    Possible commands:

        init           - Initializes new environment
        remove/rm      - Removes an existing environment
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

    Possible options:
        --help, -h     - Show this help message and exit
"""
        return 1
    fi

    local cmd
    cmd=$1
    shift

    # Make sure the correct directories are there
    mkdir -p "$TUE_DIR"/user/envs

    local create_venv dir env_name targets_url venv_include_system_site
    create_venv="true"
    venv_include_system_site="true"

    if [[ ${cmd} == "init" ]]
    then
        if [[ -z "$1" ]]
        then
            show_help="true"
        else
            for i in "$@"
            do
                case $i in
                    --targets-url=* )
                        targets_url="${i#*=}" ;;
                    --create-virtualenv=* )
                        create_venv="${i#*=}" ;;
                    --virtualenv-include-system-site-packages=* )
                        venv_include_system_site="${i#*=}" ;;
                    --help | -h )
                        show_help="true"
                        break
                        ;;
                    * )
                        if [[ -z "${env_name}" ]]
                        then
                            env_name="$i"
                        elif [[ -z "${dir}" ]]
                        then
                            dir="$i"
                        else
                            tue-install-error "Unknown input variable $i"
                        fi
                        ;;
                esac
            done
        fi

        if [[ "${show_help}" == "true" ]]
        then
            echo "Usage: tue-env init NAME [DIRECTORY] [--help|-h] [--targets-url=TARGETS GIT URL] [--create-virtualenv=false|TRUE] [--virtualenv-include-system-site-packages=false|TRUE]"
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
            tue-env init-venv "${env_name}" --include-system-site-packages="${venv_include_system_site}"
        fi

    elif [[ ${cmd} == "remove" || ${cmd} == "rm" ]]
    then
        # Set purge to be false by default
        local PURGE env
        PURGE=false
        if [[ -z "$1" ]]
        then
            show_help="true"
        else
            for i in "$@"
            do
                case $i in
                    --purge)
                        PURGE=true
                        ;;
                    --help | -h )
                        show_help="true"
                        break
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

        if [[ ${show_help} == "true" ]]
        then
            # shellcheck disable=SC1078,SC1079
            echo """Usage: tue-env remove [options] ENVIRONMENT
options:
    --help, -h
        Show this help message and exit
    --purge
        Using this would completely remove the selected ENVIRONMENT if it exists"""
            return 1
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

    elif [[ ${cmd} == "switch" ]]
    then
        [[ -z "$1" ]] && show_help="true"
        for i in "$@"
        do
            [[ ${show_help} == "true" ]] && break
            case $i in
                --help | -h )
                    show_help="true" ;;
            esac
        done

        if [[ ${show_help} == "true" ]]
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

    elif [[ ${cmd} == "set-default" ]]
    then
        if [[ -z "$1" ]]
        then
            show_help="true"
        else
            for i in "$@"
            do
                case $i in
                    --help | -h )
                        show_help="true"
                        break
                        ;;
                esac
            done
        fi

        if [[ ${show_help} == "true" ]]
        then
            echo "Usage: tue-env set-default ENVIRONMENT"
            return 1
        fi

        mkdir -p "$TUE_DIR"/user/config
        echo "$1" > "$TUE_DIR"/user/config/default_env
        echo "[tue-env] Default environment set to $1"

    elif [[ ${cmd} == "unset-default" ]]
    then
        [[ -n "$1" ]] && show_help="true" # No arguments allowed

        if [[ ${show_help} == "true" ]]
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

    elif [[ ${cmd} == "init-targets" ]]
    then
        if { [[ -z "$1" ]] || { [ -z "${TUE_ENV}" ] && [ -z "$2" ]; }; }
        then
            show_help="true"
        else
            for i in "$@"
            do
                case $i in
                    --help | -h )
                        show_help="true"
                        break
                        ;;
                esac
            done
        fi

        if [[ ${show_help} == "true" ]]
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

    elif [[ ${cmd} == "targets" ]]
    then
        for i in "$@"
        do
            case $i in
                --help | -h )
                    show_help="true"
                    break
                    ;;
            esac
        done

        if [[ ${show_help} == "true" ]]
        then
            echo "Usage: tue-env targets [ENVIRONMENT]"
            return 1
        fi

        local env
        env=$1
        [ -n "$env" ] || env=$TUE_ENV

        if [ -n "$env" ]
        then
            local tue_env_dir
            tue_env_dir=$(cat "$TUE_DIR"/user/envs/"$env")
            cd "${tue_env_dir}"/.env/targets || { echo -e "Targets directory '${tue_env_dir}/.env/targets' (environment '${env}') does not exist"; return 1; }
        fi

    elif [[ ${cmd} == "init-venv" ]]
    then
        local env include_system_site
        include_system_site="true"
        for i in "$@"
        do
            case $i in
                --help | -h )
                    show_help="true"
                    break
                    ;;
                --include-system-site-packages=* )
                    include_system_site="${i#*=}" ;;
                --* )
                    echo "[tue-env] Unknown option $i" ;;
                * )
                    if [[ -z "${env}" ]]
                    then
                        env=$i
                    else
                        echo "[tue-env] Unknown input variable $i"
                        return 1
                    fi
                    ;;
            esac
        done

        [[ -n "${env}" ]] || env=${TUE_ENV}
        [[ -z "${env}" ]] && show_help="true" && echo "[tue-env](init-venv) no environment set or provided"

        if [[ ${show_help} == "true" ]]
        then
            echo "Usage: tue-env init-venv [ENVIRONMENT] [--include-system-site-packages=false|true]"
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

        local system_site_args
        if [[ "${include_system_site}" == "true" ]]
        then
            system_site_args="--system-site-packages"
        fi
        python3 -m virtualenv "${venv_dir}" ${system_site_args:+${system_site_args} }--symlinks -q 2>/dev/null
        echo "[tue-env] Initialized virtualenv of environment '${env}'"

        if [ "${env}" == "${TUE_ENV}" ]
        then
            local tue_env_dir
            tue_env_dir=$(cat "${TUE_DIR}"/user/envs/"${env}")
            # shellcheck disable=SC1090
            source "${tue_env_dir}"/.venv/"${env}"/bin/activate
            echo "[tue-env] Activated new virtualenv of currently active environment '${env}'"
        fi

    elif [[ ${cmd} == "config" ]]
    then
        for i in "$@"
        do
            case $i in
                --help | -h )
                    show_help="true"
                    break
                    ;;
            esac
        done

        if [[ ${show_help} == "true" ]]
        then
            echo "Usage: tue-env config [ENVIRONMENT] [FUNCTION]"
            return 1
        fi

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

    elif [[ ${cmd} == "cd" ]]
    then
        for i in "$@"
        do
            case $i in
                --help | -h )
                    show_help="true"
                    break
                    ;;
            esac
        done

        if [[ ${show_help} == "true" ]]
        then
            echo "Usage: tue-env cd [ENVIRONMENT]"
            return 1
        fi

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

    elif [[ ${cmd} == "list" ]]
    then
        [ -d "$TUE_DIR"/user/envs ] || return 0

        for env in "$TUE_DIR"/user/envs/*
        do
            basename "$env"
        done

    elif [[ ${cmd} == "current" ]]
    then
        if [[ -n $TUE_ENV ]]
        then
            echo "$TUE_ENV"
        else
            echo "[tue-env] no environment set"
        fi

    else
        echo "[tue-env] Unknown command: '${cmd}'"
        return 1
    fi
}

export -f tue-env

# ----------------------------------------------------------------------------------------------------

function _tue-env
{
    local cur
    cur=${COMP_WORDS[COMP_CWORD]}

    local help_options
    help_options="'-h' '--help'"

    if [ "$COMP_CWORD" -eq 1 ]
    then
        mapfile -t COMPREPLY < <(compgen -W "init list switch current remove rm cd set-default config init-targets targets init-venv ${help_options}" -- "$cur")
    else
        local cmd
        cmd=${COMP_WORDS[1]}
        if [[ ${cmd} == "switch" ]] || [[ ${cmd} == "remove" || ${cmd} == "rm" ]] || [[ ${cmd} == "cd" ]] || [[ ${cmd} == "set-default" ]] || [[ ${cmd} == "init-targets" ]] || [[ ${cmd} == "targets" ]] || [[ ${cmd} == "init-venv" ]]
        then
            if [ "$COMP_CWORD" -eq 2 ]
            then
                local envs
                [ -d "$TUE_DIR"/user/envs ] && envs=$(ls "$TUE_DIR"/user/envs)

                mapfile -t COMPREPLY < <(compgen -W "${envs} ${help_options}" -- "${cur}")

            elif [[ ${cmd} == "remove" || ${cmd} == "rm" ]] && [ "$COMP_CWORD" -eq 3 ]
            then
                mapfile -t COMPREPLY < <(compgen -W "'--purge' ${help_options}" -- "${cur}")
            elif [[ ${cmd} == "init-venv" ]] && [ "$COMP_CWORD" -eq 3 ]
            then
                mapfile -t COMPREPLY < <(compgen -W "'--include-system-site-packages=false' '--include-system-site-packages=true' ${help_options}" -- "${cur}")
            fi
        elif [[ ${cmd} == "config" ]]
        then
            if [ "$COMP_CWORD" -eq 2 ]
            then
                local envs
                [ -d "$TUE_DIR/user/envs" ] && envs=$(ls "$TUE_DIR"/user/envs)
                mapfile -t COMPREPLY < <(compgen -W "${envs} ${help_options}" -- "$cur")
            fi
            if [ "$COMP_CWORD" -eq 3 ]
            then
                local functions
                functions=$(grep 'function ' "$TUE_DIR"/setup/tue-env-config.bash | awk '{print $2}' | grep "tue-env-")
                functions=${functions//tue-env-/}
                mapfile -t COMPREPLY < <(compgen -W "${functions} ${help_options}" -- "$cur")
            fi
        fi
    fi
}
complete -F _tue-env tue-env
