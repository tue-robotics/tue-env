#! /usr/bin/env bash

# ----------------------------------------------------------------------------------------------------
#                                        TUE-ENV IMPLEMENTATION
# ----------------------------------------------------------------------------------------------------

function _tue-env-deactivate-current-env
{
    # The ledger is the sole authority when it holds anything: it recorded everything the load did,
    # including everything the virtual environment's `activate` did, so `deactivate` is not called any
    # more and its _OLD_* variables are unset with everything else.
    if declare -F _tue-env-track-revert > /dev/null && _tue-env-track-revert
    then
        return 0
    fi

    # Empty ledger: a shell started before change tracking existed. Behave exactly as this function
    # did before. A non-interactive child shell has no ledger either, but it never gets this far:
    # this function is not exported, so `tue-env deactivate` fails to find it and reports the
    # failure to the user instead - a pre-existing limitation, unchanged by change tracking.

    # Deactivate the old virtualenv if it exists
    if [[ -n ${VIRTUAL_ENV} ]]
    then
        echo "[tue-env](deactivate) deactivating old virtualenv"
        deactivate || { echo "[tue-env](deactivate) Failed to deactivate the old virtualenv"; return 1; }
    fi

    echo "[tue-env](deactivate) Unsetting all TUE_ENV* of the old environment: '${TUE_ENV}'"
    for var in ${!TUE_ENV*}
    do
        unset "${var}"
    done
    for var in ${!ROS_*}
    do
        unset "${var}"
    done
    unset AMENT_PREFIX_PATH
    unset COLCON_PREFIX_PATH
    unset PYTHONPATH

    return 0
}

function _tue-env-compare-version
{
    # _tue-env-compare-version version requirement
    # Example usage: _tue-env-compare-version 1.2.3 ">=1.2.0"
    if [[ -z "$2" ]]
    then
        echo "[tue-env](compare-version) Version and requirement must be provided." >&2
        return 1
    fi

    local version requirement dpkg_op req_version

    version=$1
    requirement=$2

    # Check if the version is a valid semantic version
    if ! [[ "${version}" =~ ^[0-9]+\.[0-9]+(\.[0-9]+)?$ ]]
    then
        echo "[tue-env](compare-version) Invalid version format: ${version}" >&2
        return 1
    fi

    # Parse requirement
    operator="${requirement%%[0-9]*}" # Extracts comparison operator
    req_version="${requirement#"${operator}"}" # Extracts required version

    # Map operators for dpkg
    case "${operator}" in
        '>=')
            dpkg_op="ge" ;;
        '<=')
            dpkg_op="le" ;;
        '>')
            dpkg_op="gt" ;;
        '<')
            dpkg_op="lt" ;;
        '=')
            dpkg_op="eq" ;;
        '==')
            dpkg_op="eq" ;;
        '!=')
            dpkg_op="ne" ;;
        * )
            echo "[tue-env](compare-version) Unknown operator ${operator}" >&2; return 1 ;;
    esac

# Compare using dpkg --compare-versions
if dpkg --compare-versions "${version}" "${dpkg_op}" "${req_version}"
then
    return 0
else
    return 1
fi
}


# ----------------------------------------------------------------------------------------------------
#                                              TUE-ENV
# ----------------------------------------------------------------------------------------------------

function tue-env
{
    # Every local here carries the __tue_env_ prefix, and that is load-bearing rather than cosmetic.
    # bash scopes locals dynamically, so a snapshot taken by setup/tue-env-track.bash sees the locals
    # of every function still on the stack - and this dispatcher is on the stack for all three of
    # _tue-env-track-begin, -commit and -revert. An unprefixed local would be recorded in the ledger
    # as a change the environment made, would make the revert compare against this function's own
    # copy instead of the real global of the same name, and could be unset out from under the call.
    local __tue_env_show_help
    __tue_env_show_help="false" # Default value
    if [[ -z "$1" ]]
    then
        __tue_env_show_help="true"
    else
        case $1 in
            --help | -h )
                __tue_env_show_help="true" ;;
        esac
    fi

    if [[ "${__tue_env_show_help}" == "true" ]]
    then
        # shellcheck disable=SC1078,SC1079
        echo """tue-env is a tool for switching between different installation environments.

    Usage: tue-env COMMAND [ARG1 ARG2 ...]

    Possible commands:

        init           - Initializes new environment
        remove/rm      - Removes an existing environment
        switch         - Switch to a different environment
        deactivate     - Unloads the current environment from this shell
        changes        - Shows what loading the current environment changed in this shell
        config         - Configures current environment
        set-default    - Set default environment
        unset-default  - Unset default environment
        init-targets   - (Re-)Initialize the target list
        targets        - Changes directory to targets directory
        init-venv      - Initializes a virtualenv
        remove-venv/
        rm-venv        - Removes a virtualenv
        list           - List all possible environments
        current        - Shows current environment
        cd             - Changes directory to environment directory

    Possible options:
        --help, -h     - Show this help message and exit
"""
        return 1
    fi

    local __tue_env_cmd
    __tue_env_cmd=$1
    shift

    # Make sure the correct directories are there
    mkdir -p "$TUE_DIR"/user/envs

    # __tue_env_i is the option-parsing loop variable of every branch below; declaring it here keeps
    # it out of the global namespace, where it used to clobber a variable named `i` in the caller.
    local __tue_env_i
    local __tue_env_create_venv __tue_env_targets_url __tue_env_tue_env __tue_env_tue_env_dir
    local __tue_env_venv_include_system_site __tue_env_venv_setuptools
    __tue_env_create_venv="true"
    __tue_env_venv_include_system_site="true"
    __tue_env_venv_setuptools="false"

    if [[ ${__tue_env_cmd} == "init" ]]
    then
        if [[ -z "$1" ]]
        then
            __tue_env_show_help="true"
        else
            for __tue_env_i in "$@"
            do
                case $__tue_env_i in
                    --targets-url=* )
                        __tue_env_targets_url="${__tue_env_i#*=}" ;;
                    --create-virtualenv=* )
                        __tue_env_create_venv="${__tue_env_i#*=}" ;;
                    --virtualenv-include-system-site-packages=* )
                        __tue_env_venv_include_system_site="${__tue_env_i#*=}" ;;
                    --virtualenv-install-setuptools=* )
                        __tue_env_venv_setuptools="${__tue_env_i#*=}" ;;
                    --help | -h )
                        __tue_env_show_help="true"
                        break
                        ;;
                    --* )
                        echo "[tue-env](init) Unknown option $__tue_env_i"
                        __tue_env_show_help="true"
                        ;;
                    * )
                        if [[ -z "${__tue_env_tue_env}" ]]
                        then
                            __tue_env_tue_env="$__tue_env_i"
                        elif [[ -z "${__tue_env_tue_env_dir}" ]]
                        then
                            __tue_env_tue_env_dir="$__tue_env_i"
                        else
                            echo "[tue-env](init) Unknown input variable $__tue_env_i"
                            __tue_env_show_help="true"
                        fi
                        ;;
                esac
            done
        fi

        [[ -z "${__tue_env_tue_env}" ]] && __tue_env_show_help="true" && echo "[tue-env](init) no environment provided"

        if [[ "${__tue_env_show_help}" == "true" ]]
        then
            echo "Usage: tue-env init NAME [DIRECTORY] [--help|-h] [--targets-url=TARGETS_GIT_URL] [--create-virtualenv=false|TRUE] [--virtualenv-include-system-site-packages=false|TRUE] [--virtualenv-install-setuptools=FALSE|true]"
            return 1
        fi

        [[ -z "${__tue_env_tue_env_dir}" ]] && __tue_env_tue_env_dir=${PWD} # If no directory is given, use current directory
        __tue_env_tue_env_dir="$( realpath "${__tue_env_tue_env_dir}" )"

        if [ -f "${TUE_DIR}"/user/envs/"${__tue_env_tue_env}" ]
        then
            echo "[tue-env](init) Environment '${__tue_env_tue_env}' already exists"
            return 1
        fi

        if [[ -d "${__tue_env_tue_env_dir}"/.env ]]
        then
            echo "[tue-env](init) Directory '${__tue_env_tue_env_dir}' is already an environment directory."
            return 1
        fi

        echo "${__tue_env_tue_env_dir}" > "${TUE_DIR}"/user/envs/"${__tue_env_tue_env}"
        # Create .env and .env/setup directories
        mkdir -p "${__tue_env_tue_env_dir}"/.env/setup
        echo -e "#! /usr/bin/env bash\n" > "${__tue_env_tue_env_dir}"/.env/setup/user_setup.bash
        echo "[tue-env](init) Created new environment ${__tue_env_tue_env}"

        if [[ -n "${__tue_env_targets_url}" ]]
        then
            tue-env init-targets "${__tue_env_tue_env}" "${__tue_env_targets_url}" || return 1
        fi

        if [[ "${__tue_env_create_venv}" == "true" ]]
        then
            tue-env init-venv "${__tue_env_tue_env}" --include-system-site-packages="${__tue_env_venv_include_system_site}" --install-setuptools="${__tue_env_venv_setuptools}" || return 1
        fi

    elif [[ ${__tue_env_cmd} == "remove" || ${__tue_env_cmd} == "rm" ]]
    then
        # Set __tue_env_purge to be false by default
        local __tue_env_purge __tue_env_tue_env
        __tue_env_purge="false"
        if [[ -z "$1" ]]
        then
            __tue_env_show_help="true"
        else
            for __tue_env_i in "$@"
            do
                case $__tue_env_i in
                    --purge)
                        __tue_env_purge=true ;;
                    --help | -h )
                        __tue_env_show_help="true"
                        break
                        ;;
                    --*)
                        echo "[tue-env](rm) Unknown option $__tue_env_i"
                        __tue_env_show_help="true"
                        ;;
                    *)
                        # Read only the first passed environment name and ignore
                        # the rest
                        if [ -z "${__tue_env_tue_env}" ]
                        then
                            __tue_env_tue_env=$__tue_env_i
                        else
                            echo "[tue-env](rm) Unknown input variable $__tue_env_i"
                            __tue_env_show_help="true"
                        fi
                        ;;
                esac
            done
        fi

        [[ -z "${__tue_env_tue_env}" ]]  && __tue_env_show_help="true" && echo "[tue-env](rm) no environment provided"

        if [[ ${__tue_env_show_help} == "true" ]]
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

        [[ -f "${TUE_DIR}"/user/envs/"${__tue_env_tue_env}" ]] || { echo "[tue-env](rm) No such environment: '${__tue_env_tue_env}'"; return 1; }

        if [[ "${__tue_env_tue_env}" == "${TUE_ENV}" ]]
        then
            echo "[tue-env](rm) The environment '${__tue_env_tue_env}' is currently active. Deactivating it first."
            _tue-env-deactivate-current-env || { echo "[tue-env](rm) Failed to deactivate the current environment, don't use this terminal anymore, open a new terminal"; return 1; }
        fi

        # Unset the default environment if it is the one being removed
        if [[ -f "${TUE_DIR}"/user/config/default_env ]] && [[ "$(cat "${TUE_DIR}"/user/config/default_env)" == "${__tue_env_tue_env}" ]]
        then
            echo "[tue-env](rm) Unsetting the default environment '${__tue_env_tue_env}'"
            tue-env unset-default || return 1
        fi

        local __tue_env_tue_env_dir
        __tue_env_tue_env_dir=$(cat "${TUE_DIR}"/user/envs/"${__tue_env_tue_env}")
        rm "${TUE_DIR}"/user/envs/"${__tue_env_tue_env}"

        if [[ -d ${__tue_env_tue_env_dir} ]]
        then
            if [[ ${__tue_env_purge} == "false" ]]
            then
                local __tue_env_tue_env_dir_moved
                __tue_env_tue_env_dir_moved=${__tue_env_tue_env_dir}.$(date +%F_%H%M%S)
                mv "${__tue_env_tue_env_dir}" "${__tue_env_tue_env_dir_moved}"
                # shellcheck disable=SC1078,SC1079
                echo """[tue-env] Removed environment '${__tue_env_tue_env}'
Moved environment directory from '${__tue_env_tue_env_dir}' to '${__tue_env_tue_env_dir_moved}'"""
            else
                rm -rf "${__tue_env_tue_env_dir}"
                # shellcheck disable=SC1078,SC1079
                echo """[tue-env] Removed environment '${__tue_env_tue_env}'
Purged environment directory '${__tue_env_tue_env_dir}'"""
            fi
        else
            # shellcheck disable=SC1078,SC1079
            echo """[tue-env] Removed environment '${__tue_env_tue_env}'
Environment directory '${__tue_env_tue_env_dir}' didn't exist (anymore)"""
        fi

    elif [[ ${__tue_env_cmd} == "deactivate" ]]
    then
        local __tue_env_dry_run
        __tue_env_dry_run="false"
        for __tue_env_i in "$@"
        do
            case $__tue_env_i in
                --help | -h )
                    __tue_env_show_help="true"
                    break
                    ;;
                --dry-run )
                    __tue_env_dry_run="true" ;;
                --*)
                    echo "[tue-env](deactivate) Unknown option $__tue_env_i"
                    __tue_env_show_help="true"
                    ;;
                * )
                    echo "[tue-env](deactivate) Unknown input variable $__tue_env_i"
                    __tue_env_show_help="true"
                    ;;
            esac
        done

        if [[ ${__tue_env_show_help} == "true" ]]
        then
            # shellcheck disable=SC1078,SC1079
            echo """Usage: tue-env deactivate [options]

    Possible options:
                --dry-run      - Show what unloading would do, without changing anything
                --help, -h     - Show this help message and exit
"""
            return 1
        fi

        # ${TUE_ENV} predates the ledger and cannot be the only authority any more: unsetting a
        # variable by hand is precisely what change tracking promises to tolerate, and a shell whose
        # TUE_ENV the user removed still holds everything else the load did, together with a ledger
        # that says how to undo it. Only when the marker AND the ledger are gone is there nothing to
        # unload. The marker is tested first so that it short-circuits: a child `bash` inherits
        # TUE_ENV and the exported `tue-env` but none of the unexported helpers, and has to keep
        # reaching the failure it has always reached rather than being told nothing is active. The
        # redirection covers the shell that predates change tracking, where the tracker's entry point
        # does not exist at all.
        if [[ -z "${TUE_ENV}" ]] && ! _tue-env-track-active 2> /dev/null
        then
            echo "[tue-env](deactivate) No environment is currently active"
            return 1
        fi

        if [[ "${__tue_env_dry_run}" == "true" ]]
        then
            if ! declare -F _tue-env-track-report > /dev/null
            then
                echo "[tue-env](deactivate) change tracking is not available in this shell"
                return 1
            fi
            if ! _tue-env-track-report revert
            then
                echo "[tue-env](deactivate) no tracked changes; this shell was started before change tracking"
                return 1
            fi
            return 0
        fi

        if [[ -n "${TUE_ENV}" ]]
        then
            echo "[tue-env](deactivate) Deactivating the current environment '${TUE_ENV}'"
        else
            # Reached only when the ledger, not the marker, is what says an environment is loaded.
            echo "[tue-env](deactivate) Deactivating the tracked environment"
        fi
        _tue-env-deactivate-current-env || { echo "[tue-env](deactivate) Failed to deactivate the current environment, don't use this terminal anymore, open a new terminal"; return 1; }

        return 0

    elif [[ ${__tue_env_cmd} == "changes" ]]
    then
        for __tue_env_i in "$@"
        do
            case $__tue_env_i in
                --help | -h )
                    __tue_env_show_help="true"
                    break
                    ;;
                --*)
                    echo "[tue-env](changes) Unknown option $__tue_env_i"
                    __tue_env_show_help="true"
                    ;;
                * )
                    echo "[tue-env](changes) Unknown input variable $__tue_env_i"
                    __tue_env_show_help="true"
                    ;;
            esac
        done

        if [[ ${__tue_env_show_help} == "true" ]]
        then
            # shellcheck disable=SC1078,SC1079
            echo """Usage: tue-env changes [options]

    Shows what loading the current environment changed in this shell.

    Possible options:
                --help, -h     - Show this help message and exit
"""
            return 1
        fi

        # The same pair as `deactivate` gates on, so the two answer alike; see the comment there.
        if [[ -z "${TUE_ENV}" ]] && ! _tue-env-track-active 2> /dev/null
        then
            echo "[tue-env](changes) No environment is currently active"
            return 1
        fi

        if ! declare -F _tue-env-track-report > /dev/null
        then
            echo "[tue-env](changes) change tracking is not available in this shell"
            return 1
        fi

        if ! _tue-env-track-report changes
        then
            echo "[tue-env](changes) no tracked changes; this shell was started before change tracking"
            return 1
        fi

        return 0

    elif [[ ${__tue_env_cmd} == "switch" ]]
    then
        local __tue_env_persistent __tue_env_tue_env
        __tue_env_persistent="false"
        if [[ -z "$1" ]]
        then
            __tue_env_show_help="true"
        else
            for __tue_env_i in "$@"
            do
                case $__tue_env_i in
                    --help | -h )
                        __tue_env_show_help="true"
                        break
                        ;;
                    --persistent )
                        __tue_env_persistent="true" ;;
                    --*)
                        echo "[tue-env](switch) Unknown option $__tue_env_i"
                        __tue_env_show_help="true"
                        ;;
                    * )
                        if [[ -z "${__tue_env_tue_env}" ]]
                        then
                            __tue_env_tue_env=$__tue_env_i
                        else
                            echo "[tue-env](switch) Unknown input variable $__tue_env_i"
                            __tue_env_show_help="true"
                        fi
                        ;;
                esac
            done
        fi

        [[ -z "${__tue_env_tue_env}" ]] && __tue_env_show_help="true" && echo "[tue-env](switch) no environment provided"

        if [[ ${__tue_env_show_help} == "true" ]]
        then
            # shellcheck disable=SC1078,SC1079
            echo """Usage: tue-env switch [options] ENVIRONMENT

    Possible options:
        --persistent   - Set the environment as default
        --help, -h     - Show this help message and exit
"""
            return 1
        fi

        [[ -f "${TUE_DIR}"/user/envs/"${__tue_env_tue_env}" ]] || { echo "[tue-env](switch) No such environment: '${__tue_env_tue_env}'"; return 1; }
        local __tue_env_tue_env_dir
        __tue_env_tue_env_dir=$(cat "${TUE_DIR}"/user/envs/"${__tue_env_tue_env}")
        [[ -d "${__tue_env_tue_env_dir}" ]] || { echo "[tue-env](switch) Environment directory '${__tue_env_tue_env_dir}' (environment '${__tue_env_tue_env}') does not exist"; return 1; }

        [[ "${__tue_env_persistent}" == "true" ]] && { tue-env set-default "${__tue_env_tue_env}" || return 1; }

        # The marker-or-ledger pair `deactivate` gates on, inverted: there it says when to refuse,
        # here it says when to unload first; see the comment there for why the marker cannot be the
        # only authority. Asking the marker alone here is worse than wrongly refusing an unload: a shell whose TUE_ENV the user removed still holds everything the first load did,
        # so the new environment would be loaded straight on top of the old one's variables,
        # aliases, functions and PATH entries, with the ledger that says how to undo them never
        # applied.
        if [[ -n "${TUE_ENV}" ]] || _tue-env-track-active 2> /dev/null
        then
            if [[ "${TUE_ENV}" == "${__tue_env_tue_env}" ]]
            then
                echo "[tue-env](switch) Already in the '${__tue_env_tue_env}' environment"
                return 0
            fi
            if [[ -n "${TUE_ENV}" ]]
            then
                echo "[tue-env](switch) Deactivating the current environment '${TUE_ENV}'"
            else
                # Reached only when the ledger, not the marker, is what says an environment is loaded.
                echo "[tue-env](switch) Deactivating the tracked environment"
            fi
            _tue-env-deactivate-current-env || { echo "[tue-env](switch) Failed to deactivate the current environment, don't use this terminal anymore, open a new terminal"; return 1; }
        fi

        # Successful, so we can set the environment variables. Begin the tracked span here rather than
        # in setup.bash, so that TUE_ENV and TUE_ENV_DIR are part of the diff; setup.bash's own
        # begin/commit pair nests inside this one and is a no-op.
        # The status is collected through `||` for the same reason setup.bash's wrapper does it: a
        # begin whose commit is skipped pins the depth counter above zero for the life of the shell,
        # which silently stops all tracking and retains the pre-load snapshot. Declare the local
        # BEFORE the begin, so the commit's snapshot does not see it appear.
        local __tue_env_ret=0
        _tue-env-track-begin

        TUE_ENV=${__tue_env_tue_env}
        export TUE_ENV
        TUE_ENV_DIR=${__tue_env_tue_env_dir}
        export TUE_ENV_DIR

        echo "[tue-env](switch) Loading the new '${TUE_ENV}' environment"
        # shellcheck disable=SC1091
        source "$TUE_DIR"/setup.bash || __tue_env_ret=$?

        _tue-env-track-commit
        return "${__tue_env_ret}"

    elif [[ ${__tue_env_cmd} == "set-default" ]]
    then
        local __tue_env_tue_env
        if [[ -z "$1" ]]
        then
            __tue_env_show_help="true"
        else
            for __tue_env_i in "$@"
            do
                case $__tue_env_i in
                    --help | -h )
                        __tue_env_show_help="true"
                        break
                        ;;
                    --*)
                        echo "[tue-env](set-default) Unknown option $__tue_env_i"
                        __tue_env_show_help="true"
                        ;;
                    * )
                        if [[ -z "${__tue_env_tue_env}" ]]
                        then
                            __tue_env_tue_env=$__tue_env_i
                        else
                            echo "[tue-env](set-default) Unknown input variable $__tue_env_i"
                            __tue_env_show_help="true"
                        fi
                        ;;
                esac
            done
        fi

        [[ -z "${__tue_env_tue_env}" ]] && __tue_env_show_help="true" && echo "[tue-env](set-default) no environment provided"

        if [[ ${__tue_env_show_help} == "true" ]]
        then
            echo "Usage: tue-env set-default ENVIRONMENT"
            return 1
        fi

        mkdir -p "${TUE_DIR}"/user/config
        echo "${__tue_env_tue_env}" > "${TUE_DIR}"/user/config/default_env
        echo "[tue-env](set-default) Default environment set to '${__tue_env_tue_env}'"

    elif [[ ${__tue_env_cmd} == "unset-default" ]]
    then
        [[ -n "$1" ]] && __tue_env_show_help="true" # No arguments allowed

        if [[ ${__tue_env_show_help} == "true" ]]
        then
            echo "Usage: tue-env unset-default"
            echo "No arguments allowed"
        fi

        if [[ ! -f "${TUE_DIR}"/user/config/default_env ]]
        then
            echo "[tue-env](unset-default) No default environment set, nothing to unset"
            return 1
        fi
        local __tue_env_default_env
        __tue_env_default_env=$(cat "${TUE_DIR}"/user/config/default_env)
        rm -f "${TUE_DIR}"/user/config/default_env
        echo "[tue-env](unset-default) Default environment '${__tue_env_default_env}' unset"
        return 0

    elif [[ ${__tue_env_cmd} == "init-targets" ]]
    then
        local __tue_env_tue_env __tue_env_url
        if { [[ -z "$1" ]] || { [ -z "${TUE_ENV}" ] && [ -z "$2" ]; }; }
        then
            __tue_env_show_help="true"
        else
            for __tue_env_i in "$@"
            do
                case $__tue_env_i in
                    --help | -h )
                        __tue_env_show_help="true"
                        break
                        ;;
                    --* )
                        echo "[tue-env](init-targets) Unknown option $__tue_env_i"
                        __tue_env_show_help="true"
                        ;;
                    * )
                        if [[ -z "${__tue_env_tue_env}" ]]
                        then
                            __tue_env_tue_env=$__tue_env_i
                        elif [[ -z "${__tue_env_url}" ]]
                        then
                            __tue_env_url=$__tue_env_i
                        else
                            echo "[tue-env](init-targets) Unknown input variable $__tue_env_i"
                            __tue_env_show_help="true"
                        fi
                        ;;
                esac
            done
        fi

        if [[ -z "${__tue_env_url}" ]]
        then
            __tue_env_url=${__tue_env_tue_env} # If no environment was given, the __tue_env_url was assigned to __tue_env_tue_env
            __tue_env_tue_env=${TUE_ENV}
        fi

        [[ -z "${__tue_env_tue_env}" ]] && __tue_env_show_help="true" && echo "[tue-env](init-targets) no environment set or provided"

        if [[ ${__tue_env_show_help} == "true" ]]
        then
            echo "Usage: tue-env init-targets [ENVIRONMENT] TARGETS_GIT_URL"
            return 1
        fi

        [[ -f "${TUE_DIR}"/user/envs/"${__tue_env_tue_env}" ]] || { echo "[tue-env](init-targets) No such environment: '${__tue_env_tue_env}'"; return 1; }
        local __tue_env_tue_env_dir
        __tue_env_tue_env_dir=$(cat "${TUE_DIR}"/user/envs/"${__tue_env_tue_env}")
        [[ -d "${__tue_env_tue_env_dir}" ]] || { echo "[tue-env](init-targets) Environment directory '${__tue_env_tue_env_dir}' (environment '${__tue_env_tue_env}') does not exist"; return 1; }

        local __tue_env_tue_env_targets_dir
        __tue_env_tue_env_targets_dir=$__tue_env_tue_env_dir/.env/targets

        if [ -d "$__tue_env_tue_env_targets_dir" ]
        then
            local __tue_env_targets_dir_moved
            __tue_env_targets_dir_moved=$__tue_env_tue_env_targets_dir.$(date +%F_%H%M%S)
            mv -f "$__tue_env_tue_env_targets_dir" "$__tue_env_targets_dir_moved"
            echo "[tue-env] Moved old targets of environment '${__tue_env_tue_env}' to ${__tue_env_targets_dir_moved}"
        fi

        git clone --recursive "$__tue_env_url" "$__tue_env_tue_env_targets_dir"
        echo "[tue-env] cloned targets of environment '${__tue_env_tue_env}' from ${__tue_env_url}"

    elif [[ ${__tue_env_cmd} == "targets" ]]
    then
        local __tue_env_tue_env
        for __tue_env_i in "$@"
        do
            case $__tue_env_i in
                --help | -h )
                    __tue_env_show_help="true"
                    break
                    ;;
                --* )
                    echo "[tue-env](targets) Unknown option $__tue_env_i"
                    __tue_env_show_help="true"
                    ;;
                * )
                    if [[ -z "${__tue_env_tue_env}" ]]
                    then
                        __tue_env_tue_env=$__tue_env_i
                    else
                        echo "[tue-env](targets) Unknown input variable $__tue_env_i"
                        __tue_env_show_help="true"
                    fi
                    ;;
            esac
        done

        [[ -n "${__tue_env_tue_env}" ]] || __tue_env_tue_env=${TUE_ENV}
        [[ -z "${__tue_env_tue_env}" ]] && __tue_env_show_help="true" && echo "[tue-env](targets) no environment set or provided"

        if [[ ${__tue_env_show_help} == "true" ]]
        then
            echo "Usage: tue-env targets [ENVIRONMENT]"
            return 1
        fi

        [[ -f "${TUE_DIR}"/user/envs/"${__tue_env_tue_env}" ]] || { echo "[tue-env](targets) No such environment: '${__tue_env_tue_env}'"; return 1; }
        local __tue_env_tue_env_dir
        __tue_env_tue_env_dir=$(cat "${TUE_DIR}"/user/envs/"${__tue_env_tue_env}")
        [[ -d "${__tue_env_tue_env_dir}" ]] || { echo "[tue-env](targets) Environment directory '${__tue_env_tue_env_dir}' (environment '${__tue_env_tue_env}') does not exist"; return 1; }
        cd "${__tue_env_tue_env_dir}"/.env/targets || { echo -e "Targets directory '${__tue_env_tue_env_dir}/.env/targets' (environment '${__tue_env_tue_env}') does not exist"; return 1; }

    elif [[ ${__tue_env_cmd} == "init-venv" ]]
    then
        local __tue_env_include_system_site __tue_env_install_setuptools __tue_env_tue_env
        __tue_env_include_system_site="true"
        __tue_env_install_setuptools="false"
        for __tue_env_i in "$@"
        do
            case $__tue_env_i in
                --help | -h )
                    __tue_env_show_help="true"
                    break
                    ;;
                --include-system-site-packages=* )
                    __tue_env_include_system_site="${__tue_env_i#*=}" ;;
                --install-setuptools=* )
                    __tue_env_install_setuptools="${__tue_env_i#*=}" ;;
                --* )
                    echo "[tue-env] Unknown option $__tue_env_i"
                    __tue_env_show_help="true"
                    ;;
                * )
                    if [[ -z "${__tue_env_tue_env}" ]]
                    then
                        __tue_env_tue_env=$__tue_env_i
                    else
                        echo "[tue-env] Unknown input variable $__tue_env_i"
                        __tue_env_show_help="true"
                    fi
                    ;;
            esac
        done

        [[ -n "${__tue_env_tue_env}" ]] || __tue_env_tue_env=${TUE_ENV}
        [[ -z "${__tue_env_tue_env}" ]] && __tue_env_show_help="true" && echo "[tue-env](init-venv) no environment set or provided"

        if [[ ${__tue_env_show_help} == "true" ]]
        then
            echo "Usage: tue-env init-venv [ENVIRONMENT] [--include-system-site-packages=false|TRUE] [--install-setuptools=FALSE|true]"
            return 1
        fi

        local __tue_env_installed_version __tue_env_parsed_version __tue_env_pkg_name __tue_env_version_requirement
        __tue_env_pkg_name="virtualenv"
        __tue_env_version_requirement=">=20.24.0"
        __tue_env_installed_version=$(/usr/bin/python3 -c "import importlib.metadata; print(importlib.metadata.version('${__tue_env_pkg_name}'))" 2>/dev/null)
        __tue_env_parsed_version=$(echo "${__tue_env_installed_version}" | grep -oE '[0-9]+\.[0-9]+(\.[0-9]+)?' | head -n1)
        if ! _tue-env-compare-version "${__tue_env_parsed_version}" "${__tue_env_version_requirement}"
        then
            /usr/bin/python3 -Ic "import ${__tue_env_pkg_name}" && \
            { echo -e "[tue-env](init-venv) '${__tue_env_pkg_name}(${__tue_env_installed_version})' does not match the required version '${__tue_env_version_requirement}' and is installed via APT." \
            "To prevent any conflicts, first uninstall it: \"sudo apt-get remove python3-${__tue_env_pkg_name}\"" \
            "Make sure you install it \"/usr/bin/python3 -m pip install --user '${__tue_env_pkg_name}${__tue_env_version_requirement}'\""; return 1; }

            { echo -e "[tue-env](init-venv) '${__tue_env_pkg_name}(${__tue_env_installed_version})' doesn't match the required version '${__tue_env_version_requirement}'. " \
            "Make sure you install it \"/usr/bin/python3 -m pip install --user '${__tue_env_pkg_name}${__tue_env_version_requirement}'\""; return 1; }
        fi

        [[ -f "${TUE_DIR}"/user/envs/"${__tue_env_tue_env}" ]] || { echo "[tue-env](init-venv) No such environment: '${__tue_env_tue_env}'"; return 1; }
        local __tue_env_tue_env_dir
        __tue_env_tue_env_dir=$(cat "${TUE_DIR}"/user/envs/"${__tue_env_tue_env}")
        [[ -d "${__tue_env_tue_env_dir}" ]] || { echo "[tue-env](init-venv) Environment directory '${__tue_env_tue_env_dir}' (environment '${__tue_env_tue_env}') does not exist"; return 1; }
        local __tue_env_venv_dir __tue_env_venv_dir_deprecated
        __tue_env_venv_dir=${__tue_env_tue_env_dir}/.env/venv
        __tue_env_venv_dir_deprecated=${__tue_env_tue_env_dir}/.venv/"${__tue_env_tue_env}"

        if [[ -d "${__tue_env_venv_dir}" ]]
        then
            local __tue_env_venv_dir_moved
            __tue_env_venv_dir_moved=${__tue_env_venv_dir}.$(date +%F_%H%M%S)
            if [[ "${VIRTUAL_ENV_PROMPT}" == "${__tue_env_tue_env}" ]]
            then
                echo "[tue-env](init-venv) deactivating currently active virtualenv of environment '${__tue_env_tue_env}'"
                deactivate
            fi
            mv -f "${__tue_env_venv_dir}" "${__tue_env_venv_dir_moved}"
            echo "[tue-env](init-venv) Moved old virtualenv of environment '${__tue_env_tue_env}' to ${__tue_env_venv_dir_moved}"
            echo "Don't use it anymore as its old path is hardcoded in the virtualenv"
        fi
        if [[ -d "${__tue_env_venv_dir_deprecated}" ]]
        then
            local __tue_env_venv_dir_deprecated_moved
            __tue_env_venv_dir_deprecated_moved=${__tue_env_venv_dir_deprecated}.$(date +%F_%H%M%S)
            if [[ $(basename "${VIRTUAL_ENV}") == "${__tue_env_tue_env}" ]]
            then
                echo "[tue-env](init-venv) deactivating currently active virtualenv of environment '${__tue_env_tue_env}'"
                deactivate
            fi
            mv -f "${__tue_env_venv_dir_deprecated}" "${__tue_env_venv_dir_deprecated_moved}"
            echo "[tue-env](init-venv) Moved old virtualenv of environment '${__tue_env_tue_env}' to ${__tue_env_venv_dir_deprecated_moved}"
            echo "Don't use it anymore as its old path is hardcoded in the virtualenv"
        fi

        local __tue_env_system_site_args
        if [[ "${__tue_env_include_system_site}" == "true" ]]
        then
            __tue_env_system_site_args="--system-site-packages"
        fi
        local __tue_env_setuptools_args
        if [[ "${__tue_env_install_setuptools}" != "true" ]]
        then
            __tue_env_setuptools_args="--no-setuptools"
        fi
        /usr/bin/python3 -m virtualenv "${__tue_env_venv_dir}" ${__tue_env_system_site_args:+${__tue_env_system_site_args} }${__tue_env_setuptools_args:+${__tue_env_setuptools_args} }--symlinks --prompt "${__tue_env_tue_env}" -q 2>/dev/null ||
        { echo "[tue-env](init-venv) Failed to initialize virtual environment '${__tue_env_venv_dir}' for environment '${__tue_env_tue_env}'"; return 1; }

        echo "[tue-env](init-venv) Initialized virtualenv of environment '${__tue_env_tue_env}'"

        if [ "${__tue_env_tue_env}" == "${TUE_ENV}" ]
        then
            # No need to check if the environment really exists, as it was checked before
            local __tue_env_tue_env_dir
            __tue_env_tue_env_dir=$(cat "${TUE_DIR}"/user/envs/"${__tue_env_tue_env}")
            # shellcheck disable=SC1091
            source "${__tue_env_tue_env_dir}"/.env/venv/bin/activate
            echo "[tue-env](init-venv) Activated new virtualenv of currently active environment '${__tue_env_tue_env}'"
        fi

    elif [[ ${__tue_env_cmd} == "remove-venv" || ${__tue_env_cmd} == "rm-venv" ]]
    then
        local __tue_env_purge __tue_env_tue_env
        __tue_env_purge="false"
        for __tue_env_i in "$@"
        do
            case $__tue_env_i in
                --help | -h )
                    __tue_env_show_help="true"
                    break
                    ;;
                --purge)
                    __tue_env_purge="true" ;;
                --* )
                    echo "[tue-env](rm-venv) Unknown option $__tue_env_i"
                    __tue_env_show_help="true"
                    ;;
                * )
                    if [[ -z "${__tue_env_tue_env}" ]]
                    then
                        __tue_env_tue_env=$__tue_env_i
                    else
                        echo "[tue-env](rm-venv) Unknown input variable $__tue_env_i"
                        __tue_env_show_help="true"
                    fi
                    ;;
            esac
        done

        [[ -n "${__tue_env_tue_env}" ]] || __tue_env_tue_env=${TUE_ENV}
        [[ -z "${__tue_env_tue_env}" ]] && __tue_env_show_help="true" && echo "[tue-env](rm-venv) no environment set or provided"

        if [[ ${__tue_env_show_help} == "true" ]]
        then
            echo "Usage: tue-env remove-venv [ENVIRONMENT]"
            return 1
        fi

        [[ -f "${TUE_DIR}"/user/envs/"${__tue_env_tue_env}" ]] || { echo "[tue-env](rm-venv) No such environment: '${__tue_env_tue_env}'"; return 1; }
        local __tue_env_tue_env_dir
        __tue_env_tue_env_dir=$(cat "${TUE_DIR}"/user/envs/"${__tue_env_tue_env}")
        [[ -d "${__tue_env_tue_env_dir}" ]] || { echo "[tue-env](rm-venv) Environment directory '${__tue_env_tue_env_dir}' (environment '${__tue_env_tue_env}') does not exist"; return 1; }
        local __tue_env_venv_dir __tue_env_venv_dir_deprecated
        __tue_env_venv_dir=${__tue_env_tue_env_dir}/.env/venv
        __tue_env_venv_dir_deprecated=${__tue_env_tue_env_dir}/.venv/"${__tue_env_tue_env}"


        if [[ -d "${__tue_env_venv_dir}" ]]
        then
            if [[ "${VIRTUAL_ENV_PROMPT}" == "${__tue_env_tue_env}" ]]
            then
                echo "[tue-env](rm-venv) deactivating currently active virtualenv of environment '${__tue_env_tue_env}'"
                deactivate
            fi
            if [[ "${__tue_env_purge}" == "true" ]]
            then
                rm -rf "${__tue_env_venv_dir}"
                echo "[tue-env](rm-venv) Purged virtualenv of environment '${__tue_env_tue_env}'"
                return 0
            else
                local __tue_env_venv_dir_moved
                __tue_env_venv_dir_moved=${__tue_env_venv_dir}.$(date +%F_%H%M%S)
                mv -f "${__tue_env_venv_dir}" "${__tue_env_venv_dir_moved}"
                echo "[tue-env](rm-venv) Moved old virtualenv of environment '${__tue_env_tue_env}' to ${__tue_env_venv_dir_moved}"
            fi
        elif [[ -d "${__tue_env_venv_dir_deprecated}" ]]
        then
          if [[ $(basename "${VIRTUAL_ENV}") == "${__tue_env_tue_env}" ]]
          then
                echo "[tue-env](rm-venv) deactivating currently active virtualenv of environment '${__tue_env_tue_env}'"
                deactivate
            fi
            if [[ "${__tue_env_purge}" == "true" ]]
            then
                rm -rf "${__tue_env_venv_dir}"
                echo "[tue-env](rm-venv) Purged virtualenv of environment '${__tue_env_tue_env}'"
                return 0
            else
                local __tue_env_venv_dir_deprecated_moved
                __tue_env_venv_dir_deprecated_moved=${__tue_env_venv_dir_deprecated}.$(date +%F_%H%M%S)
                mv -f "${__tue_env_venv_dir_deprecated}" "${__tue_env_venv_dir_deprecated_moved}"
                echo "[tue-env](rm-venv) Moved old virtualenv of environment '${__tue_env_tue_env}' to ${__tue_env_venv_dir_deprecated_moved}"
            fi
        else
            echo "[tue-env](rm-venv) No virtualenv found for environment '${__tue_env_tue_env}'"
        fi

    elif [[ ${__tue_env_cmd} == "config" ]]
    then
        local __tue_env_tue_env __tue_env_args
        __tue_env_args=()
        for __tue_env_i in "$@"
        do
            case $__tue_env_i in
                --help | -h )
                    __tue_env_show_help="true"
                    break
                    ;;
                * )
                    if [[ -z "${__tue_env_tue_env}" ]]
                    then
                        __tue_env_tue_env=$__tue_env_i
                    else
                        __tue_env_args+=("$__tue_env_i")
                    fi
                    ;;
            esac
        done

        [[ -n "${__tue_env_tue_env}" ]] || __tue_env_tue_env=${TUE_ENV}
        [[ -z "${__tue_env_tue_env}" ]] && __tue_env_show_help="true" && echo "[tue-env](config) no environment set or provided"

        if [[ ${__tue_env_show_help} == "true" ]]
        then
            echo "Usage: tue-env config [ENVIRONMENT] [FUNCTION]"
            return 1
        fi

        "${TUE_DIR}"/setup/tue-env-config.bash "${__tue_env_tue_env}" "${__tue_env_args[@]}"

        if [[ "${__tue_env_tue_env}" == "${TUE_ENV}" ]]
        then
            # Assuming the current environment does exist
            local __tue_env_tue_env_dir
            __tue_env_tue_env_dir=$(cat "${TUE_DIR}"/user/envs/"${__tue_env_tue_env}")
            # shellcheck disable=SC1091
            source "${__tue_env_tue_env_dir}"/.env/setup/user_setup.bash
        fi

    elif [[ ${__tue_env_cmd} == "cd" ]]
    then
        local __tue_env_tue_env __tue_env_rel_path
        for __tue_env_i in "$@"
        do
            case $__tue_env_i in
                --help | -h )
                    __tue_env_show_help="true"
                    break
                    ;;
                --* )
                    echo "[tue-env](cd) Unknown option $__tue_env_i"
                    __tue_env_show_help="true"
                    ;;
                * )
                    if [[ -z "${__tue_env_tue_env}" ]]
                    then
                        __tue_env_tue_env=$__tue_env_i
                    else
                        __tue_env_rel_path=$__tue_env_i
                    fi
                    ;;
            esac
        done

        [[ -n "${__tue_env_tue_env}" ]] || __tue_env_tue_env=${TUE_ENV}
        [[ -z "${__tue_env_tue_env}" ]] && __tue_env_show_help="true" && echo "[tue-env](cd) no environment set or provided"

        if [[ ${__tue_env_show_help} == "true" ]]
        then
            echo "Usage: tue-env cd [ENVIRONMENT]"
            return 1
        fi

        [[ -f "${TUE_DIR}"/user/envs/"${__tue_env_tue_env}" ]] || { echo "[tue-env](cd) No such environment: '${__tue_env_tue_env}'"; return 1; }
        local __tue_env_tue_env_dir
        __tue_env_tue_env_dir=$(cat "${TUE_DIR}"/user/envs/"${__tue_env_tue_env}")

        if [[ -n "${__tue_env_rel_path}" ]]
        then
            # Validate __tue_env_rel_path to prevent directory traversal
            if [[ "${__tue_env_rel_path}" == *".."* ]] || [[ "${__tue_env_rel_path}" == /* ]]; then
                echo "[tue-env](cd) Invalid relative path: '${__tue_env_rel_path}'. Directory traversal is not allowed."
                return 1
            fi
            local __tue_env_full_path
            __tue_env_full_path="${__tue_env_tue_env_dir}/${__tue_env_rel_path}"
            cd "${__tue_env_full_path}" 2> /dev/null || { echo "[tue-env](cd) Directory '${__tue_env_rel_path}' relative to '${__tue_env_tue_env_dir}' does not exist in environment '${__tue_env_tue_env}'"; return 1; }
            return 0
        fi

        cd "${__tue_env_tue_env_dir}" 2> /dev/null || { echo "[tue-env](cd) Environment directory '${__tue_env_tue_env_dir}' (environment '${__tue_env_tue_env}') does not exist"; return 1; }

    elif [[ ${__tue_env_cmd} == "list" ]]
    then
        [ -d "$TUE_DIR"/user/envs ] || return 0

        for __tue_env_tue_env in "${TUE_DIR}"/user/envs/*
        do
            basename "${__tue_env_tue_env}"
        done

    elif [[ ${__tue_env_cmd} == "current" ]]
    then
        if [[ -n $TUE_ENV ]]
        then
            echo "$TUE_ENV"
        else
            echo "[tue-env](current) no environment set"
        fi

    else
        echo "[tue-env] Unknown command: '${__tue_env_cmd}'"
        return 1
    fi
}

export -f tue-env

# ----------------------------------------------------------------------------------------------------

function _tue-env
{
    local IFS
    IFS=$'\n'
    local cur
    cur=${COMP_WORDS[COMP_CWORD]}

    local help_options
    help_options="'-h '\n'--help '"

    if [[ "${COMP_CWORD}" -eq 1 ]]
    then
        mapfile -t COMPREPLY < <(compgen -W "$(echo -e "'init '\n'list '\n'deactivate '\n'switch '\n'current '\n'changes '\n'remove '\n'rm '\n'cd '\n'set-default '\n'unset-default '\n'config '\n'init-targets '\n'targets '\n'init-venv '\n'remove-venv '\n'rm-venv '\n${help_options}")" -- "${cur}")
    else
        local cmd
        cmd=${COMP_WORDS[1]}
        if [[ ${cmd} == "rm" ]] || [[ ${cmd} == "rm-venv" ]] || [[ ${cmd} == "cd" ]] || [[ ${cmd} == "config" ]] || [[ ${cmd} == "init-targets" ]] || [[ ${cmd} == "init-venv" ]] || [[ ${cmd} == "remove" ]] || [[ ${cmd} == "remove-venv" ]] || [[ ${cmd} == "set-default" ]] || [[ ${cmd} == "targets" ]] || [[ ${cmd} == "switch" ]]
        then
            if [[ "${COMP_CWORD}" -eq 2 ]]
            then
                [[ ! -d "${TUE_DIR}"/user/envs ]] && return 1
                mapfile -t COMPREPLY < <(compgen -W "$(echo -e "$(find "${TUE_DIR}"/user/envs -mindepth 1 -maxdepth 1 -type f -not -name ".*" -printf "%f\n" | sed "s/.*/'& '/g")")" -- "${cur}")
            elif [[ ${cmd} == "config" ]] && [[ "${COMP_CWORD}" -eq 3 ]]
            then
                local functions
                functions=$(grep 'function tue-env-' "${TUE_DIR}"/setup/tue-env-config.bash | awk '{print $2,"\n"}')
                functions=${functions//tue-env-/}
                mapfile -t COMPREPLY < <(compgen -W "$(echo -e "${functions}\n${help_options}")" -- "${cur}")
            elif [[ ${cmd} == "cd" ]] && [[ "${COMP_CWORD}" -eq 3 ]]
            then
                local tue_env
                tue_env=${COMP_WORDS[2]}
                [[ ! -f "${TUE_DIR}"/user/envs/"${tue_env}" ]] && return 1

                local tue_env_dir
                tue_env_dir=$(cat "${TUE_DIR}"/user/envs/"${tue_env}")
                [[ ! -d "${tue_env_dir}" ]] && return 1

                local full_prefix partial_path
                partial_path="${cur}"
                full_prefix="${tue_env_dir}/${partial_path}"

                # Generate completions relative to the env path
                mapfile -t COMPREPLY < <(compgen -o nospace -d -S / -- "${full_prefix}")

                # Strip the tue_env_dir prefix from suggestions to keep them relative
                for i in "${!COMPREPLY[@]}"
                do
                    COMPREPLY[i]="${COMPREPLY[$i]#${tue_env_dir}/}"
                done

                # Add help options if they match the current input
                local help_completions
                mapfile -t help_completions < <(compgen -W "$(echo -e "${help_options}")" -- "${cur}")

                # Append help completions to COMPREPLY
                COMPREPLY+=("${help_completions[@]}")
            elif [[ ${cmd} == "remove" || ${cmd} == "rm" || ${cmd} == "remove-venv" || ${cmd} == "rm-venv" ]] && [[ "${COMP_CWORD}" -eq 3 ]]
            then
                mapfile -t COMPREPLY < <(compgen -W "$(echo -e "'--purge '\n${help_options}")" -- "${cur}")
            elif [[ ${cmd} == "switch" ]] && [[ "${COMP_CWORD}" -eq 3 ]]
            then
                mapfile -t COMPREPLY < <(compgen -W "$(echo -e "'--persistent '\n${help_options}")" -- "${cur}")
            elif [[ ${cmd} == "init-venv" ]] && [[ "${COMP_CWORD}" -eq 3 ]]
            then
                mapfile -t COMPREPLY < <(compgen -W "$(echo -e "'--include-system-site-packages='\n${help_options}")" -- "${cur}")
            fi
        elif [[ ${cmd} == "deactivate" ]]
        then
            if [[ "${COMP_CWORD}" -eq 2 ]]
            then
                mapfile -t COMPREPLY < <(compgen -W "$(echo -e "'--dry-run '\n${help_options}")" -- "${cur}")
            fi
        elif [[ ${cmd} == "changes" ]]
        then
            if [[ "${COMP_CWORD}" -eq 2 ]]
            then
                mapfile -t COMPREPLY < <(compgen -W "$(echo -e "${help_options}")" -- "${cur}")
            fi
        elif [[ ${cmd} == "init" ]]
        then
            if [[ "${COMP_CWORD}" -ge 3 ]]
            then
                mapfile -t COMPREPLY < <(compgen -W "$(echo -e "'--targets-url='\n'--create-virtualenv='\n'--virtualenv-include-system-site-packages='\n${help_options}")" -- "${cur}")
            fi
        fi
    fi
}
complete -o nospace -F _tue-env tue-env
