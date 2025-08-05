#! /usr/bin/env bash

# ----------------------------------------------------------------------------------------------------
#                                        TUE-ENV IMPLEMENTATION
# ----------------------------------------------------------------------------------------------------

function _tue-env-deactivate-current-env
{
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
    local show_help
    show_help="false" # Default value
    if [[ -z "$1" ]]
    then
        show_help="true"
    else
        case $1 in
            --help | -h )
                show_help="true" ;;
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

    local cmd
    cmd=$1
    shift

    # Make sure the correct directories are there
    mkdir -p "$TUE_DIR"/user/envs

    local create_venv targets_url tue_env tue_env_dir venv_include_system_site venv_setuptools
    create_venv="true"
    venv_include_system_site="true"
    venv_setuptools="false"

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
                    --virtualenv-install-setuptools=* )
                        venv_setuptools="${i#*=}" ;;
                    --help | -h )
                        show_help="true"
                        break
                        ;;
                    --* )
                        echo "[tue-env](init) Unknown option $i"
                        show_help="true"
                        ;;
                    * )
                        if [[ -z "${tue_env}" ]]
                        then
                            tue_env="$i"
                        elif [[ -z "${tue_env_dir}" ]]
                        then
                            tue_env_dir="$i"
                        else
                            echo "[tue-env](init) Unknown input variable $i"
                            show_help="true"
                        fi
                        ;;
                esac
            done
        fi

        [[ -z "${tue_env}" ]] && show_help="true" && echo "[tue-env](init) no environment provided"

        if [[ "${show_help}" == "true" ]]
        then
            echo "Usage: tue-env init NAME [DIRECTORY] [--help|-h] [--targets-url=TARGETS_GIT_URL] [--create-virtualenv=false|TRUE] [--virtualenv-include-system-site-packages=false|TRUE] [--virtualenv-install-setuptools=FALSE|true]"
            return 1
        fi

        [[ -z "${tue_env_dir}" ]] && tue_env_dir=${PWD} # If no directory is given, use current directory
        tue_env_dir="$( realpath "${tue_env_dir}" )"

        if [ -f "${TUE_DIR}"/user/envs/"${tue_env}" ]
        then
            echo "[tue-env](init) Environment '${tue_env}' already exists"
            return 1
        fi

        if [[ -d "${tue_env_dir}"/.env ]]
        then
            echo "[tue-env](init) Directory '${tue_env_dir}' is already an environment directory."
            return 1
        fi

        echo "${tue_env_dir}" > "${TUE_DIR}"/user/envs/"${tue_env}"
        # Create .env and .env/setup directories
        mkdir -p "${tue_env_dir}"/.env/setup
        echo -e "#! /usr/bin/env bash\n" > "${tue_env_dir}"/.env/setup/user_setup.bash
        echo "[tue-env](init) Created new environment ${tue_env}"

        if [[ -n "${targets_url}" ]]
        then
            tue-env init-targets "${tue_env}" "${targets_url}" || return 1
        fi

        if [[ "${create_venv}" == "true" ]]
        then
            tue-env init-venv "${tue_env}" --include-system-site-packages="${venv_include_system_site}" --install-setuptools="${venv_setuptools}" || return 1
        fi

    elif [[ ${cmd} == "remove" || ${cmd} == "rm" ]]
    then
        # Set purge to be false by default
        local purge tue_env
        purge="false"
        if [[ -z "$1" ]]
        then
            show_help="true"
        else
            for i in "$@"
            do
                case $i in
                    --purge)
                        purge=true ;;
                    --help | -h )
                        show_help="true"
                        break
                        ;;
                    --*)
                        echo "[tue-env](rm) Unknown option $i"
                        show_help="true"
                        ;;
                    *)
                        # Read only the first passed environment name and ignore
                        # the rest
                        if [ -z "${tue_env}" ]
                        then
                            tue_env=$i
                        else
                            echo "[tue-env](rm) Unknown input variable $i"
                            show_help="true"
                        fi
                        ;;
                esac
            done
        fi

        [[ -z "${tue_env}" ]]  && show_help="true" && echo "[tue-env](rm) no environment provided"

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

        [[ -f "${TUE_DIR}"/user/envs/"${tue_env}" ]] || { echo "[tue-env](rm) No such environment: '${tue_env}'"; return 1; }

        if [[ "${tue_env}" == "${TUE_ENV}" ]]
        then
            echo "[tue-env](rm) The environment '${tue_env}' is currently active. Deactivating it first."
            _tue-env-deactivate-current-env || { echo "[tue-env](rm) Failed to deactivate the current environment, don't use this terminal anymore, open a new terminal"; return 1; }
        fi

        # Unset the default environment if it is the one being removed
        if [[ -f "${TUE_DIR}"/user/config/default_env ]] && [[ "$(cat "${TUE_DIR}"/user/config/default_env)" == "${tue_env}" ]]
        then
            echo "[tue-env](rm) Unsetting the default environment '${tue_env}'"
            tue-env unset-default || return 1
        fi

        local tue_env_dir
        tue_env_dir=$(cat "${TUE_DIR}"/user/envs/"${tue_env}")
        rm "${TUE_DIR}"/user/envs/"${tue_env}"

        if [[ -d ${tue_env_dir} ]]
        then
            if [[ ${purge} == "false" ]]
            then
                local tue_env_dir_moved
                tue_env_dir_moved=${tue_env_dir}.$(date +%F_%R)
                mv "${tue_env_dir}" "${tue_env_dir_moved}"
                # shellcheck disable=SC1078,SC1079
                echo """[tue-env] Removed environment '${tue_env}'
Moved environment directory from '${tue_env_dir}' to '${tue_env_dir_moved}'"""
            else
                rm -rf "${tue_env_dir}"
                # shellcheck disable=SC1078,SC1079
                echo """[tue-env] Removed environment '${tue_env}'
Purged environment directory '${tue_env_dir}'"""
            fi
        else
            # shellcheck disable=SC1078,SC1079
            echo """[tue-env] Removed environment '${tue_env}'
Environment directory '${tue_env_dir}' didn't exist (anymore)"""
        fi

    elif [[ ${cmd} == "deactivate" ]]
    then
        for i in "$@"
        do
            case $i in
                --help | -h )
                    show_help="true"
                    break
                    ;;
                --*)
                    echo "[tue-env](deactivate) Unknown option $i"
                    show_help="true"
                    ;;
                * )
                    echo "[tue-env](deactivate) Unknown input variable $i"
                    show_help="true"
                    ;;
            esac
        done

        if [[ ${show_help} == "true" ]]
        then
            # shellcheck disable=SC1078,SC1079
            echo """Usage: tue-env deactivate [options]

    Possible options:
                --help, -h     - Show this help message and exit
"""
            return 1
        fi

        if [[ -z "${TUE_ENV}" ]]
        then
            echo "[tue-env](deactivate) No environment is currently active"
            return 1
        fi

        echo "[tue-env](deactivate) Deactivating the current environment '${TUE_ENV}'"
        _tue-env-deactivate-current-env || { echo "[tue-env](deactivate) Failed to deactivate the current environment, don't use this terminal anymore, open a new terminal"; return 1; }

        return 0

    elif [[ ${cmd} == "switch" ]]
    then
        local persistent tue_env
        persistent="false"
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
                    --persistent )
                        persistent="true" ;;
                    --*)
                        echo "[tue-env](switch) Unknown option $i"
                        show_help="true"
                        ;;
                    * )
                        if [[ -z "${tue_env}" ]]
                        then
                            tue_env=$i
                        else
                            echo "[tue-env](switch) Unknown input variable $i"
                            show_help="true"
                        fi
                        ;;
                esac
            done
        fi

        [[ -z "${tue_env}" ]] && show_help="true" && echo "[tue-env](switch) no environment provided"

        if [[ ${show_help} == "true" ]]
        then
            # shellcheck disable=SC1078,SC1079
            echo """Usage: tue-env switch [options] ENVIRONMENT

    Possible options:
        --persistent   - Set the environment as default
        --help, -h     - Show this help message and exit
"""
            return 1
        fi

        [[ -f "${TUE_DIR}"/user/envs/"${tue_env}" ]] || { echo "[tue-env](switch) No such environment: '${tue_env}'"; return 1; }
        local tue_env_dir
        tue_env_dir=$(cat "${TUE_DIR}"/user/envs/"${tue_env}")
        [[ -d "${tue_env_dir}" ]] || { echo "[tue-env](switch) Environment directory '${tue_env_dir}' (environment '${tue_env}') does not exist"; return 1; }

        [[ "${persistent}" == "true" ]] && { tue-env set-default "${tue_env}" || return 1; }

        if [[ -n "${TUE_ENV}" ]]
        then
            if [[ "${TUE_ENV}" == "${tue_env}" ]]
            then
                echo "[tue-env](switch) Already in the '${tue_env}' environment"
                return 0
            fi
            echo "[tue-env](switch) Deactivating the current environment '${TUE_ENV}'"
            _tue-env-deactivate-current-env || { echo "[tue-env](switch) Failed to deactivate the current environment, don't use this terminal anymore, open a new terminal"; return 1; }
        fi

        # Successful, so we can set the environment variables
        TUE_ENV=${tue_env}
        export TUE_ENV
        TUE_ENV_DIR=${tue_env_dir}
        export TUE_ENV_DIR

        echo "[tue-env](switch) Loading the new '${TUE_ENV}' environment"
        # shellcheck disable=SC1091
        source "$TUE_DIR"/setup.bash

    elif [[ ${cmd} == "set-default" ]]
    then
        local tue_env
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
                    --*)
                        echo "[tue-env](set-default) Unknown option $i"
                        show_help="true"
                        ;;
                    * )
                        if [[ -z "${tue_env}" ]]
                        then
                            tue_env=$i
                        else
                            echo "[tue-env](set-default) Unknown input variable $i"
                            show_help="true"
                        fi
                        ;;
                esac
            done
        fi

        [[ -z "${tue_env}" ]] && show_help="true" && echo "[tue-env](set-default) no environment provided"

        if [[ ${show_help} == "true" ]]
        then
            echo "Usage: tue-env set-default ENVIRONMENT"
            return 1
        fi

        mkdir -p "${TUE_DIR}"/user/config
        echo "${tue_env}" > "${TUE_DIR}"/user/config/default_env
        echo "[tue-env](set-default) Default environment set to '${tue_env}'"

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
            echo "[tue-env](unset-default) No default environment set, nothing to unset"
            return 1
        fi
        local default_env
        default_env=$(cat "${TUE_DIR}"/user/config/default_env)
        rm -f "${TUE_DIR}"/user/config/default_env
        echo "[tue-env](unset-default) Default environment '${default_env}' unset"
        return 0

    elif [[ ${cmd} == "init-targets" ]]
    then
        local tue_env url
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
                    --* )
                        echo "[tue-env](init-targets) Unknown option $i"
                        show_help="true"
                        ;;
                    * )
                        if [[ -z "${tue_env}" ]]
                        then
                            tue_env=$i
                        elif [[ -z "${url}" ]]
                        then
                            url=$i
                        else
                            echo "[tue-env](init-targets) Unknown input variable $i"
                            show_help="true"
                        fi
                        ;;
                esac
            done
        fi

        if [[ -z "${url}" ]]
        then
            url=${tue_env} # If no environment was given, the url was assigned to tue_env
            tue_env=${TUE_ENV}
        fi

        [[ -z "${tue_env}" ]] && show_help="true" && echo "[tue-env](init-targets) no environment set or provided"

        if [[ ${show_help} == "true" ]]
        then
            echo "Usage: tue-env init-targets [ENVIRONMENT] TARGETS_GIT_URL"
            return 1
        fi

        [[ -f "${TUE_DIR}"/user/envs/"${tue_env}" ]] || { echo "[tue-env](init-targets) No such environment: '${tue_env}'"; return 1; }
        local tue_env_dir
        tue_env_dir=$(cat "${TUE_DIR}"/user/envs/"${tue_env}")
        [[ -d "${tue_env_dir}" ]] || { echo "[tue-env](init-targets) Environment directory '${tue_env_dir}' (environment '${tue_env}') does not exist"; return 1; }

        local tue_env_targets_dir
        tue_env_targets_dir=$tue_env_dir/.env/targets

        if [ -d "$tue_env_targets_dir" ]
        then
            local targets_dir_moved
            targets_dir_moved=$tue_env_targets_dir.$(date +%F_%R)
            mv -f "$tue_env_targets_dir" "$targets_dir_moved"
            echo "[tue-env] Moved old targets of environment '${tue_env}' to ${targets_dir_moved}"
        fi

        git clone --recursive "$url" "$tue_env_targets_dir"
        echo "[tue-env] cloned targets of environment '${tue_env}' from ${url}"

    elif [[ ${cmd} == "targets" ]]
    then
        local tue_env
        for i in "$@"
        do
            case $i in
                --help | -h )
                    show_help="true"
                    break
                    ;;
                --* )
                    echo "[tue-env](targets) Unknown option $i"
                    show_help="true"
                    ;;
                * )
                    if [[ -z "${tue_env}" ]]
                    then
                        tue_env=$i
                    else
                        echo "[tue-env](targets) Unknown input variable $i"
                        show_help="true"
                    fi
                    ;;
            esac
        done

        [[ -n "${tue_env}" ]] || tue_env=${TUE_ENV}
        [[ -z "${tue_env}" ]] && show_help="true" && echo "[tue-env](targets) no environment set or provided"

        if [[ ${show_help} == "true" ]]
        then
            echo "Usage: tue-env targets [ENVIRONMENT]"
            return 1
        fi

        [[ -f "${TUE_DIR}"/user/envs/"${tue_env}" ]] || { echo "[tue-env](targets) No such environment: '${tue_env}'"; return 1; }
        local tue_env_dir
        tue_env_dir=$(cat "${TUE_DIR}"/user/envs/"${tue_env}")
        [[ -d "${tue_env_dir}" ]] || { echo "[tue-env](targets) Environment directory '${tue_env_dir}' (environment '${tue_env}') does not exist"; return 1; }
        cd "${tue_env_dir}"/.env/targets || { echo -e "Targets directory '${tue_env_dir}/.env/targets' (environment '${tue_env}') does not exist"; return 1; }

    elif [[ ${cmd} == "init-venv" ]]
    then
        local include_system_site install_setuptools tue_env
        include_system_site="true"
        install_setuptools="false"
        for i in "$@"
        do
            case $i in
                --help | -h )
                    show_help="true"
                    break
                    ;;
                --include-system-site-packages=* )
                    include_system_site="${i#*=}" ;;
                --install-setuptools=* )
                    install_setuptools="${i#*=}" ;;
                --* )
                    echo "[tue-env] Unknown option $i"
                    show_help="true"
                    ;;
                * )
                    if [[ -z "${tue_env}" ]]
                    then
                        tue_env=$i
                    else
                        echo "[tue-env] Unknown input variable $i"
                        show_help="true"
                    fi
                    ;;
            esac
        done

        [[ -n "${tue_env}" ]] || tue_env=${TUE_ENV}
        [[ -z "${tue_env}" ]] && show_help="true" && echo "[tue-env](init-venv) no environment set or provided"

        if [[ ${show_help} == "true" ]]
        then
            echo "Usage: tue-env init-venv [ENVIRONMENT] [--include-system-site-packages=false|TRUE] [--install-setuptools=FALSE|true]"
            return 1
        fi

        local installed_version parsed_version pkg_name version_requirement
        pkg_name="virtualenv"
        version_requirement=">=20.24.0"
        installed_version=$(/usr/bin/python3 -c "import importlib.metadata; print(importlib.metadata.version('${pkg_name}'))" 2>/dev/null)
        parsed_version=$(echo "${installed_version}" | grep -oE '[0-9]+\.[0-9]+(\.[0-9]+)?' | head -n1)
        if ! _tue-env-compare-version "${parsed_version}" "${version_requirement}"
        then
            /usr/bin/python3 -Ic "import ${pkg_name}" && \
            { echo -e "[tue-env](init-venv) '${pkg_name}(${installed_version})' does not match the required version '${version_requirement}' and is installed via APT." \
            "To prevent any conflicts, first uninstall it: \"sudo apt-get remove python3-${pkg_name}\"" \
            "Make sure you install it \"/usr/bin/python3 -m pip install --user '${pkg_name}${version_requirement}'\""; return 1; }

            { echo -e "[tue-env](init-venv) '${pkg_name}(${installed_version})' doesn't match the required version '${version_requirement}'. " \
            "Make sure you install it \"/usr/bin/python3 -m pip install --user '${pkg_name}${version_requirement}'\""; return 1; }
        fi

        [[ -f "${TUE_DIR}"/user/envs/"${tue_env}" ]] || { echo "[tue-env](init-venv) No such environment: '${tue_env}'"; return 1; }
        local tue_env_dir
        tue_env_dir=$(cat "${TUE_DIR}"/user/envs/"${tue_env}")
        [[ -d "${tue_env_dir}" ]] || { echo "[tue-env](init-venv) Environment directory '${tue_env_dir}' (environment '${tue_env}') does not exist"; return 1; }
        local venv_dir venv_dir_deprecated
        venv_dir=${tue_env_dir}/.env/venv
        venv_dir_deprecated=${tue_env_dir}/.venv/"${tue_env}"

        if [[ -d "${venv_dir}" ]]
        then
            local venv_dir_moved
            venv_dir_moved=${venv_dir}.$(date +%F_%R)
            if [[ "${VIRTUAL_ENV_PROMPT}" == "${tue_env}" ]]
            then
                echo "[tue-env](init-venv) deactivating currently active virtualenv of environment '${tue_env}'"
                deactivate
            fi
            mv -f "${venv_dir}" "${venv_dir_moved}"
            echo "[tue-env](init-venv) Moved old virtualenv of environment '${tue_env}' to ${venv_dir_moved}"
            echo "Don't use it anymore as its old path is hardcoded in the virtualenv"
        fi
        if [[ -d "${venv_dir_deprecated}" ]]
        then
            local venv_dir_deprecated_moved
            venv_dir_deprecated_moved=${venv_dir_deprecated}.$(date +%F_%R)
            if [[ $(basename "${VIRTUAL_ENV}") == "${tue_env}" ]]
            then
                echo "[tue-env](init-venv) deactivating currently active virtualenv of environment '${tue_env}'"
                deactivate
            fi
            mv -f "${venv_dir_deprecated}" "${venv_dir_deprecated_moved}"
            echo "[tue-env](init-venv) Moved old virtualenv of environment '${tue_env}' to ${venv_dir_deprecated_moved}"
            echo "Don't use it anymore as its old path is hardcoded in the virtualenv"
        fi

        local system_site_args
        if [[ "${include_system_site}" == "true" ]]
        then
            system_site_args="--system-site-packages"
        fi
        local setuptools_args
        if [[ "${install_setuptools}" != "true" ]]
        then
            setuptools_args="--no-setuptools"
        fi
        /usr/bin/python3 -m virtualenv "${venv_dir}" ${system_site_args:+${system_site_args} }${setuptools_args:+${setuptools_args} }--symlinks --prompt "${tue_env}" -q 2>/dev/null ||
        { echo "[tue-env](init-venv) Failed to initialize virtual environment '${venv_dir}' for environment '${tue_env}'"; return 1; }

        echo "[tue-env](init-venv) Initialized virtualenv of environment '${tue_env}'"

        if [ "${tue_env}" == "${TUE_ENV}" ]
        then
            # No need to check if the environment really exists, as it was checked before
            local tue_env_dir
            tue_env_dir=$(cat "${TUE_DIR}"/user/envs/"${tue_env}")
            # shellcheck disable=SC1091
            source "${tue_env_dir}"/.env/venv/bin/activate
            echo "[tue-env](init-venv) Activated new virtualenv of currently active environment '${tue_env}'"
        fi

    elif [[ ${cmd} == "remove-venv" || ${cmd} == "rm-venv" ]]
    then
        local purge tue_env
        purge="false"
        for i in "$@"
        do
            case $i in
                --help | -h )
                    show_help="true"
                    break
                    ;;
                --purge)
                    purge="true" ;;
                --* )
                    echo "[tue-env](rm-venv) Unknown option $i"
                    show_help="true"
                    ;;
                * )
                    if [[ -z "${tue_env}" ]]
                    then
                        tue_env=$i
                    else
                        echo "[tue-env](rm-venv) Unknown input variable $i"
                        show_help="true"
                    fi
                    ;;
            esac
        done

        [[ -n "${tue_env}" ]] || tue_env=${TUE_ENV}
        [[ -z "${tue_env}" ]] && show_help="true" && echo "[tue-env](rm-venv) no environment set or provided"

        if [[ ${show_help} == "true" ]]
        then
            echo "Usage: tue-env remove-venv [ENVIRONMENT]"
            return 1
        fi

        [[ -f "${TUE_DIR}"/user/envs/"${tue_env}" ]] || { echo "[tue-env](rm-venv) No such environment: '${tue_env}'"; return 1; }
        local tue_env_dir
        tue_env_dir=$(cat "${TUE_DIR}"/user/envs/"${tue_env}")
        [[ -d "${tue_env_dir}" ]] || { echo "[tue-env](rm-venv) Environment directory '${tue_env_dir}' (environment '${tue_env}') does not exist"; return 1; }
        local venv_dir venv_dir_deprecated
        venv_dir=${tue_env_dir}/.env/venv
        venv_dir_deprecated=${tue_env_dir}/.venv/"${tue_env}"


        if [[ -d "${venv_dir}" ]]
        then
            if [[ "${VIRTUAL_ENV_PROMPT}" == "${tue_env}" ]]
            then
                echo "[tue-env](rm-venv) deactivating currently active virtualenv of environment '${tue_env}'"
                deactivate
            fi
            if [[ "${purge}" == "true" ]]
            then
                rm -rf "${venv_dir}"
                echo "[tue-env](rm-venv) Purged virtualenv of environment '${tue_env}'"
                return 0
            else
                local venv_dir_moved
                venv_dir_moved=${venv_dir}.$(date +%F_%R)
                mv -f "${venv_dir}" "${venv_dir_moved}"
                echo "[tue-env](rm-venv) Moved old virtualenv of environment '${tue_env}' to ${venv_dir_moved}"
            fi
        elif [[ -d "${venv_dir_deprecated}" ]]
        then
          if [[ $(basename "${VIRTUAL_ENV}") == "${tue_env}" ]]
          then
                echo "[tue-env](rm-venv) deactivating currently active virtualenv of environment '${tue_env}'"
                deactivate
            fi
            if [[ "${purge}" == "true" ]]
            then
                rm -rf "${venv_dir}"
                echo "[tue-env](rm-venv) Purged virtualenv of environment '${tue_env}'"
                return 0
            else
                local venv_dir_deprecated_moved
                venv_dir_deprecated_moved=${venv_dir_deprecated}.$(date +%F_%R)
                mv -f "${venv_dir_deprecated}" "${venv_dir_deprecated_moved}"
                echo "[tue-env](rm-venv) Moved old virtualenv of environment '${tue_env}' to ${venv_dir_deprecated_moved}"
            fi
        else
            echo "[tue-env](rm-venv) No virtualenv found for environment '${tue_env}'"
        fi

    elif [[ ${cmd} == "config" ]]
    then
        local tue_env args
        args=()
        for i in "$@"
        do
            case $i in
                --help | -h )
                    show_help="true"
                    break
                    ;;
                * )
                    if [[ -z "${tue_env}" ]]
                    then
                        tue_env=$i
                    else
                        args+=("$i")
                    fi
                    ;;
            esac
        done

        [[ -n "${tue_env}" ]] || tue_env=${TUE_ENV}
        [[ -z "${tue_env}" ]] && show_help="true" && echo "[tue-env](config) no environment set or provided"

        if [[ ${show_help} == "true" ]]
        then
            echo "Usage: tue-env config [ENVIRONMENT] [FUNCTION]"
            return 1
        fi

        "${TUE_DIR}"/setup/tue-env-config.bash "${tue_env}" "${args[@]}"

        if [[ "${tue_env}" == "${TUE_ENV}" ]]
        then
            # Assuming the current environment does exist
            local tue_env_dir
            tue_env_dir=$(cat "${TUE_DIR}"/user/envs/"${tue_env}")
            # shellcheck disable=SC1091
            source "${tue_env_dir}"/.env/setup/user_setup.bash
        fi

    elif [[ ${cmd} == "cd" ]]
    then
        local tue_env rel_path
        for i in "$@"
        do
            case $i in
                --help | -h )
                    show_help="true"
                    break
                    ;;
                --* )
                    echo "[tue-env](cd) Unknown option $i"
                    show_help="true"
                    ;;
                * )
                    if [[ -z "${tue_env}" ]]
                    then
                        tue_env=$i
                    else
                        rel_path=$i
                    fi
                    ;;
            esac
        done

        [[ -n "${tue_env}" ]] || tue_env=${TUE_ENV}
        [[ -z "${tue_env}" ]] && show_help="true" && echo "[tue-env](cd) no environment set or provided"

        if [[ ${show_help} == "true" ]]
        then
            echo "Usage: tue-env cd [ENVIRONMENT]"
            return 1
        fi

        [[ -f "${TUE_DIR}"/user/envs/"${tue_env}" ]] || { echo "[tue-env](cd) No such environment: '${tue_env}'"; return 1; }
        local tue_env_dir
        tue_env_dir=$(cat "${TUE_DIR}"/user/envs/"${tue_env}")

        if [[ -n "${rel_path}" ]]
        then
            local full_path
            full_path="${tue_env_dir}/${rel_path}"
            cd "${full_path}" 2> /dev/null || { echo "[tue-env](cd) Directory '${rel_path}' relative to '${tue_env_dir}' does not exist in environment '${tue_env}'"; return 1; }
            return 0
        fi

        cd "${tue_env_dir}" 2> /dev/null || { echo "[tue-env](cd) Environment directory '${tue_env_dir}' (environment '${tue_env}') does not exist"; return 1; }

    elif [[ ${cmd} == "list" ]]
    then
        [ -d "$TUE_DIR"/user/envs ] || return 0

        for tue_env in "${TUE_DIR}"/user/envs/*
        do
            basename "${tue_env}"
        done

    elif [[ ${cmd} == "current" ]]
    then
        if [[ -n $TUE_ENV ]]
        then
            echo "$TUE_ENV"
        else
            echo "[tue-env](current) no environment set"
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
    local IFS
    IFS=$'\n'
    local cur
    cur=${COMP_WORDS[COMP_CWORD]}

    local help_options
    help_options="'-h '\n'--help '"

    if [[ "${COMP_CWORD}" -eq 1 ]]
    then
        mapfile -t COMPREPLY < <(compgen -W "$(echo -e "'init '\n'list '\n'deactivate '\n'switch '\n'current '\n'remove '\n'rm '\n'cd '\n'set-default '\n'unset-default '\n'config '\n'init-targets '\n'targets '\n'init-venv '\n'remove-venv '\n'rm-venv '\n${help_options}")" -- "${cur}")
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
