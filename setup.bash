#! /usr/bin/env bash

# ------------------------------------------
# Helper function for checking if all env vars are set
function _tue-check-env-vars
{
    [[ -n "${TUE_DIR}" ]] && [[ -n "${TUE_ENV}" ]] && [[ -n "${TUE_ENV_DIR}" ]] \
       && [[ -n "${TUE_BIN}" ]] && [[ -n "${TUE_ENV_TARGETS_DIR}" ]] && return 0
    echo "[tue] Not all needed environment variables are set."
    return 1
}
export -f _tue-check-env-vars

function _tue-env-bootstrap
{
    # Everything here runs before the tracked span begins, so it survives `tue-env deactivate`.

    # -----------------------------------------
    # Set the TUE_DIR variable
    TUE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
    export TUE_DIR

    # -----------------------------------------
    # Add `.local/bin` and TUE_DIR to PATH
    export TUE_BIN=${TUE_DIR}/bin
    # .local/bin is needed in the path for all user installs like pip. It gets added automatically on reboot but not on CI
    if [[ :${PATH}: != *:${HOME}/.local/bin:* ]]
    then
        export PATH=${HOME}/.local/bin${PATH:+:${PATH}}
    fi

    if [[ :${PATH}: != *:${TUE_BIN}:* ]]
    then
        export PATH=${TUE_BIN}${PATH:+:${PATH}}
    fi

    # -----------------------------------------
    # Load tue-env tool
    # shellcheck disable=SC1091
    source "${TUE_DIR}"/setup/tue-env.bash

    # -----------------------------------------
    # Load the change tracker, so that it exists before the tracked span begins
    # shellcheck disable=SC1091
    source "${TUE_DIR}"/setup/tue-env-track.bash
}

function _tue-env-load
{
    # Load the (optional) default environment
    if [[ -z "${TUE_ENV}" ]]
    then
        [[ -f "${TUE_DIR}"/user/config/default_env ]] || return 0  # Quietly return

        TUE_ENV=$(cat "${TUE_DIR}"/user/config/default_env)
        export TUE_ENV

        if [[ ! -f "${TUE_DIR}"/user/envs/"${TUE_ENV}" ]]
        then
            echo "[tue] No such environment: '${TUE_ENV}'"
            return 1
        fi
    fi

    # -----------------------------------------
    # Set the TUE_ENV_DIR and TUE_ENV_TARGETS_DIR variables
    TUE_ENV_DIR=$(cat "${TUE_DIR}"/user/envs/"${TUE_ENV}")
    export TUE_ENV_DIR

    if [[ ! -d "${TUE_ENV_DIR}" ]]
    then
        echo "[tue] Environment directory '${TUE_ENV_DIR}' (environment '${TUE_ENV}') does not exist"
        return 1
    fi

    export TUE_ENV_TARGETS_DIR=${TUE_ENV_DIR}/.env/targets

    if [[ ! -d "${TUE_ENV_TARGETS_DIR}" ]]
    then
        echo "[tue] Targets directory '${TUE_ENV_TARGETS_DIR}' (environment '${TUE_ENV}') does not exist"
        return 1
    fi

    # -----------------------------------------
    # Load the user setup file
    if [[ -f "${TUE_ENV_DIR}"/.env/setup/user_setup.bash ]]
    then
        # shellcheck disable=SC1091
        source "${TUE_ENV_DIR}"/.env/setup/user_setup.bash
    fi

    # -----------------------------------------
    # Load the python virtual environment if it exists
    if [[ -d "${TUE_ENV_DIR}"/.env/venv/ ]]
    then
        # shellcheck disable=SC1091
        source "${TUE_ENV_DIR}"/.env/venv/bin/activate
    elif [[ -d "${TUE_ENV_DIR}"/.venv/"${TUE_ENV}" ]]
    then
        echo -e "\e[33;1m[tue] virtual environment location '${TUE_ENV_DIR}/.venv/${TUE_ENV}' is deprecated. Please create a new environment using 'tue-env init-venv ${TUE_ENV}'.\e[0m"
        # shellcheck disable=SC1090
        source "${TUE_ENV_DIR}"/.venv/"${TUE_ENV}"/bin/activate
    fi

    # -----------------------------------------
    # Load all the bash functions
    # shellcheck disable=SC1091
    source "${TUE_DIR}"/setup/tue-functions.bash

    if [[ -f "${TUE_DIR}"/setup/tue-misc.bash ]]
    then
        # shellcheck disable=SC1091
        source "${TUE_DIR}"/setup/tue-misc.bash
    fi

    # -----------------------------------------
    # Load the target setup files
    # The target setup files could depend on anything that was loaded above
    if [[ -f "${TUE_ENV_DIR}"/.env/setup/target_setup.bash ]]
    then
        # shellcheck disable=SC1091
        source "${TUE_ENV_DIR}"/.env/setup/target_setup.bash
    fi
}

function _tue-env-main
{
    _tue-env-bootstrap

    # The wrapper is required, not cosmetic: _tue-env-load returns early on a missing environment, a
    # missing TUE_ENV_DIR and a missing targets directory, all after TUE_ENV has been exported.
    # Without committing on those paths a failed load would leave TUE_ENV* set and untracked.
    # Collecting the status through `||` also keeps a caller's `set -e` from skipping the commit, and
    # the local is prefixed so that the commit snapshot does not record it as an added variable.
    local __tue_env_ret=0
    _tue-env-track-begin
    _tue-env-load "$@" || __tue_env_ret=$?
    _tue-env-track-commit
    return "${__tue_env_ret}"
}

_tue-env-main "$@"

unset -f _tue-env-bootstrap _tue-env-load _tue-env-main
