#! /usr/bin/env bash
TUE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
export TUE_DIR

# Load tue-env tool
# shellcheck disable=SC1091
source "$TUE_DIR"/setup/tue-env.bash

# ------------------------------------------
# Helper function for checking if all env vars are set
function _tue-check-env-vars
{
    [ -n "$TUE_DIR" ] && [ -n "$TUE_ENV" ] && [ -n "$TUE_ENV_DIR" ] \
       && [ -n "$TUE_BIN" ] && [ -n "$TUE_ENV_TARGETS_DIR" ] && return 0
    echo "[tue] Not all needed environment variables are set."
    return 1
}
export -f _tue-check-env-vars

if [ -z "$TUE_ENV" ]
then
    if [ ! -f "$TUE_DIR"/user/config/default_env ]
    then
        # No environment, so all environment specific setup below does not need to be sourced
        return 0
    fi

    TUE_ENV=$(cat "$TUE_DIR"/user/config/default_env)
    export TUE_ENV

    if [ ! -f "$TUE_DIR"/user/envs/"$TUE_ENV" ]
    then
        echo "[tue] No such environment: '$TUE_ENV'"
        return 1
    fi
fi

TUE_ENV_DIR=$(cat "$TUE_DIR"/user/envs/"$TUE_ENV")
export TUE_ENV_DIR

if [ ! -d "$TUE_ENV_DIR" ]
then
    echo "[tue] Environment directory '$TUE_ENV_DIR' (environment '$TUE_ENV') does not exist"
    return 1
fi

export TUE_ENV_TARGETS_DIR=$TUE_ENV_DIR/.env/targets

if [ ! -d "$TUE_ENV_TARGETS_DIR" ]
then
    echo "[tue] Targets directory '$TUE_ENV_TARGETS_DIR' (environment '$TUE_ENV') does not exist"
    return 1
fi

if [ -f "$TUE_ENV_DIR"/.env/setup/user_setup.bash ]
then
    # shellcheck disable=SC1091
    source "$TUE_ENV_DIR"/.env/setup/user_setup.bash
fi

# -----------------------------------------
# Load all the bash functions
# shellcheck disable=SC1091
source "$TUE_DIR"/setup/tue-functions.bash

if [ -f "$TUE_DIR"/setup/tue-misc.bash ]
then
    # shellcheck disable=SC1091
    source "$TUE_DIR"/setup/tue-misc.bash
fi

export TUE_BIN=$TUE_DIR/bin

# .local/bin is needed in the path for all user installs like pip. It gets added automatically on reboot but not on CI
if [[ :$PATH: != *:$HOME/.local/bin:* ]]
then
    export PATH=$HOME/.local/bin${PATH:+:${PATH}}
fi

if [[ :$PATH: != *:$TUE_BIN:* ]]
then
    export PATH=$TUE_BIN${PATH:+:${PATH}}
fi

# Source the python virtual environment if it exists
if [[ -d "${TUE_ENV_DIR}"/.venv/"${TUE_ENV}" ]]
then
    # shellcheck disable=SC1090
    source "${TUE_ENV_DIR}"/.venv/"${TUE_ENV}"/bin/activate
fi

if [ -f "$TUE_ENV_DIR"/.env/setup/target_setup.bash ]
then
    # shellcheck disable=SC1091
    source "$TUE_ENV_DIR"/.env/setup/target_setup.bash
fi
