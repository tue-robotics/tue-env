#! /usr/bin/env bash
TUE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
export TUE_DIR

# Load tue-env tool
# shellcheck disable=SC1090
ext=
hash register-python-argcomplete3 2> /dev/null && ext="3"
eval "$(register-python-argcomplete${ext} tue-env)"

# ------------------------------------------
# Helper function for checking if all env vars are set
function _tue-check-env-vars
{
    [ -n "$TUE_DIR" ] && [ -n "$TUE_ENV" ] && [ -n "$TUE_ENV_DIR" ] \
        && [ -n "$TUE_ENV_TARGETS_DIR" ] && return 0
    echo "[tue] Not all needed environment variables are set."
    return 1
}
export -f _tue-check-env-vars

if [ -z "$TUE_ENV" ]
then
    TUE_ENV=$(tue-env name)
    if [[ -z "${TUE_ENV}" ]]; then
        return 0
    fi
    export TUE_ENV
fi

TUE_ENV_DIR=$(tue-env locate)
export TUE_ENV_DIR
if [ -f "$TUE_ENV_DIR"/.env/setup/user_setup.bash ]
then
    # shellcheck disable=SC1090
    source "$TUE_ENV_DIR"/.env/setup/user_setup.bash
fi

TUE_ENV_TARGETS_DIR="$(tue-env targets-dir)"
export TUE_ENV_TARGETS_DIR

function _tue-env-targets
{
    [[ -n ${TUE_ENV_TARGETS_DIR} ]] && cd ${TUE_ENV_TARGETS_DIR}
}
alias tue-env-targets="_tue-env-targets"

# -----------------------------------------
# Load all the bash functions
# shellcheck disable=SC1090
source "$TUE_DIR"/setup/tue-functions.bash

if [ -f "$TUE_DIR"/setup/tue-misc.bash ]
then
    # shellcheck disable=SC1090
    source "$TUE_DIR"/setup/tue-misc.bash
fi

# .local/bin is needed in the path for all user installs like pip. It gets added automatically on reboot but not on CI
if [[ :$PATH: != *:$HOME/.local/bin:* ]]
then
    export PATH=$HOME/.local/bin${PATH:+:${PATH}}
fi

if [ -f "$TUE_ENV_DIR"/.env/setup/target_setup.bash ]
then
    # shellcheck disable=SC1090
    source "$TUE_ENV_DIR"/.env/setup/target_setup.bash
fi
