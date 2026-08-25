#! /usr/bin/env bash

# -----------------------------------------
# Fix annoying perl language warnings
export LC_ALL="C.UTF-8"

# ------------------------------------------
# pip bash completion
if hash pip3 2> /dev/null
then
    _pip_completion()
    {
        # shellcheck disable=SC2207
        COMPREPLY=( $( COMP_WORDS="${COMP_WORDS[*]}" \
                       COMP_CWORD=$COMP_CWORD \
                       PIP_AUTO_COMPLETE=1 $1 2>/dev/null ) )
    }
    complete -o default -F _pip_completion pip3
    # `pip -V` and `pip3 -V` each start a Python interpreter, which measured 0.27 s of every
    # interactive shell on the machine this was written on - 40% of the whole environment load -
    # only to decide whether the two names are the same program. The scripts themselves answer
    # that, and builtins can read them without a fork.
    function __tue_env_pip_is_pip3
    {
        local __tue_env_pip __tue_env_pip3
        __tue_env_pip="$(command -v pip)" || return 1
        __tue_env_pip3="$(command -v pip3)" || return 1
        local -a __tue_env_pip_a __tue_env_pip_b
        mapfile -t __tue_env_pip_a < "${__tue_env_pip}" || return 1
        mapfile -t __tue_env_pip_b < "${__tue_env_pip3}" || return 1
        [[ "${__tue_env_pip_a[*]}" == "${__tue_env_pip_b[*]}" ]] || return 1
        return 0
    }
    if __tue_env_pip_is_pip3
    then
        complete -o default -F _pip_completion pip
    fi
    # Every local here is __tue_env_* prefixed and the helper is unset again, so nothing of this
    # reaches the environment change ledger or outlives the load.
    unset -f __tue_env_pip_is_pip3
fi
