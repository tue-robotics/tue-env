#! /usr/bin/env bash

# -----------------------------------------
# Fix annoying perl language warnings
export LC_ALL="C.UTF-8"

# ------------------------------------------
# pip bash completion
if hash pip 2> /dev/null || hash pip2 2> /dev/null || hash pip3 2> /dev/null
then
    _pip_completion()
    {
        # shellcheck disable=SC2207
        COMPREPLY=( $( COMP_WORDS="${COMP_WORDS[*]}" \
                       COMP_CWORD=$COMP_CWORD \
                       PIP_AUTO_COMPLETE=1 $1 2>/dev/null ) )
    }
    complete -o default -F _pip_completion pip
fi

hash pip 2> /dev/null && complete -o default -F _pip_completion pip
hash pip2 2> /dev/null && complete -o default -F _pip_completion pip2
hash pip3 2> /dev/null && complete -o default -F _pip_completion pip3
