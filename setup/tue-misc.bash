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
    if [ "$(pip -V)" == "$(pip3 -V)" ]
    then
        complete -o default -F _pip_completion pip
    fi
fi
