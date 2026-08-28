#! /usr/bin/env bash

# -----------------------------------------
# Fix annoying perl language warnings
export LC_ALL="C.UTF-8"

# ------------------------------------------
# pip bash completion start
if hash pip3 2> /dev/null
then
    _pip_completion()
    {
        local IFS=$' \t\n'
        # shellcheck disable=SC2207
        COMPREPLY=( $( COMP_WORDS="${COMP_WORDS[*]}" \
                       COMP_CWORD=$COMP_CWORD \
                       PIP_AUTO_COMPLETE=1 $1 2>/dev/null ) )
    }
    complete -o default -F _pip_completion pip3
    # `pip` and `pip3` have been the same program since python2 went away; this used to run
    # `pip -V` and `pip3 -V` to tell them apart, which started two Python interpreters and cost
    # 0.27 s of every interactive shell - 40% of the whole environment load.
    complete -o default -F _pip_completion pip
fi
# pip bash completion end
