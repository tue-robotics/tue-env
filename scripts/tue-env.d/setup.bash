#! /usr/bin/env bash

# This needed because without the dot, the script is run in a child process.
# But we don't want to run it with a dot all the time.
function tue-env
{
    . tue-env $@
}

function _tue-env
{
    local cur=${COMP_WORDS[COMP_CWORD]}
    local prev=${COMP_WORDS[COMP_CWORD-1]}

    if [ $COMP_CWORD -eq 1 ]; then
        COMPREPLY=( $(compgen -W "init list switch list-current remove cd set-default config init-targets targets" -- $cur) )
    else
        cmd=${COMP_WORDS[1]}
        if [[ $cmd == "switch" ]] || [[ $cmd == "remove" ]] || [[ $cmd == "cd" ]] || [[ $cmd == "set-default" ]] || [[ $cmd == "init-targets" ]] || [[ $cmd == "targets" ]]
        then
            if [ $COMP_CWORD -eq 2 ]
            then
                local envs=
                [ -d $TUE_DIR/user/envs ] && envs=$(ls $TUE_DIR/user/envs)
                COMPREPLY=( $(compgen -W "$envs" -- $cur) )
            fi
        fi
    fi
}
complete -F _tue-env tue-env
