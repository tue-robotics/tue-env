#! /usr/bin/env bash

function _tue-data
{
    local cur=${COMP_WORDS[COMP_CWORD]}
    local prev=${COMP_WORDS[COMP_CWORD-1]}

    if [ $COMP_CWORD -eq 1 ]
    then
        COMPREPLY=( $(compgen -W "update-dirs list update store" -- $cur) )
    fi
}
complete -F _tue-data tue-data
