#! /usr/bin/env bash

function _tue-make
{
    local cur=${COMP_WORDS[COMP_CWORD]}
    local prev=${COMP_WORDS[COMP_CWORD-1]}

    COMPREPLY=( $(compgen -W "`_list_subdirs $_TUE_CATKIN_SYSTEM_DIR/src`" -- $cur) )
}

complete -F _tue-make tue-make
complete -F _tue-make tue-make-system
