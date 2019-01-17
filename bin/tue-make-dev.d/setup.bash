#! /usr/bin/env bash

function _tue-make-dev
{
    local cur=${COMP_WORDS[COMP_CWORD]}
    local prev=${COMP_WORDS[COMP_CWORD-1]}

    COMPREPLY=( $(compgen -W "`_list_subdirs $_TUE_CATKIN_DEV_DIR/src`" -- $cur) )
}
complete -F _tue-make-dev tue-make-dev
complete -F _tue-make-dev tue-make-dev-isolated
