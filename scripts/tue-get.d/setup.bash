#! /usr/bin/env bash

function _tue-get
{
    local cur=${COMP_WORDS[COMP_CWORD]}
    local prev=${COMP_WORDS[COMP_CWORD-1]}

    if [ $COMP_CWORD -eq 1 ]; then
        local IFS=$'\n'
        options="'dep '\n'install '\n'update '\n'remove '\n'list-installed '\n'show '"
        COMPREPLY=( $(compgen -W "$(echo -e "$options")" -- $cur) )
    else
        cmd=${COMP_WORDS[1]}
        if [[ $cmd == "install" ]]
        then
            local IFS=$'\n'
            COMPREPLY=( $(compgen -W "$(echo -e "$(ls $TUE_ENV_TARGETS_DIR | sed "s/.*/'& '/g")\n'--debug '\n'--branch='")" -- $cur) )
        elif [[ $cmd == "dep" ]]
        then
            local IFS=$'\n'
            COMPREPLY=( $(compgen -W "$(echo -e "$(ls $TUE_ENV_DIR/.env/dependencies | sed "s/.*/'& '/g")\n'--plain '\n'--verbose '\n'--all '\n'--level='")" -- $cur) )
        elif [[ $cmd == "update" ]]
        then
            local IFS=$'\n'
            COMPREPLY=( $(compgen -W "$(echo -e "$(ls $TUE_ENV_DIR/.env/dependencies | sed "s/.*/'& '/g")\n'--debug '\n'--branch='")" -- $cur) )
        elif [[ $cmd == "remove" ]]
        then
            local IFS=$'\n'
            COMPREPLY=( $(compgen -W "`ls $TUE_ENV_DIR/.env/installed | sed "s/.*/'& '/g"`" -- $cur) )
        elif [[ $cmd == "show" ]]
        then
            local IFS=$'\n'
            COMPREPLY=( $(compgen -W "`ls $TUE_ENV_TARGETS_DIR | sed "s/.*/'& '/g"`" -- $cur) )
        else
            COMREPLY=""
        fi
    fi
}
complete -o nospace -F _tue-get tue-get
