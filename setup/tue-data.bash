#! /usr/bin/env bash

LOCAL_DATA_DIR=~/ros/data
ROBOTICSSRV_LOGIN=data@roboticssrv.wtb.tue.nl
REMOTE_DATA_DIR=/home/data/data

function tue-data
{
    # Temporary check
    if [ -d ~/ros/data ] && [ ! -d ~/ros/data/private ]
    then
        mv ~/ros/data ~/ros/data-tmp-dir
        mkdir -p ~/ros/data
        mv ~/ros/data-tmp-dir ~/ros/data/private

        echo -e "\e[0;33m[IMPORTANT] I moved the data folder. '~/ros/data' has now become '~/ros/data/private'\e[0m"
        echo ""
    fi

    if [ -z "$1" ]
    then
        # shellcheck disable=SC1078,SC1079
        echo """tue-data is a tool for uploading data to and downloading data from the TU/e robotics server.

    Usage: tue-data COMMAND [ARG1 ARG2 ...]

    Possible commands:

        list           - Lists contents of current folder on server
        update         - Downloads contents of current folder from server
        update-dirs    - Locally updates the data folder structure
        store          - Uploads current folder to server
"""
        return 1
    fi

    local cmd
    cmd=$1
    shift

    if [[ $cmd == "update-dirs" ]]
    then
        rsync -a --include='*/' --exclude='*' $ROBOTICSSRV_LOGIN:$REMOTE_DATA_DIR/ $LOCAL_DATA_DIR/ --progress
    elif [[ $cmd == "list" ]]
    then
        # Check if user is in the data directory
        if [[ $PWD != "$LOCAL_DATA_DIR"* ]]
        then
            echo "You are not in the data folder ('$LOCAL_DATA_DIR')"
            return 1
        fi

        # Determine current directory relative to local data dir root
        local rel_dir
        rel_dir=${PWD#"${LOCAL_DATA_DIR}"}

        # shellcheck disable=SC2029
        ssh $ROBOTICSSRV_LOGIN "ls $REMOTE_DATA_DIR/$rel_dir -alh"
    elif [[ $cmd == "update" ]]
    then
        # Check if user is in the data directory
        if [[ $PWD != "$LOCAL_DATA_DIR"* ]]
        then
            echo "You are not in the data folder ('$LOCAL_DATA_DIR')"
            return 1
        fi

        # Determine current directory relative to local data dir root
        local rel_dir
        rel_dir=${PWD#"${LOCAL_DATA_DIR}"}

        rsync "$ROBOTICSSRV_LOGIN":"$REMOTE_DATA_DIR"/"$rel_dir"/ . -av --progress --exclude=".git"
    elif [[ $cmd == "store" ]]
    then
        if [ -z "$1" ]
        then
            # shellcheck disable=SC1078,SC1079
            echo """Usage: tue-data store <FILE-OR-FOLDER>

For example, to store everything in the current folder, use:

    tue-data store .
"""
            return 1
        fi

        local target
        target="$1"
        if [[ $target != "/"* ]]
        then
            target="$PWD/$target"
        fi

        # Check if user is in the data directory
        if [[ $target != "$LOCAL_DATA_DIR"* ]]
        then
            echo "You are not in the data folder ('$LOCAL_DATA_DIR')"
            return 1
        fi

        # Determine current directory relative to local data dir root
        local rel_dir
        rel_dir=${target#"${LOCAL_DATA_DIR}"}

        rsync "$LOCAL_DATA_DIR"/./"$rel_dir" "$ROBOTICSSRV_LOGIN":"$REMOTE_DATA_DIR"/ -av --relative --progress --exclude=".git"
    fi
}

function _tue-data
{
    local cur
    cur=${COMP_WORDS[COMP_CWORD]}

    if [ "$COMP_CWORD" -eq 1 ]
    then
        mapfile -t COMPREPLY < <(compgen -W "update-dirs list update store" -- "$cur")
    fi
}
complete -F _tue-data tue-data
