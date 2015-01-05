LOCAL_DATA_DIR=~/ros/data
ROBOTICSSRV_LOGIN=amigo@192.168.2.240
REMOTE_DATA_DIR=/home/amigo/data

function tue-data
{
    if [ -z "$1" ]
    then
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
        local rel_dir=${PWD#$LOCAL_DATA_DIR}

        ssh $ROBOTICSSRV_LOGIN "ls $REMOTE_DATA_DIR/$rel_dir"
    elif [[ $cmd == "update" ]]
    then
        # Check if user is in the data directory
        if [[ $PWD != "$LOCAL_DATA_DIR"* ]]
        then
            echo "You are not in the data folder ('$LOCAL_DATA_DIR')"
            return 1
        fi

        # Determine current directory relative to local data dir root
        local rel_dir=${PWD#$LOCAL_DATA_DIR}

        rsync $ROBOTICSSRV_LOGIN:$REMOTE_DATA_DIR/$rel_dir/ . -av --progress --exclude=".svn"   
    elif [[ $cmd == "store" ]]
    then
        # Check if user is in the data directory
        if [[ $PWD != "$LOCAL_DATA_DIR"* ]]
        then
            echo "You are not in the data folder ('$LOCAL_DATA_DIR')"
            return 1
        fi

        # Determine current directory relative to local data dir root
        local rel_dir=${PWD#$LOCAL_DATA_DIR}

        rsync . $ROBOTICSSRV_LOGIN:$REMOTE_DATA_DIR/$rel_dir -av --progress --exclude=".svn"
    fi
}

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
