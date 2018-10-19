# ----------------------------------------------------------------------------------------------------
#                                              TUE-ENV
# ----------------------------------------------------------------------------------------------------

function tue-env
{
    if [ -z "$1" ]
    then
        echo """tue-env is a tool for switching between different installation environments.

    Usage: tue-env COMMAND [ARG1 ARG2 ...]

    Possible commands:

        init           - Initializes new environment
        remove         - Removes an existing enviroment (no data is lost)
        switch         - Switch to a different environment
        config         - Configures current environment
        set-default    - Set default environment
        list           - List all possible environments
        list-current   - Shows current environment
        cd             - Changes directory to environment directory
"""
        return 1
    fi

    cmd=$1
    shift

    # Make sure the correct directories are there
    mkdir -p $TUE_DIR/user/envs

    if [[ $cmd == "init" ]]
    then
        if [ -z "$1" ]
        then
            echo "Usage: tue-env init NAME [ DIRECTORY ]"
            return 1
        fi

        local dir=$PWD   # default directory is current directory
        [ -z "$2" ] || dir=$2

        # TODO: make dir absolute

        if [ -f $TUE_DIR/user/envs/$1 ]
        then
            echo "[tue-env] Environment '$1' already exists"
            return 1
        fi

        if [ -d $dir/.env ]
        then
            echo "[tue-env] Directory '$dir' is already an environment directory."
            return 1
        fi

        echo "$dir" > $TUE_DIR/user/envs/$1
        mkdir -p $dir/.env
    elif [[ $cmd == "remove" ]]
    then
        if [ -z "$1" ]
        then
            echo "Usage: tue-env remove ENVIRONMENT"
            return 1
        fi

        if [ ! -f $TUE_DIR/user/envs/$1 ]
        then
            echo "[tue-env] No such environment: '$1'."
            return 1
        fi

        dir=`cat $TUE_DIR/user/envs/$1`
        rm $TUE_DIR/user/envs/$1
        rm -rf $dir
    elif [[ $cmd == "switch" ]]
    then
        if [ -z "$1" ]
        then
            echo "Usage: tue-env switch ENVIRONMENT"
            return 1
        fi

        if [ ! -f $TUE_DIR/user/envs/$1 ]
        then
            echo "[tue-env] No such environment: '$1'."
            return 1
        fi

        export TUE_ENV=$1
        export TUE_ENV_DIR=`cat $TUE_DIR/user/envs/$1`
        
        source ~/.bashrc

    elif [[ $cmd == "set-default" ]]
    then
        if [ -z "$1" ]
        then
            echo "Usage: tue-env set-default ENVIRONMENT"
            return 1
        fi

        mkdir -p $TUE_DIR/user/config
        echo "$1" > $TUE_DIR/user/config/default_env

    elif [[ $cmd == "config" ]]
    then
        vim $TUE_ENV_DIR/.env/setup/user_setup.bash

    elif [[ $cmd == "cd" ]]
    then
        local env=$1
        [ -n "$env" ] || env=$TUE_ENV

        local dir=`cat $TUE_DIR/user/envs/$env`
        cd $dir

    elif [[ $cmd == "list" ]]
    then
        [ -d $TUE_DIR/user/envs ] || return 0

        for env in `ls $TUE_DIR/user/envs`
        do
            echo $env
        done
    elif [[ $cmd == "list-current" ]]
    then
        echo $TUE_ENV
    else
        echo "[tue-env] Unknown command: '$cmd'"
        return 1
    fi
}

# ----------------------------------------------------------------------------------------------------

function _tue-env
{
    local cur=${COMP_WORDS[COMP_CWORD]}
    local prev=${COMP_WORDS[COMP_CWORD-1]}

    if [ $COMP_CWORD -eq 1 ]; then
        COMPREPLY=( $(compgen -W "init list switch list-current remove cd set-default config" -- $cur) )
    else
        cmd=${COMP_WORDS[1]}
        if [[ $cmd == "switch" ]] || [[ $cmd == "remove" ]] || [[ $cmd == "cd" ]] || [[ $cmd == "set-default" ]]
        then
            if [ $COMP_CWORD -eq 2 ]
            then
                local envs=
                [ -d $TUE_DIR/user/envs ] && envs=`ls $TUE_DIR/user/envs`
                COMPREPLY=( $(compgen -W "$envs" -- $cur) )        
            fi
        fi
    fi
}
complete -F _tue-env tue-env
