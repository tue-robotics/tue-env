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
        remove         - Removes an existing enviroment
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
            echo "Usage: tue-env init NAME [ DIRECTORY ] [ TARGETS GIT URL ]"
            return 1
        fi

        local dir=$PWD   # default directory is current directory
        [ -z "$2" ] || dir=$2
        dir="$( realpath $dir )"

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
        echo "[tue-env] Created new environment $1"

        if [ -n "$3" ]
        then
            $(tue-env init-targets $3)
        fi

    elif [[ $cmd == "remove" ]]
    then
        if [ -z "$1" ]
        then
            echo """Usage: tue-env remove [options] ENVIRONMENT
options:
    --purge
        Using this would completely remove the selected ENVIRONMENT if it exists"""
            return 1
        else
            # Set purge to be false by default
            PURGE=false
            env=""
            while test $# -gt 0
            do
                case "$1" in
                    --purge)
                        PURGE=true
                        ;;
                    --*)
                        echo "[tue-env] Unknown option $1"
                        ;;
                    *)
                        # Read only the first passed environment name and ignore
                        # the rest
                        if [ -z $env ]
                        then
                            env=$1
                        fi
                        ;;
                esac
                shift
            done
        fi

        if [ ! -f $TUE_DIR/user/envs/$env ]
        then
            echo "[tue-env] No such environment: '$env'."
            return 1
        fi

        dir=$(cat $TUE_DIR/user/envs/$env)
        rm $TUE_DIR/user/envs/$env

        if [ $PURGE == "false" ]
        then
            dir_moved=$dir.$(date +%F_%R)
            mv $dir $dir_moved
            echo """[tue-env] Removed environment '$env'
Moved environment directory of '$env' to '$dir_moved'"""
        else
            rm -rf $dir
            echo """[tue-env] Removed environment '$env'
Purged environment directory of '$env'"""
        fi

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
        export TUE_ENV_DIR=$(cat $TUE_DIR/user/envs/$1)

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
        echo "[tue-env] Default environment set to $1"

    elif [[ $cmd == "init-targets" ]]
    then
        if [ -z "$1" ]
        then
            echo "Usage: tue-env init-targets TARGETS_GIT_URL"
            return 1
        fi

        local url=$1
        echo "tue-env init-targets $1"

        if [ ! -d $TUE_ENV_TARGETS_DIR ]
        then
            git clone $url $TUE_ENV_TARGETS_DIR
        else
            dir_moved=$TUE_ENV_TARGETS_DIR.$(date +%F_%R)
            mv $dir $dir_moved
            git clone $url $TUE_ENV_TARGETS_DIR
        fi

    elif [[ $cmd == "config" ]]
    then
        vim $TUE_ENV_DIR/.env/setup/user_setup.bash

    elif [[ $cmd == "cd" ]]
    then
        local env=$1
        [ -n "$env" ] || env=$TUE_ENV

        local dir=$(cat $TUE_DIR/user/envs/$env)
        cd $dir

    elif [[ $cmd == "list" ]]
    then
        [ -d $TUE_DIR/user/envs ] || return 0

        for env in $(ls $TUE_DIR/user/envs)
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
        COMPREPLY=( $(compgen -W "init list switch list-current remove cd set-default config init-targets" -- $cur) )
    else
        cmd=${COMP_WORDS[1]}
        if [[ $cmd == "switch" ]] || [[ $cmd == "remove" ]] || [[ $cmd == "cd" ]] || [[ $cmd == "set-default" ]]
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
