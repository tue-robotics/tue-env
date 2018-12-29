#!/bin/bash
_tue-check-env-vars || return 1

# Update installer
if [ ! -d $TUE_DIR ]
then
    git clone git@github.com:tue-robotics/tue-env.git $TUE_DIR 2> /dev/null || https://github.com/tue-robotics/tue-env.git $TUE_DIR
elif [[ -z "$CI" ]] #Do not update with continuous integration
then
    mem_pwd=$PWD
    cd $TUE_DIR
    echo -en "Updating tue-get... "
    git pull --ff-only --prune

    error_code=$?
    cd $mem_pwd

    if [ ! $error_code -eq 0 ]
    then
        # prompt for conformation
        exec < /dev/tty
        read -p "[tue-get] Could not update tue-get. Continue? " -n 1 -r
        exec <&-
        echo    # (optional) move to a new line
        if [[ ! $REPLY =~ ^[Yy]$ ]]
        then
            exit 1
        fi
    fi
fi

# Run installer
source $TUE_DIR/installer/scripts/tue-install-impl.bash $@
