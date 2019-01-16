#!/bin/bash
_tue-check-env-vars || return 1

# Update installer
if [ ! -d $TUE_DIR ]
then
    git clone https://github.com/tue-robotics/tue-env.git $TUE_DIR
elif [[ -n "$CI" ]] #Do not update with continuous integration but do fetch to refresh available branches
then
    echo -en "Fetching tue-get... "
    git -C $TUE_DIR fetch
else
    echo -en "Updating tue-get... "
    git -C $TUE_DIR pull --ff-only --prune

    error_code=$?

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

if [ ! -d $TUE_ENV_TARGETS_DIR ]
then
    git clone https://github.com/tue-robotics/tue-env-targets.git $TUE_ENV_TARGETS_DIR
elif [[ -n "$CI" ]] #Do not update with continuous integration but do fetch to refresh available branches
then
    echo -en "Fetching tue-env-targets... "
    git -C $TUE_ENV_TARGETS_DIR fetch
    git -C $TUE_ENV_TARGETS_DIR checkout ${TRAVIS_PULL_REQUEST_BRANCH:-$TRAVIS_BRANCH}
else
    echo -en "Updating tue-env-targets... "
    git -C $TUE_ENV_TARGETS_DIR pull --ff-only --prune

    error_code=$?

    if [ ! $error_code -eq 0 ]
    then
        # prompt for conformation
        exec < /dev/tty
        read -p "[tue-env-targets] Could not update targets. Continue? " -n 1 -r
        exec <&-
        echo    # (optional) move to a new line
        if [[ ! $REPLY =~ ^[Yy]$ ]]
        then
            exit 1
        fi
    fi
fi

# Run installer
source $TUE_DIR/installer/tue-install-impl.bash $@
