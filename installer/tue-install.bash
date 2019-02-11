#!/bin/bash
_tue-check-env-vars || return 1

# Update installer
if [ ! -d $TUE_DIR ]
then
    echo "[tue-get] 'TUE_DIR' $TUE_DIR doesn't exist"
    exit 1
elif [[ -n "$CI" ]] #Do not update with continuous integration but do fetch to refresh available branches
then
    echo -e "[tue-get] Fetching tue-get... "
    git -C $TUE_DIR fetch
else
    echo -en "[tue-get] Updating tue-get... "
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
    echo "[tue-get] 'TUE_ENV_TARGETS_DIR' $TUE_ENV_TARGETS_DIR doesn't exist"
    echo """To setup the default tue-env targets repository do,

tue-env init-targets https://github.com/tue-robotics/tue-env-targets.git
"""
    exit 1
else
    echo -en "[tue-env-targets] Updating targets... "
    git -C $TUE_ENV_TARGETS_DIR pull --ff-only --prune

    error_code=$?

    if [ ! $error_code -eq 0 ] && [ -z "$CI" ]
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

if [[ -n "$CI" ]] #With continuous integration try to switch the targets repo to the PR branch
then
    current_branch=$(git -C $TUE_ENV_TARGETS_DIR rev-parse --abbrev-ref HEAD)

    # BRANCH is an environment variable set by ci/install-package.sh in the
    # Docker container. The container has no knowledge about TRAVIS environment
    # variable. BRANCH=${TRAVIS_PULL_REQUEST_BRANCH:-$TRAVIS_BRANCH}
    if [ -n $BRANCH ]
    then
        echo -en "[tue-env-targets] Trying to switch to branch $BRANCH..."
        test_branch=$(git -C $TUE_ENV_TARGETS_DIR branch -a 2> /dev/null | grep -q $BRANCH)
        if [ $? -eq 0 ]
        then
            if [[ "$current_branch" == "$BRANCH" ]]
            then
                echo -e "[tue-env-targets] Already on branch $BRANCH"
            else
                git -C $TUE_ENV_TARGETS_DIR checkout $BRANCH 2>&1
                echo -e "[tue-env-targets] Switchted to branch $BRANCH"
            fi
        else
            echo -e "[tue-env-targets] Branch '$BRANCH' does not exist. Current branch is $current_branch"
        fi
    fi
fi

# Run installer
source $TUE_DIR/installer/tue-install-impl.bash $@
