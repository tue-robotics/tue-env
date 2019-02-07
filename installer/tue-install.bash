#!/bin/bash
_tue-check-env-vars || return 1

# Update installer
if [ ! -d $TUE_DIR ]
then
    echo "[tue-get] 'TUE_DIR' $TUE_DIR doesn't exist"
    exit 1
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
    echo "[tue-get] 'TUE_ENV_TARGETS_DIR' $TUE_ENV_TARGETS_DIR doesn't exist"
    echo """To setup the default tue-env targets repository do,

tue-env init-targets https://github.com/tue-robotics/tue-env-targets.git
"""
    exit 1
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

# If the same branch as the PR exists on tue-env-targets, switch to it
if [[ -n "$CI" ]]
then
    branch=${TRAVIS_PULL_REQUEST_BRANCH:-$TRAVIS_BRANCH}
    echo "CI branch: $branch"
    if [ -n $branch ]
    then
        test_branch=$(git -C $TUE_ENV_TARGETS_DIR branch -a 2> /dev/null | grep $branch || echo "does not exist")
        if [ ! "$test_branch" == "does not exist" ]
        then
            local current_branch=$(git -C $TUE_ENV_TARGETS_DIR rev-parse --abbrev-ref HEAD)
            if [[ "$current_branch" == "$branch" ]]
            then
                echo "[tue-env-targets] Already on branch $branch"
            else
                git -C $pkg_dir checkout $branch 2>&1
                echo "[tue-env-targets] Switchted to branch $branch"
            fi
        fi
    fi
fi

# Run installer
source $TUE_DIR/installer/tue-install-impl.bash $@
