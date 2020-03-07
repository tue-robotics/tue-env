#! /usr/bin/env bash
_tue-check-env-vars || exit 1

# Update installer
if [ ! -d "$TUE_DIR" ]
then
    echo "[tue-get] 'TUE_DIR' $TUE_DIR doesn't exist"
    exit 1
else
    current_url=$(git -C "$TUE_DIR" config --get remote.origin.url)
    new_url=$(_github_https_or_ssh "$current_url")

    if [ "$current_url" != "$new_url" ]
    then
        git -C "$TUE_DIR" remote set-url origin "$new_url"
        echo -e "[tue-get] Origin has switched to $new_url"
    fi

    if [[ -n "$CI" ]]
    then
        # Do not update with continuous integration but do fetch to refresh available branches
        echo -e "[tue-get] Fetching tue-get... "
        git -C "$TUE_DIR" fetch
    else
        echo -en "[tue-get] Updating tue-get... "

        if ! git -C "$TUE_DIR" pull --prune --rebase
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
fi

if [ ! -d "$TUE_ENV_TARGETS_DIR" ]
then
    echo "[tue-get] 'TUE_ENV_TARGETS_DIR' $TUE_ENV_TARGETS_DIR doesn't exist"
    # shellcheck disable=SC1078,SC1079
    echo """To setup the default tue-env targets repository do,

tue-env init-targets https://github.com/tue-robotics/tue-env-targets.git
"""
    exit 1
else
    current_url=$(git -C "$TUE_ENV_TARGETS_DIR" config --get remote.origin.url)
    new_url=$(_github_https_or_ssh "$current_url")

    if [ "$current_url" != "$new_url" ]
    then
        git -C "$TUE_ENV_TARGETS_DIR" remote set-url origin "$new_url"
        echo -e "[tue-env-targets] Origin has switched to $new_url"
    fi

    echo -en "[tue-env-targets] Updating targets... "

    if ! git -C "$TUE_ENV_TARGETS_DIR" pull --prune --recurse-submodules --rebase && [ -z "$CI" ]
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

if [[ -n "$CI" ]] # With continuous integration try to switch the targets repo to the PR branch
then
    current_branch=$(git -C "$TUE_ENV_TARGETS_DIR" rev-parse --abbrev-ref HEAD)

    # BRANCH is an environment variable set by ci/install-package.sh in the
    # Docker container. The container has no knowledge about TRAVIS environment
    # variable. BRANCH=${TRAVIS_PULL_REQUEST_BRANCH:-$TRAVIS_BRANCH}
    if [ -n "$BRANCH" ]
    then
        echo -en "[tue-env-targets] Trying to switch to branch $BRANCH..."
        if git -C "$TUE_ENV_TARGETS_DIR" rev-parse --quiet --verify origin/"$BRANCH" 1>/dev/null
        then
            if [[ "$current_branch" == "$BRANCH" ]]
            then
                echo -en "Already on branch $BRANCH"
            else
                git -C "$TUE_ENV_TARGETS_DIR" checkout "$BRANCH" 2>&1
                git -C "$TUE_ENV_TARGETS_DIR" submodule update --init --recursive 2>&1
                echo -en "Switched to branch $BRANCH"
            fi
        else
            echo # (Optional) move to a new line
            echo -e "[tue-env-targets] Branch '$BRANCH' does not exist. Current branch is $current_branch"
        fi
    fi
fi

# Run installer
# shellcheck disable=SC1090
source "$TUE_DIR"/installer/tue-install-impl.bash "$@"
