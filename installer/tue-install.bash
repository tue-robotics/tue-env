#! /usr/bin/env bash
_tue-check-env-vars || exit 1

function _function_test
{
    local function_missing
    function_missing="false"
    # shellcheck disable=SC2048
    for func in $*
    do
        declare -f "$func" > /dev/null || { echo -e "\e[38;1mFunction '$func' missing, resource the setup\e[0m" && function_missing="true"; }
    done
    [[ "$function_missing" == "true" ]] && exit 1
}

_function_test _git_https_or_ssh

# Update installer
if [ ! -d "$TUE_DIR" ]
then
    echo "[tue-get] 'TUE_DIR' $TUE_DIR doesn't exist"
    exit 1
else
    current_url=$(git -C "$TUE_DIR" config --get remote.origin.url)
    new_url=$(_git_https_or_ssh "$current_url")

    if ! grep -q "^git@.*\.git$\|^https://.*\.git$" <<< "$new_url"
    then
        # shellcheck disable=SC2140
        echo -e "[tue-get] (tue-env) new_url: '$new_url' is invalid. It is generated from the current_url: '$current_url'\n"\
"The problem will probably be solved by resourcing the setup"
        exit 1
    fi


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

        if ! git -C "$TUE_DIR" pull --ff-only --prune
        then
            # prompt for conformation
            exec < /dev/tty
            read -p "[tue-get] Could not update tue-get. Continue? [y/N]" -n 1 -r
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
    new_url=$(_git_https_or_ssh "$current_url")

    if ! grep -q "^git@.*\.git$\|^https://.*\.git$" <<< "$new_url"
    then
        # shellcheck disable=SC2140
        echo -e "[tue-get] (tue-env-targets) new_url: '$new_url' is invalid. It is generated from the current_url: '$current_url'\n"\
"The problem will probably be solved by resourcing the setup"
        exit 1
    fi

    if [ "$current_url" != "$new_url" ]
    then
        git -C "$TUE_ENV_TARGETS_DIR" remote set-url origin "$new_url"
        echo -e "[tue-env-targets] Origin has switched to $new_url"
    fi

    echo -en "[tue-env-targets] Updating targets... "

    if ! { git -C "$TUE_ENV_TARGETS_DIR" pull --ff-only --prune && git -C "$TUE_ENV_TARGETS_DIR" submodule sync --recursive 1>/dev/null && git -C "$TUE_ENV_TARGETS_DIR" submodule update --init --recursive; } && [ -z "$CI" ]
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
    BRANCH=""
    for var in "$@"
    do
        if [[ "${var}" == --try-branch* ]] || [[ "${var}" == --branch* ]]
        then
            # shellcheck disable=SC2001
            BRANCH="$(echo "${var}" | sed -e 's/^[^=]*=//g')${BRANCH:+ ${BRANCH}}"
        fi
    done

    current_branch=$(git -C "${TUE_ENV_TARGETS_DIR}" rev-parse --abbrev-ref HEAD)

    if ! git -C "${TUE_ENV_TARGETS_DIR}" rev-parse --quiet --verify origin/"${current_branch}" 1>/dev/null
    then
        echo -e "[tue-env-targets] Current branch '${current_branch}' isn't available anymore, switching to the default branch"
        __tue-git-checkout-default-branch "${TUE_ENV_TARGETS_DIR}"
        git -C "${TUE_ENV_TARGETS_DIR}" pull --ff-only --prune
        git -C "${TUE_ENV_TARGETS_DIR}" submodule sync --recursive 2>&1
        git -C "${TUE_ENV_TARGETS_DIR}" submodule update --init --recursive 2>&1
        current_branch=$(git -C "${TUE_ENV_TARGETS_DIR}" rev-parse --abbrev-ref HEAD)
        echo -e "[tue-env-targets] Switched to the default branch, '${current_branch}'"
    fi

    for branch in ${BRANCH}
    do
        echo -en "[tue-env-targets] Trying to switch to branch '${branch}'..."

        if git -C "${TUE_ENV_TARGETS_DIR}" rev-parse --quiet --verify origin/"${branch}" 1>/dev/null
        then
            if [[ "${current_branch}" == "${branch}" ]]
            then
                echo -en "Already on branch ${branch}"
            else
                git -C "${TUE_ENV_TARGETS_DIR}" checkout "${branch}" --recurse-submodules -- 2>&1
                git -C "${TUE_ENV_TARGETS_DIR}" submodule sync --recursive 2>&1
                git -C "${TUE_ENV_TARGETS_DIR}" submodule update --init --recursive 2>&1
                echo -e "Switched to branch ${branch}"
            fi
            break
        else
            echo # (Optional) move to a new line
            echo -e "[tue-env-targets] Branch '${branch}' does not exist. Current branch is '${current_branch}'"
        fi
    done
fi

# Run installer
# shellcheck disable=SC1090
source "$TUE_DIR"/installer/tue-install-impl.bash "$@"
