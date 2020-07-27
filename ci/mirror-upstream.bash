#! /usr/bin/env bash

# Script to mirror a branch from a different remote

# Stop on errors
set -o errexit

# Execute script only in a CI environment
if [ "$CI" != "true" ]
then
    echo -e "\e[35m\e[1m Error!\e[0m Trying to execute a CI script in a non-CI environment. Exiting script."
    exit 1
fi

hash git 2> /dev/null || { echo "Please install git to use this script"; exit 1; }

for i in "$@"
do
    case $i in
        --upstream-remote=* )
            UPSTREAM_REMOTE="${i#*=}" ;;

        --upstream-remote-branch=* )
            UPSTREAM_REMOTE_BRANCH="${i#*=}" ;;

        --upstream-branch=* )
            UPSTREAM_BRANCH="${i#*=}" ;;

        * )
            echo -e "\e[35m\e[1m Unknown input argument '$i'. Check CI .yml file \e[0m"
            exit 1 ;;
    esac
    shift
done

echo -e "\e[35m\e[1m UPSTREAM_REMOTE        = ${UPSTREAM_REMOTE} \e[0m"
git remote add fork "$UPSTREAM_REMOTE" || { echo "Could not add remote: $UPSTREAM_REMOTE"; exit 1; }
git fetch fork || { echo "Could not fetch remote refs for $UPSTREAM_REMOTE"; exit 1; }

default_remote_branch=$(git remote show "$UPSTREAM_REMOTE" | grep HEAD | awk '{print $3;}')
[ -z "$UPSTREAM_REMOTE_BRANCH" ] && UPSTREAM_REMOTE_BRANCH="$default_remote_branch"

echo -e "\e[35m\e[1m UPSTREAM_REMOTE_BRANCH = ${UPSTREAM_REMOTE_BRANCH} \e[0m"
echo -e "\e[35m\e[1m UPSTREAM_BRANCH        = ${UPSTREAM_BRANCH} \e[0m"

# Sync with the upstream repository
git fetch origin
git checkout "$UPSTREAM_BRANCH" || { echo "Could not checkout upstream branch"; exit 1; }
git merge fork/"$UPSTREAM_REMOTE_BRANCH" --ff-only || { echo "Could not sync with upstream remote branch"; exit 1; }
git push origin "$UPSTREAM_BRANCH"

echo
echo -e "\e[35m\e[1m Synced '$UPSTREAM_BRANCH' with upstream remote"
