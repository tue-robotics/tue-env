#! /usr/bin/env bash

BRANCH=""
for i in "$@"
do
    case $i in
        -r=* | --repo=* )
            REPO="${i#*=}"
        ;;
        -b=* | --branch=* )
            BRANCH="${i#*=}"
        ;;
        * )
        # unknown option
        if [[ -n "$i" ]]
        then
            echo -e "\e[35m\e[1mUnknown input argument '$i'. Check CI .yml file\e[0m"
            exit 1
        fi ;;
    esac
    shift
done

echo -e "\e[35m\e[1mREPO   = ${REPO}\e[0m"
echo -e "\e[35m\e[1mBRANCH = ${BRANCH}\e[0m"

echo -e "\e[35m\e[1mGetting default branch by GitHub API of: ${REPO}\e[0m"

DEFAULT_BRANCH=$(curl -H "Accept: application/vnd.github.v3+json" https://api.github.com/repos/"${REPO}" 2>/dev/null | jq ".default_branch" -r)

echo -e "\e[35m\e[1mBRANCH         = ${BRANCH}\e[0m"
echo -e "\e[35m\e[1mDEFAULT_BRANCH = ${DEFAULT_BRANCH}\e[0m"

if [[ "$BRANCH" == "$DEFAULT_BRANCH" ]]
then
    echo -e "\e[35m\e[1mOn the default branch, clearing BRANCH variable\e[0m"
    BRANCH=""
else
    echo -e "\e[35m\e[1mOn a feature branch, keeping the BRANCH variable: ${BRANCH}\e[0m"
fi

export BRANCh
