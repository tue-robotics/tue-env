#! /usr/bin/env bash

BRANCH=""
for i in "$@"
do
    case $i in
        -b=* | --branch=* )
            BRANCH="${i#*=}"
        ;;
        * )
        # unknown option
        if [[ -n "$i" ]]  # Ignore empty arguments
        then
            echo -e "\e[35;1mUnknown input argument '$i'. Check CI .yml file\e[0m"
            exit 1
        fi ;;
    esac
    shift
done

DEFAULT_BRANCH=$(git remote show origin 2>/dev/null | grep HEAD | awk '{print $3}')

echo -e "\e[35;1mBRANCH         = ${BRANCH}\e[0m"
echo -e "\e[35;1mDEFAULT_BRANCH = ${DEFAULT_BRANCH}\e[0m"

if [[ "$BRANCH" == "$DEFAULT_BRANCH" ]]
then
    echo -e "\e[35;1mOn the default branch, clearing BRANCH variable\e[0m"
    BRANCH=""
else
    echo -e "\e[35;1mOn a feature branch, keeping the BRANCH variable: ${BRANCH}\e[0m"
fi

export BRANCH
