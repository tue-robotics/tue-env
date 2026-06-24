#! /usr/bin/env bash

BRANCH=""
DEFAULT_BRANCH=""
for i in "$@"
do
    case $i in
        -b=* | --branch=* )
            BRANCH="${i#*=}"
        ;;
        -d=* | --default-branch=* )
            DEFAULT_BRANCH="${i#*=}"
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

if [[ -z "${DEFAULT_BRANCH}" ]]
then
    echo -e "\e[35;1mNo default branch provided\e[0m"
    DEFAULT_BRANCH=$(git remote show origin 2>/dev/null | grep HEAD | awk '{print $3}')
fi

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
