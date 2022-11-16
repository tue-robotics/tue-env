#! /usr/bin/env bash

exclude_dirs=""
for i in "$@"
do
    case $i in
        -c=* | --commit-range=* )
            COMMIT_RANGE="${i#*=}"
        ;;
        -r=* | --pullrequest=* )
            PULL_REQUEST="${i#*=}"
        ;;
        -b=* | --branch=* )
            BRANCH="${i#*=}"
        ;;
        -a=* | --all=* )
            ALL="${i#*=}"
        ;;
        * ) exclude_dirs="${exclude_dirs:+${exclude_dirs} }$1"
        ;;
    esac
    shift
done

echo -e "\e[35m\e[1mCOMMIT_RANGE = ${COMMIT_RANGE}\e[0m"
echo -e "\e[35m\e[1mPULL_REQUEST = ${PULL_REQUEST}\e[0m"
echo -e "\e[35m\e[1mBRANCH       = ${BRANCH}\e[0m"
echo -e "\e[35m\e[1mALL          = ${ALL}\e[0m"
echo -e "\e[35m\e[1mexclude_dirs = ${exclude_dirs}\e[0m"

exclude_dirs=$(echo "${exclude_dirs}" | xargs ls -dl 2>/dev/null |  grep "^d" | grep -v "\." | awk '{print $NF}')

if [ -n "$COMMIT_RANGE" ]
then
    oldest_commit=${COMMIT_RANGE%...*}
    newest_commit=${COMMIT_RANGE#*...}
    echo -e "\e[35m\e[1mOldest commit: ${oldest_commit}\e[0m"
    echo -e "\e[35m\e[1mNewest commit: ${newest_commit}\e[0m"

    mod_files=$(git diff-tree --name-only "${oldest_commit}" "${newest_commit}")
else
    # For compatibility
    if [[ $PULL_REQUEST == "false" ]]
    then
        diff_tag="HEAD^"
    else
        diff_tag=$(git merge-base HEAD remotes/origin/"${BRANCH}")
    fi

    mod_files=$(git diff-tree --name-only HEAD "${diff_tag}")
fi

dir_mod=$(echo "${mod_files}" | xargs ls -dl 2>/dev/null |  grep "^d" | grep -v "\." | awk '{print $NF}')

if [ "$ALL" == "true" ] || [[ ${mod_files} == *".travis.yml"* ]] || [[ ${mod_files} == *"azure-pipelines.yml"* ]] || [[ ${mod_files} == *".github"* ]]
then
    # If CI config is modified, build all pkgs (=all sub directories)
    PACKAGES=$(printf "%s\n" ./*/ | cut -f2 -d '/')
elif [ "${dir_mod}" ]
then
    # Else only the modified pkgs
    PACKAGES=${dir_mod}
else
    PACKAGES=""
fi

PACKAGES=$(echo "${PACKAGES}" | xargs ls -dl 2>/dev/null |  grep "^d" | grep -v "\." | awk '{print $NF}'| grep -v -w "${exclude_dirs}")
export PACKAGES

for PKG in $PACKAGES
do
    # shellcheck disable=SC2089
    PACKAGES_DICT="${PACKAGES_DICT:+$PACKAGES_DICT, }'${PKG}': {'PACKAGE': '${PKG}'}"
    # shellcheck disable=SC2089
    PACKAGES_LIST="${PACKAGES_LIST:+$PACKAGES_LIST, }'${PKG}'"
done
PACKAGES_DICT="{${PACKAGES_DICT}}"
PACKAGES_LIST="[${PACKAGES_LIST}]"
# shellcheck disable=SC2090
export PACKAGES_DICT
# shellcheck disable=SC2090
export PACKAGES_LIST

echo -e "\e[35m\e[1mPACKAGES:\e[0m"
for PKG in $PACKAGES
do
    echo -e "\e[35m\e[1m  ${PKG}\e[0m"
done
