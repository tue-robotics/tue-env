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
        * ) exclude_dirs="$exclude_dirs $1"
        ;;
    esac
    shift
done

exclude_dirs=$(echo "$exclude_dirs" | xargs ls -dl 2>/dev/null |  grep "^d" | grep -v "\." | awk '{print $NF}')

if [ -n "$COMMIT_RANGE" ]
then
    newest_commit=${COMMIT_RANGE%...*}
    oldest_commit=${COMMIT_RANGE#*...}

    mod_files=$(git diff-tree --name-only "$newest_commit" "$oldest_commit")
else
    # For compatibility
    if [[ $PULL_REQUEST == "false" ]]
    then
        diff_tag="HEAD^"
    else
        diff_tag=$(git merge-base HEAD remotes/origin/"$BRANCH")
    fi

    mod_files=$(git diff-tree --name-only HEAD "$diff_tag")
fi

dir_mod=$(echo "$mod_files" | xargs ls -dl 2>/dev/null |  grep "^d" | grep -v "\." | awk '{print $NF}')

if [ "$dir_mod" ]
then
    # Else only the modified pkgs
    PACKAGES=$dir_mod
else
    PACKAGES=""
fi

PACKAGES=$(echo "$PACKAGES" | xargs ls -dl 2>/dev/null |  grep "^d" | grep -v "\." | awk '{print $NF}'| grep -v -w "$exclude_dirs")
export PACKAGES

PACKAGES_DICT="{"
for PKG in $PACKAGES
do
    if [[ "$PACKAGES_DICT" != "{" ]]
    then
        PACKAGES_DICT+=", "
    fi
    PACKAGES_DICT+="'${PKG}': {'PACKAGE': '${PKG}'}"
done
PACKAGES_DICT+="}"
if [[ "$PACKAGES_DICT" == "{}" ]]
then
	PACKAGES_DICT=""
fi
export PACKAGES_DICT

echo -e "\e[35m\e[1m PACKAGES: \e[0m"
for PKG in $PACKAGES
do
    echo -e "\e[35m\e[1m $PKG \e[0m"
done
