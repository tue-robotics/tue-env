#! /usr/bin/env bash

exclude_dirs=""
for i in "$@"
do
    case $i in
        -r=* | --pullrequest=* )
            PULL_REQUEST="${i#*=}"
	    ;;
        -b=* | --branch=* )
            BRANCH="${i#*=}"
	    ;;
        * ) exclude_dirs="$exclude_dirs $1"
	    ;;
    esac
    shift
done

exclude_dirs=$(echo "$exclude_dirs" | xargs ls -dl 2>/dev/null |  grep "^d" | grep -v "\." | awk '{print $NF}')

if [[ $PULL_REQUEST == "false" ]]
then
    diff_tag="HEAD^"
else
    diff_tag=$(git merge-base HEAD remotes/origin/"$BRANCH")
fi
echo "diff tag command"
echo "git merge-base HEAD "$BRANCH""

echo "diff_tag"
echo "$diff_tag"

mod_files=$(git diff-tree --name-only HEAD "$diff_tag")
dir_mod=$(echo "$mod_files" | xargs ls -dl 2>/dev/null |  grep "^d" | grep -v "\." | awk '{print $NF}')
echo "dir_mod"
echo "$dir_mod"

if [[ $mod_files == *".travis.yml"* ]] || [[ $mod_files == *"azure-pipelines.yml"* ]]
then
    # When CI config is modified, build all pkgs (=all sub directories)
    PACKAGES=$(printf "%s\n" ./*/ | cut -f2 -d '/')
elif [ "$dir_mod" ]
then
	# Else only the modified pkgs
    PACKAGES=$dir_mod
else
	PACKAGES=""
fi
echo "PACKAGES1"
echo "$PACKAGES"

PACKAGES=$(echo "$PACKAGES" | xargs ls -dl 2>/dev/null |  grep "^d" | grep -v "\." | awk '{print $NF}'| grep -v -w "$exclude_dirs")
export PACKAGES
echo "PACKAGES2"
echo "$PACKAGES"

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
export PACKAGES_DICT

echo -e "\e[35m\e[1m PACKAGES: \e[0m"
for PKG in $PACKAGES
do
    echo -e "\e[35m\e[1m $PKG \e[0m"
done
