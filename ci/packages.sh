#! /usr/bin/env bash

exclude_dirs=$(echo "$@" | xargs ls -dl 2>/dev/null |  grep "^d" | grep -v "\." | awk '{print $NF}')

if [[ $TRAVIS_PULL_REQUEST == "false" ]]
then
    diff_tag="HEAD^"
else
    diff_tag=$(git merge-base HEAD "$TRAVIS_BRANCH")
fi

dir_mod=$(git diff-tree --name-only HEAD "$diff_tag" | xargs ls -dl 2>/dev/null |  grep "^d" | grep -v "\." | awk '{print $NF}')
if [ "$dir_mod" ]
then
    PACKAGES=$dir_mod
else
    # When no pkg is modified, build all pkgs (=all sub directories)
    PACKAGES=$(printf "%s\n" ./*/ | cut -f2 -d '/')
fi

PACKAGES=$(echo "$PACKAGES" | xargs ls -dl 2>/dev/null |  grep "^d" | grep -v "\." | awk '{print $NF}'| grep -v -w "$exclude_dirs")
export PACKAGES

PACKAGES_DICT="{"
for PKG in $PACKAGES
do
	if [ $PACKAGES_DICT != "{" ]
	then
		PACKAGES_DICT+=", "
	fi
	PACKAGES_DICT+="{${PKG}: PACKAGE: ${PKG}}"
done
PACKAGES_DICT+="}"

echo -e "\e[35m\e[1m PACKAGES: \e[0m"
for PKG in $PACKAGES
do
    echo -e "\e[35m\e[1m $PKG \e[0m"
done
