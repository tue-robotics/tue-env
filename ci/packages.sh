#!/bin/bash

if [[ $TRAVIS_PULL_REQUEST == "false" ]]
then
    diff_tag="HEAD^"
else
    diff_tag=$(git merge-base HEAD $TRAVIS_BRANCH)
fi

dir_mod=$(git diff-tree --name-only HEAD $diff_tag | xargs ls -dl 2>/dev/null |  grep "^d" | grep -v "\." | awk '{print $NF}')
if [ $dir_mod ]
then
    export PACKAGES=$dir_mod
else
    # When no pkg is modified, build all pkgs (=all sub directories)
    export PACKAGES=$(ls -l | grep ^d | awk '{print $NF}')
fi

echo -e "\e[35m\e[1m PACKAGES: \e[0m"
for PKG in $PACKAGES
do
    echo -e "\e[35m\e[1m $PKG \e[0m"
done
