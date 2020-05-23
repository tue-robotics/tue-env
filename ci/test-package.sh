#! /usr/bin/env bash
# shellcheck disable=SC2016
#
# Package builder (CI script)
# This script can only be run after the build-package.sh

# Stop on errors
set -o errexit

# Execute script only in a CI environment
if [ "$CI" != "true" ]
then
    echo -e "\e[35m\e[1m Error!\e[0m Trying to execute a CI script in a non-CI environment. Exiting script."
    exit 1
fi

# Standard argument parsing, example: build-package --package=ros_robot
for i in "$@"
do
    case $i in
        -p=* | --package=* )
            PACKAGE="${i#*=}" ;;

        * )
            # unknown option
            echo -e "\e[35m\e[1m Unknown input argument '$i'. Check CI .yml file \e[0m"
            exit 1 ;;
    esac
    shift
done

echo -e "\e[35m\e[1m PACKAGE     = ${PACKAGE} \e[0m"

# If packages is non-zero, this is a multi-package repo. In multi-package repo, check if this package needs CI.
# If a single-package repo, CI is always needed.
# shellcheck disable=SC2153
if [ -n "$PACKAGES" ] && ! echo "$PACKAGES" | grep -sqw "$PACKAGE"
then
    echo -e "\e[35m\e[1m No changes in this package, so no need to run CI \e[0m"
    exit 0
fi

# Run unit tests
echo -e "\e[35m\e[1m Run tests on this package (catkin run_tests --this --no-deps -DCATKIN_ENABLE_TESTING=ON) \e[0m"
docker exec -t tue-env bash -c 'source ~/.bashrc; cd "$TUE_SYSTEM_DIR"/src/"$PACKAGE" && catkin run_tests --this --no-status --no-deps -DCATKIN_ENABLE_TESTING=ON'

# Check results of unit tests
echo -e "\e[35m\e[1m Check results of the tests on this package (catkin_test_results build/$PACKAGE) \e[0m"
docker exec -t tue-env bash -c 'source ~/.bashrc; cd "$TUE_SYSTEM_DIR" && [ ! -d build/"$PACKAGE" ] || catkin_test_results build/"$PACKAGE"'
