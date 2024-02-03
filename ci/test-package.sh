#! /usr/bin/env bash
# shellcheck disable=SC2016
#
# Package tester (CI script)
# This script can only be run after the build-package.sh

# Stop on errors
set -o errexit

# Execute script only in a CI environment
if [ "$CI" != "true" ]
then
    echo -e "\e[35m\e[1mError!\e[0mTrying to execute a CI script in a non-CI environment. Exiting script."
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
            if [[ -n "$i" ]]  # Ignore empty arguments
            then
                echo -e "\e[35m\e[1mUnknown input argument '$i'. Check CI .yml file\e[0m"
                exit 1
            fi ;;
    esac
    shift
done

echo -e "\e[35m\e[1mPACKAGE     = ${PACKAGE}\e[0m"
ROS_VERSION=$(docker exec -t tue-env bash -c 'source ~/.bashrc; echo "${ROS_VERSION}"' | tr -d '\r')
echo -e "\e[35m\e[1mROS_VERSION = ${ROS_VERSION}\e[0m"

# If packages is non-zero, this is a multi-package repo. In multi-package repo, check if this package needs CI.
# If a single-package repo, CI is always needed.
# shellcheck disable=SC2153
if [ -n "$PACKAGES" ] && ! echo "$PACKAGES" | grep -sqw "$PACKAGE"
then
    echo -e "\e[35m\e[1mNo changes in this package, so no need to run CI\e[0m"
    exit 0
fi

# Use docker environment variables in all exec commands instead of script variables
if [[ "${ROS_VERSION}" == 1 ]]
then
    # Build test targets
    echo -e "\e[35m\e[1mBuild test targets of this package (catkin build --this --no-deps -DCATKIN_ENABLE_TESTING=ON)\e[0m"
    docker exec -t tue-env bash -c 'source ~/.bashrc; cd "${TUE_SYSTEM_DIR}"/src/"${PACKAGE}" && /usr/bin/python3 "$(command -v catkin)" build --this --no-status --no-deps -DCATKIN_ENABLE_TESTING=ON'

    # Run unit tests
    echo -e "\e[35m\e[1mRun tests on this package (catkin test --this --no-deps -DCATKIN_ENABLE_TESTING=ON)\e[0m"
    docker exec -t tue-env bash -c 'source ~/.bashrc; cd "${TUE_SYSTEM_DIR}"/src/"${PACKAGE}" && /usr/bin/python3 "$(command -v catkin)" test --this --no-status --no-deps -DCATKIN_ENABLE_TESTING=ON'
else
    # Build test targets
    echo -e "\e[35m\e[1mBuild test targets of this package (colcon build --packages-select ${PACKAGE} --mixin rel-with-deb-info build-testing-on)\e[0m"
    docker exec -t tue-env bash -c 'source ~/.bashrc; cd "${TUE_SYSTEM_DIR}" && colcon build --packages-select "${PACKAGE}" --mixin rel-with-deb-info build-testing-on --event-handlers desktop_notification- status- terminal_title-'

    # Run unit tests
    echo -e "\e[35m\e[1mRun tests on this package (colcon test --packages-select ${PACKAGE} --executor sequential)\e[0m"
    docker exec -t tue-env bash -c 'source ~/.bashrc; cd "${TUE_SYSTEM_DIR}" && colcon test --packages-select "${PACKAGE}" --executor sequential --event-handlers desktop_notification- status- terminal_title- console_cohesion+'

    # Check test results
    echo -e "\e[35m\e[1mCheck test results (colcon test-result --verbose)\e[0m"
    docker exec -t tue-env bash -c 'source ~/.bashrc; cd "${TUE_SYSTEM_DIR}" && colcon test-result --verbose'
fi
