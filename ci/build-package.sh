#! /usr/bin/env bash
# shellcheck disable=SC2016
#
# Package builder (CI script)
# This script can only be run after the install-package.sh

# Stop on errors
set -o errexit

# Execute script only in a CI environment
if [ "$CI" != "true" ]
then
    echo -e "\e[35m\e[1mError!\e[0m Trying to execute a CI script in a non-CI environment. Exiting script."
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
# Compile the package
if [[ "${ROS_VERSION}" == 1 ]]
then
    echo -e "\e[35m\e[1mCompile the package (catkin build -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCATKIN_ENABLE_TESTING=OFF)\e[0m"
    docker exec -t tue-env bash -c 'source ~/.bashrc; cd "${TUE_SYSTEM_DIR}"/src/"${PACKAGE}" && /usr/bin/python3 "$(command -v catkin)" build --this --no-status -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCATKIN_ENABLE_TESTING=OFF'
else
    echo -e "\e[35m\e[1mCheck for default mixin repo (colcon mixin list)\e[0m"
    MIXIN_REPOS=$(docker exec -t tue-env bash -c 'source ~/.bashrc; cd "${TUE_SYSTEM_DIR}" && colcon mixin list | grep -v "^- "' | tr -d '\r' | awk -F ": " '{print $1}')
    if ! echo -e "${MIXIN_REPOS}" | grep "^default$" -q
    then
        echo -e "\e[35m\e[1mAdd the default mixin repo (colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml)\e[0m"
        docker exec -t tue-env bash -c 'source ~/.bashrc; cd "${TUE_SYSTEM_DIR}" && colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml'
    else
        echo -e "\e[35m\e[1mDefault mixin repo already exists\e[0m"
    fi

    echo -e "\e[35m\e[1mUpdate colcon mixins (colcon mixin update)\e[0m"
    docker exec -t tue-env bash -c 'source ~/.bashrc; cd "${TUE_SYSTEM_DIR}" && colcon mixin update'

    echo -e "\e[35m\e[1mDeleting the merged install directory\e[0m"
    docker exec -t tue-env bash -c 'source ~/.bashrc; cd "${TUE_SYSTEM_DIR}" && rm -rf install'

    echo -e "\e[35m\e[1mCompile the package (colcon build --mixin rel-with-deb-info build-testing-off)\e[0m"
    docker exec -t tue-env bash -c 'source ~/.bashrc; cd "${TUE_SYSTEM_DIR}" && colcon build --packages-up-to "${PACKAGE}" --mixin rel-with-deb-info build-testing-off --event-handlers desktop_notification- status- terminal_title-'
fi
