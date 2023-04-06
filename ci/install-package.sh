#! /usr/bin/env bash
# shellcheck disable=SC2016
#
# Package installer (CI script)
# This script uses the Docker image of tue-env and installs the current git
# repository as a tue-env package using tue-install in the CI

# Stop on errors
set -o errexit

# Execute script only in a CI environment
if [ "$CI" != "true" ]
then
    echo -e "\e[35m\e[1mError!\e[0m Trying to execute a CI script in a non-CI environment. Exiting script."
    exit 1
fi

# Standard argument parsing, example: install-package --branch=master --package=ros_robot
for i in "$@"
do
    case $i in
        -p=* | --package=* )
            PACKAGE="${i#*=}" ;;

        -b=* | --branch=* )
        # BRANCH should allways be targetbranch
            BRANCH="${i#*=}" ;;

        -c=* | --commit=* )
            COMMIT="${i#*=}" ;;

        -r=* | --pullrequest=* )
            PULL_REQUEST="${i#*=}" ;;

        -i=* | --image=* )
            IMAGE_NAME="${i#*=}" ;;

        -s=* | --shared=* )
            SHARED_DIR="${i#*=}" ;;

        --ssh )
            USE_SSH=true ;;

        --ssh-key=* )
            SSH_KEY="${i#*=}" ;;

        --ref-name=* )
            REF_NAME="${i#*=}" ;;

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

echo -e "\e[35m\e[1mPACKAGE      = ${PACKAGE}\e[0m"
echo -e "\e[35m\e[1mBRANCH       = ${BRANCH}\e[0m"
echo -e "\e[35m\e[1mCOMMIT       = ${COMMIT}\e[0m"
echo -e "\e[35m\e[1mPULL_REQUEST = ${PULL_REQUEST}\e[0m"

# Set default value for IMAGE_NAME
[ -z "$IMAGE_NAME" ] && IMAGE_NAME='ghcr.io/tue-robotics/tue-env-ros-noetic'
echo -e "\e[35m\e[1mIMAGE_NAME   = ${IMAGE_NAME}\e[0m"

# Set default value for directory to place mountable container assests
[ -z "$SHARED_DIR" ] && SHARED_DIR="$HOME"
echo -e "\e[35m\e[1mSHARED_DIR   = ${SHARED_DIR}\e[0m"

# Set default value for REF_NAME
[ -z "$REF_NAME" ] && REF_NAME="pull"
echo -e "\e[35m\e[1mREF_NAME     = ${REF_NAME}\e[0m"

if [ "$USE_SSH" == "true" ]
then
    eval "$(ssh-agent -s)" &> /dev/null  # Start SSH agent
    SSH_KEY_CHECK=
    { ssh-add &> /dev/null && SSH_KEY_CHECK="true"; } || SSH_KEY_CHECK="false"  # Add any existing default keys

    # Copy contents of default ssh directory, except for known_hosts file if there is
    # a pre-existing known_hosts file in the shared directory
    if [[ "${SHARED_DIR}" != "${HOME}" ]]
    then
        mkdir -p "${SHARED_DIR}"/.ssh

        { [[ -f "${SHARED_DIR}"/.ssh/known_hosts ]] && mv "${SHARED_DIR}/.ssh/known_hosts" "${SHARED_DIR}/.ssh/known_hosts_shared"; } || true

        cp -r "${HOME}"/.ssh/* "${SHARED_DIR}"/.ssh

        { [[ -f "${SHARED_DIR}"/.ssh/known_hosts_shared ]] && mv "${SHARED_DIR}/.ssh/known_hosts_shared" "${SHARED_DIR}/.ssh/known_hosts"; } || true
    fi

    if [[ -n "${SSH_KEY}" && ! -f "${SSH_KEY}" ]]
    then
        echo "SSH key does not exist at '${SSH_KEY}'"
        exit 1
    fi

    if [[ -n "${SSH_KEY}" && -f "${SSH_KEY}" ]]
    then
        chmod 600 "${SSH_KEY}"
        SSH_KEY_FINGERPRINT="$(ssh-keygen -lf "${SSH_KEY}" 2> /dev/null | awk '{print $2}')"
        [[ -z "${SSH_KEY_FINGERPRINT}" ]] && { echo "'${SSH_KEY}' has an invalid SSH fingerprint" && exit 1; }

        if [[ "$(ssh-add -l)" != *"${SSH_KEY_FINGERPRINT}"* ]]
        then
            { ssh-add "${SSH_KEY}" &> /dev/null && SSH_KEY_CHECK="true"; } || { echo "'${SSH_KEY}' is an invalid SSH key" && exit 1; }

        else
            SSH_KEY_CHECK="true"
        fi

        if [[ "${SSH_KEY_CHECK}" == "true" && "$(dirname "${SSH_KEY}")" != "${SHARED_DIR}/.ssh" ]]
        then
            cp "${SSH_KEY}" "${SHARED_DIR}"/.ssh/
        fi
    fi

    [[ "${SSH_KEY_CHECK}" == "true" ]] || { echo "No SSH keys found" && exit 1; }

    [[ -n "${SSH_KEY_FINGERPRINT}" ]] || SSH_KEY_FINGERPRINT="default"
    echo -e "\e[35m\e[1mSSH_KEY      = ${SSH_KEY_FINGERPRINT}\e[0m"

    DOCKER_SSH_AUTH_SOCK="/tmp/ssh_auth_sock"
    DOCKER_MOUNT_KNOWN_HOSTS_ARGS="-e SSH_AUTH_SOCK=$DOCKER_SSH_AUTH_SOCK --mount type=bind,source=$SHARED_DIR/.ssh,target=/tmp/.ssh"

    # Used in the print statement to reproduce CI build locally
    ADDITIONAL_ARGS_LOCAL_BUILD="--shared=/tmp/shared/${PACKAGE} --ssh"
fi

echo -e "\e[35m\e[1m
This build can be reproduced locally using the following commands:

tue-get install docker
~/.tue/ci/install-package.sh --package=${PACKAGE} --branch=${BRANCH} --commit=${COMMIT} --pullrequest=${PULL_REQUEST} --image=${IMAGE_NAME} --ref-name=${REF_NAME} ${ADDITIONAL_ARGS_LOCAL_BUILD}
~/.tue/ci/build-package.sh --package=${PACKAGE}
~/.tue/ci/test-package.sh --package=${PACKAGE}

Optionally fix your compilation errors and re-run only the last command
\e[0m"

# If packages is non-zero, this is a multi-package repo. In multi-package repo, check if this package needs CI.
# If a single-package repo, CI is always needed.
# shellcheck disable=SC2153
if [ -n "$PACKAGES" ] && ! echo "$PACKAGES" | grep -sqw "$PACKAGE"
then
    echo -e "\e[35m\e[1mNo changes in this package, so no need to run CI\e[0m"
    exit 0
fi

# Determine docker tag if the same branch exists there
BRANCH_TAG="$(echo "$BRANCH" | tr '[:upper:]' '[:lower:]' | sed -e 's:/:_:g')-amd64"

# Set the default fallback branch to latest
MASTER_TAG="latest-amd64"

# Remove any previously started containers if they exist (if not exist, still return true to let the script continue)
docker stop tue-env  &> /dev/null || true
docker rm tue-env &> /dev/null || true

# Pull the identical branch name from dockerhub if exist, use master as fallback
echo -e "\e[35m\e[1mTrying to fetch docker image: $IMAGE_NAME:$BRANCH_TAG\e[0m"
if ! docker pull "$IMAGE_NAME:$BRANCH_TAG"
then
    echo -e "\e[35m\e[1mNo worries, we just test against the master branch: $IMAGE_NAME:$MASTER_TAG\e[0m"
    docker pull "$IMAGE_NAME":"$MASTER_TAG"
    BRANCH_TAG=$MASTER_TAG
fi

DOCKER_HOME=$(docker run --name tue-env --rm "$IMAGE_NAME:$BRANCH_TAG" bash -c 'echo "$HOME"' | tr -d '\r')

# Make sure the ~/.ccache folder exists
mkdir -p "$HOME"/.ccache

# Make sure the ~/.cache/pip folder exists
mkdir -p "$HOME"/.cache/pip

# Run the docker image along with setting new environment variables
# shellcheck disable=SC2086
docker run --detach --interactive --tty -e CI="true" -e PACKAGE="$PACKAGE" -e BRANCH="$BRANCH" -e COMMIT="$COMMIT" -e PULL_REQUEST="$PULL_REQUEST" -e REF_NAME="$REF_NAME" --name tue-env --mount type=bind,source=$HOME/.ccache,target=$DOCKER_HOME/.ccache --mount type=bind,source=$HOME/.cache/pip,target=$DOCKER_HOME/.cache/pip $DOCKER_MOUNT_KNOWN_HOSTS_ARGS "$IMAGE_NAME:$BRANCH_TAG"

# Own the ~/.ccache folder for permissions
docker exec -t tue-env bash -c 'sudo chown "${USER}":"${USER}" -R ~/.ccache'

# Own the ~/.cache/pip folder for permissions
docker exec -t tue-env bash -c 'sudo chown "${USER}":"${USER}" -R ~/.cache/pip'

if [ "$USE_SSH" == "true" ]
then
    docker exec -t tue-env bash -c 'sudo chown "${USER}":"${USER}" -R /tmp/.ssh'

    docker exec -t tue-env bash -c "[[ -f /tmp/.ssh/known_hosts ]] && mv ~/.ssh/known_hosts ~/.ssh/known_hosts_container"
    docker exec -t tue-env bash -c 'sudo cp -r /tmp/.ssh/* ~/.ssh/ && sudo chown -R "${USER}":"${USER}" ~/.ssh && ls -aln ~/.ssh'

    docker exec -t tue-env bash -c "[[ -f ~/.ssh/known_hosts && -f ~/.ssh/known_hosts_container ]] && ~/.tue/ci/ssh-merge-known_hosts.py ~/.ssh/known_hosts_container ~/.ssh/known_hosts --output ~/.ssh/known_hosts"
    docker exec -e DOCKER_SSH_AUTH_SOCK="$DOCKER_SSH_AUTH_SOCK" -t tue-env bash -c 'eval "$(ssh-agent -s)" && ln -sf "$SSH_AUTH_SOCK" "$DOCKER_SSH_AUTH_SOCK" && grep -slR "PRIVATE" ~/.ssh/ | xargs ssh-add'
fi

# Use docker environment variables in all exec commands instead of script variables
# Catch the ROS_DISTRO of the docker container
# stip carriage return from docker output by "tr -d '\r'"
# see https://unix.stackexchange.com/a/487185
ROS_DISTRO=$(docker exec -t tue-env bash -c 'source ~/.bashrc; echo "$ROS_DISTRO"' | tr -d '\r')
echo -e "\e[35m\e[1mROS_DISTRO = ${ROS_DISTRO}\e[0m"

TUE_SYSTEM_DIR=$(docker exec -t tue-env bash -c 'source ~/.bashrc; echo "$TUE_SYSTEM_DIR"' | tr -d '\r')

# First install only the git repo of the package so that appropriate branch can be checked out later
echo -e "\e[35m\e[1mtue-get install ros-$PACKAGE --no-ros-deps\e[0m"
docker exec tue-env bash -c 'echo "debconf debconf/frontend select Noninteractive" | sudo debconf-set-selections'
docker exec tue-env bash -c 'source ~/.bashrc; tue-get install ros-"$PACKAGE" --no-ros-deps'

if [[ $PULL_REQUEST != "false" ]]
then
    # Fetch the merged state ref of the pull request before running tue-get install
    # with the --try-branch=PULLREQUEST option. Only the tested repo is checked
    # out to the merged pull request while all other packages are checked out
    # to their default branch.
    # This is needed before tue-get, so also new deps are installed.
    # After a tue-get run, we checkout forced, just to be sure.

    # Fetch the merged branch
    echo -e "\e[35m\e[1mgit -C ~${TUE_SYSTEM_DIR#"${DOCKER_HOME}"}/src/$PACKAGE fetch origin $REF_NAME/$PULL_REQUEST/merge:PULLREQUEST\e[0m"
    docker exec -t tue-env bash -c 'source ~/.bashrc; git -C "$TUE_SYSTEM_DIR"/src/"$PACKAGE" fetch origin "$REF_NAME"/"$PULL_REQUEST"/merge:PULLREQUEST'

    # Install the package completely
    branch_string=${BRANCH:+" --try-branch=${BRANCH}"}
    echo -e "\e[35m\e[1mtue-get install ros-$PACKAGE --test-depend${branch_string} --try-branch=PULLREQUEST\e[0m"
    docker exec tue-env bash -c 'source ~/.bashrc; tue-get install ros-"${PACKAGE}" --test-depend --try-branch="${BRANCH}" --try-branch=PULLREQUEST'

    # Checkout -f to be really sure
    echo -e "\e[35m\e[1mgit -C ~${TUE_SYSTEM_DIR#"${DOCKER_HOME}"}/src/$PACKAGE checkout -f PULLREQUEST --\e[0m"
    docker exec -t tue-env bash -c 'source ~/.bashrc; git -C "$TUE_SYSTEM_DIR"/src/"$PACKAGE" checkout -f PULLREQUEST --'
else
    # Install the package
    branch_string=${BRANCH:+" --try-branch=${BRANCH}"}
    echo -e "\e[35m\e[1mtue-get install ros-${PACKAGE} --test-depend${branch_string}\e[0m"
    docker exec tue-env bash -c 'source ~/.bashrc; tue-get install ros-"${PACKAGE}" --test-depend --try-branch="${BRANCH}"'

    # Set the package to the right commit
    echo -e "\e[35m\e[1mReset package to this commit\e[0m"
    echo -e "\e[35m\e[1mgit -C ~${TUE_SYSTEM_DIR#"${DOCKER_HOME}"}/src/$PACKAGE reset --hard $COMMIT\e[0m"
    docker exec -t tue-env bash -c 'source ~/.bashrc; git -C "$TUE_SYSTEM_DIR"/src/"$PACKAGE" reset --hard "$COMMIT"'
fi

# Allow everyone to read ~/.cache/pip folder for caching inside CI pipelines
docker exec -t tue-env bash -c 'sudo chmod -R a+r ~/.cache/pip'
