#! /usr/bin/env bash
#
# Package installer (CI script)
# This script uses the Docker image of tue-env and installs the current git
# repository as a tue-env package using tue-install in the CI

# Stop on errors
set -o errexit

# Execute script only in a CI environment
if [ "$CI" != "true" ]
then
    echo -e "\e[35m\e[1m Error!\e[0m Trying to execute a CI script in a non-CI environment. Exiting script."
    exit 1
fi

# Standard argument parsing, example: install-package --branch=master --package=ros_robot
for i in "$@"
do
    case $i in
        -p=* | --package=* )
            PACKAGE="${i#*=}" ;;

        -b=* | --branch=* )
            BRANCH="${i#*=}" ;;

        -c=* | --commit=* )
            COMMIT="${i#*=}" ;;

        -r=* | --pullrequest=* )
            PULL_REQUEST="${i#*=}" ;;

        -i=* | --image=* )
            IMAGE_NAME="${i#*=}" ;;

        --ssh )
            USE_SSH=true ;;

        --ssh-key=* )
            SSH_KEY="${i#*=}" ;;

        * )
            # unknown option
            echo -e "\e[35m\e[1m Unknown input argument '$i'. Check CI .yml file \e[0m"
            exit 1 ;;
    esac
    shift
done

echo -e "\e[35m\e[1m PACKAGE      = ${PACKAGE} \e[0m"
echo -e "\e[35m\e[1m BRANCH       = ${BRANCH} \e[0m"
echo -e "\e[35m\e[1m COMMIT       = ${COMMIT} \e[0m"
echo -e "\e[35m\e[1m PULL_REQUEST = ${PULL_REQUEST} \e[0m"

# Set default value for IMAGE_NAME
[ -z "$IMAGE_NAME" ] && IMAGE_NAME='tuerobotics/tue-env'
echo -e "\e[35m\e[1m IMAGE_NAME   = ${IMAGE_NAME} \e[0m"

if [ "$USE_SSH" == "true" ]
then
    SSH_KEY_FINGERPRINT=$(ssh-keygen -lf /dev/stdin <<< "$SSH_KEY" | awk '{print $2}')
    echo -e "\e[35m\e[1m SSH_KEY      = ${SSH_KEY_FINGERPRINT} \e[0m"
fi

echo -e "\e[35m\e[1m
This build can be reproduced locally using the following commands:

tue-get install docker
~/.tue/ci/install-package.sh --package=${PACKAGE} --branch=${BRANCH} --commit=${COMMIT} --pullrequest=${PULL_REQUEST}
~/.tue/ci/build-package.sh --package=${PACKAGE}

Optionally fix your compilation errors and re-run only the last command
\e[0m"

# If packages is non-zero, this is a multi-package repo. In multi-package repo, check if this package needs CI.
# If a single-package repo, CI is always needed.
# shellcheck disable=SC2153
if [ -n "$PACKAGES" ] && ! echo "$PACKAGES" | grep -sqw "$PACKAGE"
then
    echo -e "\e[35m\e[1m No changes in this package, so no need to run CI \e[0m"
    exit 0
fi

# Determine docker tag if the same branch exists there
BRANCH_TAG=$(echo "$BRANCH" | tr '[:upper:]' '[:lower:]' | sed -e 's:/:_:g')

# Set the default fallback branch to latest
MASTER_TAG="latest"

# Remove any previously started containers if they exist (if not exist, still return true to let the script continue)
docker stop tue-env  &> /dev/null || true
docker rm tue-env &> /dev/null || true

# Pull the identical branch name from dockerhub if exist, use master as fallback
echo -e "\e[35m\e[1m Trying to fetch docker image: $IMAGE_NAME:$BRANCH_TAG \e[0m"
if ! docker pull "$IMAGE_NAME:$BRANCH_TAG"
then
    echo -e "\e[35m\e[1m No worries, we just test against the master branch: $IMAGE_NAME:$MASTER_TAG \e[0m"
    docker pull "$IMAGE_NAME":"$MASTER_TAG"
    BRANCH_TAG=$MASTER_TAG
fi

if [ -f ~/.ssh/known_hosts ]
then
    MERGE_KNOWN_HOSTS="true"
    DOCKER_MOUNT_KNOWN_HOSTS_ARGS="--mount type=bind,source=$HOME/.ssh/known_hosts,target=/tmp/known_hosts_extra"
fi

# Run the docker image along with setting new environment variables
# shellcheck disable=SC2086
docker run --detach --interactive --tty -e CI="true" -e PACKAGE="$PACKAGE" -e BRANCH="$BRANCH" -e COMMIT="$COMMIT" -e PULL_REQUEST="$PULL_REQUEST" --name tue-env $DOCKER_MOUNT_KNOWN_HOSTS_ARGS "$IMAGE_NAME:$BRANCH_TAG"

if [ "$MERGE_KNOWN_HOSTS" == "true" ]
then
    docker exec -t tue-env bash -c "sudo chown 1000:1000 /tmp/known_hosts_extra && ~/.tue/ci/ssh-merge-known_hosts.py ~/.ssh/known_hosts /tmp/known_hosts_extra --output ~/.ssh/known_hosts"
fi

if [ "$USE_SSH" == "true" ]
then
    docker exec -t tue-env bash -c "eval $(ssh-agent -s)"
    docker exec -t tue-env bash -c "echo '$SSH_KEY' > ~/.ssh/id_rsa && chmod 700 ~/.ssh/id_rsa"
fi

# Use docker environment variables in all exec commands instead of script variables
# Catch the ROS_DISTRO of the docker container
ROS_DISTRO=$(docker exec -t tue-env bash -c 'source ~/.bashrc; echo "$ROS_DISTRO"')
echo -e "\e[35m\e[1m ROS_DISTRO = ${ROS_DISTRO}\e[0m"

TUE_SYSTEM_DIR=$(docker exec -t tue-env bash -c 'source ~/.bashrc; echo "$TUE_SYSTEM_DIR"')
DOCKER_HOME=$(docker exec -t tue-env bash -c 'source ~/.bashrc; echo "$HOME"')

if [[ $PULL_REQUEST != "false" ]]
then
    # Install only the git repo of the package
    echo -e "\e[35m\e[1m tue-get install ros-$PACKAGE --no-ros-deps\e[0m"
    docker exec tue-env bash -c 'source ~/.bashrc; tue-get install ros-"$PACKAGE" --no-ros-deps'

    # Fetch the merge branch
    echo -e "\e[35m\e[1m git -C ~${TUE_SYSTEM_DIR#$DOCKER_HOME}/src/$PACKAGE fetch origin pull/$PULL_REQUEST/merge:PULLREQUEST\e[0m"
    docker exec -t tue-env bash -c 'source ~/.bashrc; git -C "$TUE_SYSTEM_DIR"/src/"$PACKAGE" fetch origin pull/$PULL_REQUEST/merge:PULLREQUEST'

    # Install the package completely
    echo -e "\e[35m\e[1m tue-get install ros-$PACKAGE --test-depend --branch=PULLREQUEST\e[0m"
    docker exec tue-env bash -c 'source ~/.bashrc; tue-get install ros-"$PACKAGE" --test-depend --branch=PULLREQUEST'

    # Checkout -f to be really sure
    echo -e "\e[35m\e[1m git -C ~${TUE_SYSTEM_DIR#$DOCKER_HOME}/src/$PACKAGE checkout -f PULLREQUEST\e[0m"
    docker exec -t tue-env bash -c 'source ~/.bashrc; git -C "$TUE_SYSTEM_DIR"/src/"$PACKAGE" checkout -f PULLREQUEST'
else
    # Install the package
    echo -e "\e[35m\e[1m tue-get install ros-$PACKAGE --test-depend --branch=$BRANCH\e[0m"
    docker exec tue-env bash -c 'source ~/.bashrc; tue-get install ros-"$PACKAGE" --test-depend --branch="$BRANCH"'

    # Set the package to the right commit
    echo -e "\e[35m\e[1m Reset package to this commit\e[0m"
    echo -e "\e[35m\e[1m git -C ~${TUE_SYSTEM_DIR#$DOCKER_HOME}/src/$PACKAGE reset --hard $COMMIT\e[0m"
    docker exec -t tue-env bash -c 'source ~/.bashrc; git -C "$TUE_SYSTEM_DIR"/src/"$PACKAGE" reset --hard "$COMMIT"'
fi
