#! /usr/bin/env bash

# local variables need to be declared before they are used and need to be lowercase
# global variables which needs to be defined outside this script are in uppercase

function conditional_apt_update
{
    local tue_get_apt_get_updated_file
    tue_get_apt_get_updated_file=/tmp/tue_get_apt_get_updated
    if [[ ! -f ${tue_get_apt_get_updated_file} ]]
    then
        echo "[tue-env](bootstrap) sudo apt-get update -qq"
        sudo apt-get update -qq || return 1
        touch ${tue_get_apt_get_updated_file}
    fi
    return 0
}

function installed_or_install
{
    # installed_or_install executable [package]
    # Provide package name if it differs from executable name
    if [[ -z "$1" ]]
    then
        echo "[tue-env](bootstrap) Error! No package name provided to check for installation."
        return 1
    fi
    local executable package
    executable=$1
    package=$1
    [[ -n "$2" ]] && package=$2
    hash "${executable}" 2> /dev/null && return 0
    conditional_apt_update || { echo "[tue-env](bootstrap)Error! Could not update apt-get."; return 1; }
    sudo apt-get install --assume-yes -qq "${package}" || { echo "[tue-env](bootstrap) Error! Could not install ${package}."; return 1; }
    return 0
}

function main
{
    # Make sure curl is installed
    installed_or_install curl
    # Make sure git is installed
    installed_or_install git
    # Make sure lsb-release is installed
    installed_or_install lsb_release lsb-release
    # Make sure python3 is installed
    installed_or_install python3
    # Make sure python3-virtualenv is installed
    installed_or_install virtualenv python3-virtualenv

    # Check if OS is Ubuntu
    local distrib_id distrib_release
    distrib_id="$(lsb_release -si)"
    distrib_release="$(lsb_release -sr)"

    if [[ "${distrib_id}" != "Ubuntu" ]]
    then
        echo "[tue-env](bootstrap) Unsupported OS ${distrib_id}. Use Ubuntu."
        return 1
    fi

    # Initialize variables
    local create_venv targets_repo tue_ros_distro tue_ros_version venv_include_system_site

    # Default values
    create_venv="true"
    venv_include_system_site="true"

    for i in "$@"
    do
        case $i in
            --ros-version=* )
                ros_version="${i#*=}" ;;
            --ros-distro=* )
                ros_distro="${i#*=}" ;;
            --targets-repo=* )
                targets_repo="${i#*=}" ;;
            --create-virtualenv=* )
                create_venv="${i#*=}" ;;
            --virtualenv-include-system-site-packages=* )
                venv_include_system_site="${i#*=}" ;;
            * )
                echo "[tue-env](bootstrap) Error! Unknown argument '${i}' provided to bootstrap script."
                return 1
                ;;
        esac
    done

    case ${distrib_release} in
        "20.04")
            if [[ "${ros_version}" -eq 2 ]]
            then
                tue_ros_version=2
                if [[ "${ros_distro}" == "foxy" ]]
                then
                    tue_ros_distro="foxy"
                elif [[ "${ros_distro}" == "galactic" ]]
                then
                    tue_ros_distro="galactic"
                elif [[ "${ros_distro}" == "rolling" ]]
                then
                    tue_ros_distro="rolling"
                elif [[ -n "${ros_distro}" ]]
                then
                    echo "[tue-env](bootstrap) Error! ROS ${ros_distro} is unsupported with tue-env."
                    return 1
                else
                    tue_ros_distro="galactic"
                    echo "[tue-env](bootstrap) Using default ROS_DISTRO '${tue_ros_distro}' with ROS_VERSION '${tue_ros_version}'"
                fi
            elif [[ "${ros_version}" -eq 1 ]]
            then
                tue_ros_distro="noetic"
                tue_ros_version=1
            elif [[ -n "${ros_version}" ]]
            then
                echo "[tue-env](bootstrap) Error! ROS ${ros_version} is unsupported with tue-env."
                return 1
            else
                tue_ros_distro="noetic"
                tue_ros_version=1
                echo "[tue-env](bootstrap) Using default ROS_DISTRO '${tue_ros_distro}' with ROS_VERSION '${tue_ros_version}'"
            fi
            ;;
        "22.04")
            if [[ -n "${ros_version}" ]] && [[ "${ros_version}" -ne 2 ]]
            then
                 echo "[tue-env](bootstrap) Error! Only ROS version 2 is supported with ubuntu 22.04 and newer"
                 return 1
            fi
            tue_ros_version=2

            if [[ "${ros_distro}" == "humble" ]]
            then
                tue_ros_distro="humble"
            elif [[ "${ros_distro}" == "rolling" ]]
            then
                tue_ros_distro="rolling"
            elif [[ -n "${ros_distro}" ]]
            then
                echo "[tue-env](bootstrap) Error! ROS ${ros_distro} is unsupported with tue-env."
                return 1
            else
                tue_ros_distro="humble"
                echo "[tue-env](bootstrap) Using default ROS_DISTRO '${tue_ros_distro}' with ROS_VERSION '${tue_ros_version}'"
            fi
            ;;
        "24.04")
            if [[ -n "${ros_version}" ]] && [[ "${ros_version}" -ne 2 ]]
            then
                 echo "[tue-env](bootstrap) Error! Only ROS version 2 is supported with ubuntu 22.04 and newer"
                 return 1
            fi
            tue_ros_version=2

            if [[ "${ros_distro}" == "jazzy" ]]
            then
                tue_ros_distro="jazzy"
            elif [[ "${ros_distro}" == "rolling" ]]
            then
                tue_ros_distro="rolling"
            elif [[ -n "${ros_distro}" ]]
            then
                echo "[tue-env](bootstrap) Error! ROS ${ros_distro} is unsupported with tue-env."
                return 1
            else
                tue_ros_distro="jazzy"
                echo "[tue-env](bootstrap) Using default ROS_DISTRO '${tue_ros_distro}' with ROS_VERSION '${tue_ros_version}'"
            fi
            ;;
        *)
            echo "[tue-env](bootstrap) Ubuntu ${distrib_release} is unsupported. Please use one of Ubuntu 20.04, 22.04 or 24.04."
            return 1
            ;;
    esac

    # Script variables
    local env_url env_targets_url env_dir workspace workspace_dir
    env_url="https://github.com/tue-robotics/tue-env.git"
    { [[ -n "${targets_repo}" ]] && env_targets_url="${targets_repo}"; } || env_targets_url="https://github.com/tue-robotics/tue-env-targets.git"
    [[ -n "${create_virtualenv}" ]] || create_virtualenv="true"
    env_dir="${HOME}/.tue"
    workspace="ros-${tue_ros_distro}"
    workspace_dir="${HOME}/ros/${tue_ros_distro}"

    # Move old environments and installer
    if [[ -d "${env_dir}" ]] && [[ -z "${CI}" ]]
    then
        local files date_now
        files=$(find "${env_dir}"/user/envs -maxdepth 1 -type f)
        date_now=$(date +%F_%R)
        for env in ${files}
        do
            mv -f "$(cat "${env}")" "$(cat "${env}")"."${date_now}"
        done
        mv -f "${env_dir}" "${env_dir}"."${date_now}"
    fi

    # If in CI with Docker, then clone tue-env with BRANCH when not testing a PR
    if [[ "${CI}" == "true" ]] && [[ "${DOCKER}" == "true" ]]
    then
        # Docker has a default value as false for PULL_REQUEST
        if [[ "${PULL_REQUEST}" == "false" ]]
        then
            if [[ -n "${COMMIT}" ]]
            then
                if [[ -n "${BRANCH}" ]]
                then
                    echo "[tue-env](bootstrap) Cloning tue-env repository with branch: ${BRANCH} at commit: ${COMMIT}"
                    git clone -q --single-branch --branch "${BRANCH}" "${env_url}" "${env_dir}"
                else
                    echo "[tue-env](bootstrap) Cloning tue-env repository with default branch at commit: ${COMMIT}"
                    git clone -q --single-branch "${env_url}" "${env_dir}"
                fi
                git -C "${env_dir}" reset --hard "${COMMIT}"
            else
                echo "[tue-env](bootstrap) Error! CI branch or commit is unset"
                return 1
            fi
        else
            echo "[tue-env](bootstrap) Testing Pull Request"
            [[ -z "${REF_NAME}" ]] && { echo "[tue-env](bootstrap) Error! Environment variable REF_NAME is not set."; return 1; }

            git clone -q --depth=10 "${env_url}" "${env_dir}"
            git -C "${env_dir}" fetch origin "${REF_NAME}"/"${PULL_REQUEST}"/merge:PULLREQUEST || { echo "[tue-env](bootstrap) Error! Could not fetch refs"; return 1; }
            git -C "${env_dir}" checkout PULLREQUEST
        fi
    else
        # Update installer
        echo "[tue-env](bootstrap) Cloning tue-env repository"
        git clone "${env_url}" "${env_dir}"
    fi

    # Source the installer commands
    # No need to follow to a file which is already checked by CI
    # shellcheck disable=SC1090
    source "${env_dir}"/setup.bash

    # Create ros environment directory
    mkdir -p "${workspace_dir}"

    # Initialize ros environment directory incl. targets
    tue-env init "${workspace}" "${workspace_dir}" \
    "--create-virtualenv=${create_venv}" \
    "--virtualenv-include-system-site-packages=${venv_include_system_site}" \
    "--targets-url=${env_targets_url}"

    # Configure environment
    tue-env config "${workspace}" set "TUE_ROS_DISTRO" "${tue_ros_distro}"
    tue-env config "${workspace}" set "TUE_ROS_VERSION" "${tue_ros_version}"

    # Add loading of TU/e tools (tue-env, tue-get, etc) to bashrc
    # shellcheck disable=SC2088
    if ! grep -q "${env_dir}/setup.bash" ~/.bashrc
    then
        echo "
# Load TU/e tools
source ${env_dir}/setup.bash" >> ~/.bashrc
    fi

    # Set this environment as default
    tue-env set-default "${workspace}"

    # Activate the default environment
    # shellcheck disable=SC1090
    source "${env_dir}"/setup.bash
}

main "$@" || echo "[tue-env](bootstrap) Error! Could not install tue-env."
