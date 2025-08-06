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

function check_available_version
{
    # check_available_version package
    if [[ -z "$1" ]]
    then
        echo "[tue-env](bootstrap) Error! No package name provided to check for available version." >&2
        return 1
    fi
    local package
    package=$1

    # Get the apt-cache policy output
    local output
    output=$(apt-cache policy "${package}" 2>/dev/null)

    # Extract the candidate version
    local candidate_version
    candidate_version=$(echo "${output}" | awk '/Candidate:/ {print $2}')

    if [[ -z "${candidate_version}" || "${candidate_version}" == "(none)" ]]
    then
      echo "[tue-env](bootstrap) No candidate version found for package '${package}'" >&2
      return 1
    fi

    # Extract major.minor.patch part (e.g., 1.2.3 or 1.2)
    local parsed_version
    parsed_version=$(echo "${candidate_version}" | grep -oE '[0-9]+\.[0-9]+(\.[0-9]+)?' | head -n1)

    echo "${parsed_version}"
    return 0
}

function check_installed_version
{
    # check_installed_version package
    if [[ -z "$1" ]]
    then
        echo "[tue-env](bootstrap) Error! No package name provided to check for installed version." >&2
        return 1
    fi
    local package
    package=$1

    # Extract the raw version
    local raw_version
    raw_version=$(dpkg-query -f '${version}' -W "${package}" 2>/dev/null) || { echo "[tue-env](bootstrap) Error! Could not query installed version for package '${package}'." >&2; return 1; }
    if [[ -z "${raw_version}" ]]
    then
        echo "[tue-env](bootstrap) No installed version found for package '${package}'" >&2
        return 1
    fi

    # Extract major.minor.patch part (e.g., 1.2.3 or 1.2)
    local parsed_version
    parsed_version=$(echo "${raw_version}" | grep -oE '[0-9]+\.[0-9]+(\.[0-9]+)?' | head -n1)

    echo "${parsed_version}"
    return 0
}


function compare_version
{
    # compare_version version requirement
    # Example usage: compare_version 1.2.3 ">=1.2.0"
    if [[ -z "$2" ]]
    then
        echo "[tue-env](bootstrap) Error! Version and requirement must be provided." >&2
        return 1
    fi

    local version requirement dpkg_op req_version

    version=$1
    requirement=$2

    # Check if the version is a valid semantic version
    if ! [[ "${version}" =~ ^[0-9]+\.[0-9]+(\.[0-9]+)?$ ]]
    then
        echo "[tue-env](bootstrap) Error! Invalid version format: ${version}" >&2
        return 1
    fi

    # Parse requirement
    operator="${requirement%%[0-9]*}" # Extracts comparison operator
    req_version="${requirement#"${operator}"}" # Extracts required version

    # Map operators for dpkg
    case "${operator}" in
        '>=')
            dpkg_op="ge" ;;
        '<=')
            dpkg_op="le" ;;
        '>')
            dpkg_op="gt" ;;
        '<')
            dpkg_op="lt" ;;
        '=')
            dpkg_op="eq" ;;
        '==')
            dpkg_op="eq" ;;
        '!=')
            dpkg_op="ne" ;;
        * )
            echo "[tue-env](bootstrap) Error! Unknown operator ${operator}" >&2; return 1 ;;
    esac

# Compare using dpkg --compare-versions
if dpkg --compare-versions "${version}" "${dpkg_op}" "${req_version}"
then
    return 0
else
    return 1
fi
}

function installed_or_install
{
    # installed_or_install executable [package] [--version=VERSION]
    # Provide package name if it differs from executable name
    if [[ -z "$1" ]]
    then
        echo "[tue-env](bootstrap) Error! No package name provided to check for installation." >&2
        return 1
    fi
    local executable package version_requirement

    if [[ -n $1 ]]
    then
        for i in "$@"
        do
            case $i in
                --version=* )
                    version_requirement="${i#*=}" ;;
                * )
                    if [[ -z ${executable} ]]
                    then
                        executable=${i}
                        package=${i}
                    elif [[ ${executable} == "${package}" ]]
                    then
                        package=${i}
                    else
                        echo "[tue-env](bootstrap) Error! No Unknown input variable ${i}" >&2; return 1
                    fi
                    ;;
            esac
        done
    fi

    if [[ -n "${version_requirement}" ]]
    then
        local installed_version
        installed_version=$(check_installed_version "${package}")
        if [[ -n "${installed_version}" ]] && compare_version "${installed_version}" "${version_requirement}"
        then
            echo "[tue-env](bootstrap) ${package} is already installed with version ${installed_version}."
            return 0
        else
            conditional_apt_update || { echo "[tue-env](bootstrap) Error! Could not update apt-get." >&2; return 1; }
            local available_version
            available_version=$(check_available_version "${package}") || { echo "[tue-env](bootstrap) Error! Could not check available version for ${package}." >&2; return 1; }
            if ! compare_version "${available_version}" "${version_requirement}"
            then
                echo "[tue-env](bootstrap) Error! Available version ${available_version} does not satisfy requirement ${version_requirement} for package ${package}." >&2
                return 1
            fi
        fi
    else
        hash "${executable}" 2> /dev/null && echo "[tue-env](bootstrap) ${package} is already installed." && return 0
    fi

    conditional_apt_update || { echo "[tue-env](bootstrap) Error! Could not update apt-get." >&2; return 1; }
    sudo apt-get install --assume-yes -qq "${package}" || { echo "[tue-env](bootstrap) Error! Could not install ${package}." >&2; return 1; }
    return 0
}

function check_python_package_installed
{
    # Check whether a python package is installed using importlib.
    # Printing the package version if installed}
    if [[ -z "$1" ]]
    then
        echo "[tue-env](bootstrap) Error! No python package name provided to check for installation."
        return 1
    fi
    local package
    package=$1
    local installed_version
    installed_version=$(/usr/bin/python3 -c "import importlib.metadata; print(importlib.metadata.version('${package}'))" 2>/dev/null)
    if [[ -n "${installed_version}" ]]
    then
        echo "${installed_version}"
        return 0
    else
        echo "[tue-env](bootstrap) Python package '${package}' is not installed." >&2
        return 1
    fi
}

function python_install_desired_version
{
    # python_install_desired_version package [version_requirement]
    if [[ -z "$1" ]]
    then
        echo "[tue-env](bootstrap) Error! No python package name provided to check for installation." >&2
        return 1
    fi
    local package version_requirement
    package=$1
    [[ -n "$2" ]] && version_requirement=$2

    # First check if the package is already installed, otherwise try to install it via apt, only when it satisfies the version requirement
    local installed_version
    installed_version=$(check_python_package_installed "${package}")

    if [[ -n "${installed_version}" && -n "${version_requirement}" ]]
    then
        # If a version requirement is specified, check if the installed version satisfies it
        if compare_version "${installed_version}" "${version_requirement}"
        then
            echo "${package}${version_requirement}: ${installed_version}"
            return 0
        fi
        # Check if an available version satisfies the requirement
    elif [[ -n "${installed_version}" ]]
    then
        # If no version requirement is specified, just return the installed version
        echo "${package}: ${installed_version}"
        return 0
    fi

    # If we reach here, the package is not installed or does not satisfy the version requirement
    installed_or_install "${package}" "python3-${package}" --version="${version_requirement}" && return 0

    # If the package is not available via apt, try to install it via pip
    # Make sure pip is installed
    installed_or_install pip python3-pip
    # Install the package
    PIP_BREAK_SYSTEM_PACKAGES=1 /usr/bin/python3 -m pip install --user "${package}${version_requirement}" || { echo "[tue-env](bootstrap) Error! Could not install '${package}${version_requirement}'." >&2; return 1; }
    return 0
}

function file_exist_or_install
{
    if [[ -z "$1" ]]
    then
        echo "[tue-env](bootstrap) Error! No file name provided to check for installation." >&2
        return 1
    fi
    if [[ -z "$2" ]]
    then
        echo "[tue-env](bootstrap) Error! No package name provided to install in case needed." >&2
        return 1
    fi
    local file_name package
    file_name=$1
    package=$2
    [[ -f ${file_name} ]] && return 0
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
    # Make sure distro-data-info is installed
    file_exist_or_install /usr/share/distro-info/ubuntu.csv distro-info-data
    # Make sure python3 is installed
    installed_or_install python3
    # Make sure python3-virtualenv is installed
    python_install_desired_version virtualenv ">=20.24.0"

    # Check if OS is Ubuntu
    local distrib_id distrib_release
    distrib_id="$(lsb_release -si)"
    distrib_release="$(lsb_release -sr)"

    if [[ "${distrib_id}" != "Ubuntu" ]]
    then
        echo "[tue-env](bootstrap) Unsupported OS ${distrib_id}. Use Ubuntu." >&2
        return 1
    fi

    # Initialize variables
    local create_venv targets_repo tue_env_ros_distro tue_env_ros_version venv_include_system_site

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
                echo "[tue-env](bootstrap) Error! Unknown argument '${i}' provided to bootstrap script." >&2
                return 1
                ;;
        esac
    done

    declare -A ros2_distribution_map
    ros2_distribution_map["20.04"]="galactic"
    ros2_distribution_map["22.04"]="humble"
    ros2_distribution_map["24.04"]="jazzy"

    case ${distrib_release} in
        "20.04")
            if [[ "${ros_version}" -eq 2 ]]
            then
                tue_env_ros_version=2
            if [[ "${ros_distro}" == "foxy" || "${ros_distro}" == "${ros2_distribution_map[${distrib_release}]}" || "${ros_distro}" == "rolling" ]]
            then
                tue_env_ros_distro="${ros_distro}"
                elif [[ -n "${ros_distro}" ]]
                then
                    echo "[tue-env](bootstrap) Error! ROS ${ros_distro} is unsupported with tue-env." >&2
                    return 1
                else
                    tue_env_ros_distro="${ros2_distribution_map[${distrib_release}]}"
                    echo "[tue-env](bootstrap) Using default ROS_DISTRO '${tue_env_ros_distro}' with ROS_VERSION '${tue_env_ros_version}'"
                fi
            elif [[ "${ros_version}" -eq 1 ]]
            then
                tue_env_ros_distro="noetic"
                tue_env_ros_version=1
            elif [[ -n "${ros_version}" ]]
            then
                echo "[tue-env](bootstrap) Error! ROS ${ros_version} is unsupported with tue-env." >&2
                return 1
            else
                tue_env_ros_distro="noetic"
                tue_env_ros_version=1
                echo "[tue-env](bootstrap) Using default ROS_DISTRO '${tue_env_ros_distro}' with ROS_VERSION '${tue_env_ros_version}'"
            fi
            ;;
        "22.04" | "24.04")
            if [[ -n "${ros_version}" ]] && [[ "${ros_version}" -ne 2 ]]
            then
                 echo "[tue-env](bootstrap) Error! Only ROS version 2 is supported with ubuntu 22.04 and newer" >&2
                 return 1
            fi
            tue_env_ros_version=2

            if [[ "${ros_distro}" == "${ros2_distribution_map[${distrib_release}]}" || "${ros_distro}" == "rolling" ]]
            then
                tue_env_ros_distro="${ros_distro}"
            elif [[ -n "${ros_distro}" ]]
            then
                echo "[tue-env](bootstrap) Error! ROS ${ros_distro} is unsupported with tue-env." >&2
                return 1
            else
                tue_env_ros_distro="${ros2_distribution_map[${distrib_release}]}"
                echo "[tue-env](bootstrap) Using default ROS_DISTRO '${tue_env_ros_distro}' with ROS_VERSION '${tue_env_ros_version}'"
            fi
            ;;
        *)
            echo "[tue-env](bootstrap) Ubuntu ${distrib_release} is unsupported. Please use one of Ubuntu 20.04, 22.04 or 24.04." >&2
            return 1
            ;;
    esac

    # Script variables
    local env_url env_targets_url env_dir workspace workspace_dir
    env_url="https://github.com/tue-robotics/tue-env.git"
    { [[ -n "${targets_repo}" ]] && env_targets_url="${targets_repo}"; } || env_targets_url="https://github.com/tue-robotics/tue-env-targets.git"
    [[ -n "${create_virtualenv}" ]] || create_virtualenv="true"
    env_dir="${HOME}/.tue"
    workspace="ros-${tue_env_ros_distro}"
    workspace_dir="${HOME}/ros/${tue_env_ros_distro}"

    # Move old environments and installer
    if [[ -d "${env_dir}" ]] && [[ -z "${CI}" ]]
    then
        local files date_now
        files=$(find "${env_dir}"/user/envs -maxdepth 1 -type f)
        date_now=$(date +%F_%H%M%S)
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
                echo "[tue-env](bootstrap) Error! CI branch or commit is unset" >&2
                return 1
            fi
        else
            echo "[tue-env](bootstrap) Testing Pull Request"
            [[ -z "${REF_NAME}" ]] && { echo "[tue-env](bootstrap) Error! Environment variable REF_NAME is not set." >&2; return 1; }

            git clone -q --depth=10 "${env_url}" "${env_dir}"
            git -C "${env_dir}" fetch origin "${REF_NAME}"/"${PULL_REQUEST}"/merge:PULLREQUEST || { echo "[tue-env](bootstrap) Error! Could not fetch refs" >&2; return 1; }
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
    tue-env config "${workspace}" set "TUE_ENV_ROS_DISTRO" "${tue_env_ros_distro}"
    tue-env config "${workspace}" set "TUE_ENV_ROS_VERSION" "${tue_env_ros_version}"

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

main "$@" || echo "[tue-env](bootstrap) Error! Could not install tue-env." >&2
