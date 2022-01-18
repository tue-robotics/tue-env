#! /usr/bin/env bash

function _function_test
{
    local function_missing="false"
    # shellcheck disable=SC2048
    for func in $*
    do
        declare -f "$func" > /dev/null || { echo -e "\e[38;1mFunction '$func' missing, resource the setup\e[0m" && function_missing="true"; }
    done
    [[ "$function_missing" == "true" ]] && exit 1
}

_function_test _git_https_or_ssh _git_split_url

TUE_INSTALL_DEPENDENCIES_DIR=$TUE_ENV_DIR/.env/dependencies
TUE_INSTALL_DEPENDENCIES_ON_DIR=$TUE_ENV_DIR/.env/dependencies-on
TUE_INSTALL_INSTALLED_DIR=$TUE_ENV_DIR/.env/installed

mkdir -p "$TUE_INSTALL_DEPENDENCIES_DIR"
mkdir -p "$TUE_INSTALL_DEPENDENCIES_ON_DIR"
mkdir -p "$TUE_INSTALL_INSTALLED_DIR"

TUE_INSTALL_TARGETS_DIR=$TUE_ENV_TARGETS_DIR

TUE_REPOS_DIR=$TUE_ENV_DIR/repos

TUE_APT_GET_UPDATED_FILE=/tmp/tue_get_apt_get_updated

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function date_stamp
{
    date +%Y_%m_%d_%H_%M_%S
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function version_gt()
{
    test "$(printf '%s\n' "$@" | sort -V | head -n 1)" != "$1";
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-error
{
    echo -e "\e[31m
Error while installing target '$TUE_INSTALL_CURRENT_TARGET':

    $1

Logfile: $INSTALL_DETAILS_FILE
\e[0m" | tee --append "$INSTALL_DETAILS_FILE"
    exit 1
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-warning
{
    echo -e "\e[33;1m[$TUE_INSTALL_CURRENT_TARGET] WARNING: $*\e[0m" | tee --append "$INSTALL_DETAILS_FILE"
    TUE_INSTALL_WARNINGS="    [$TUE_INSTALL_CURRENT_TARGET] $*\n${TUE_INSTALL_WARNINGS}"
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-info
{
    echo -e "\e[0;36m[$TUE_INSTALL_CURRENT_TARGET] INFO: $*\e[0m"  | tee --append "$INSTALL_DETAILS_FILE"
    TUE_INSTALL_INFOS="    [$TUE_INSTALL_CURRENT_TARGET] $*\n${TUE_INSTALL_INFOS}"
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-debug
{
    if [ "$DEBUG" == "true" ]
    then
        echo -e "\e[0;34m[$TUE_INSTALL_CURRENT_TARGET] DEBUG: $*\e[0m"  | tee --append "$INSTALL_DETAILS_FILE"
    else
        echo -e "\e[0;34m[$TUE_INSTALL_CURRENT_TARGET] DEBUG: $*\e[0m"  | tee --append "$INSTALL_DETAILS_FILE" 1> /dev/null
    fi
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-echo
{
    echo -e "\e[0;1m[$TUE_INSTALL_CURRENT_TARGET]: $*\e[0m" | tee --append "$INSTALL_DETAILS_FILE"
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-tee
{
    echo -e "$*" | tee --append "$INSTALL_DETAILS_FILE"
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-pipe
{
    local pipefail_old
    pipefail_old=$(set -o | grep pipefail | awk '{printf $2}')
    [ "$pipefail_old" != "on" ] && set -o pipefail # set pipefail if not yet set
    echo -e "\e[0;1m[$TUE_INSTALL_CURRENT_TARGET]: $*\e[0m" | tee --append "$INSTALL_DETAILS_FILE"
    # Executes the command (all arguments), catch stdout and stderr, red styled, print them directly and to file
    "$@" 2> >(sed $'s,.*,\e[31m&\e[m,'>&1) | tee --append "$INSTALL_DETAILS_FILE"
    local return_code=$?
    [ "$pipefail_old" != "on" ] && set +o pipefail # restore old pipefail setting
    return $return_code
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-target-now
{
    tue-install-debug "tue-install-target-now $*"

    local target=$1

    tue-install-debug "calling: tue-install-target $target true"
    tue-install-target "$target" "true"
    return $?
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-target
{
    tue-install-debug "tue-install-target $*"

    local target=$1
    local now=$2

    tue-install-debug "Installing target: $target"

    # Check if valid target received as input
    if [ ! -d "$TUE_INSTALL_TARGETS_DIR"/"$target" ]
    then
        tue-install-debug "Target '$target' does not exist."
        return 1
    fi

    local parent_target=$TUE_INSTALL_CURRENT_TARGET
    TUE_INSTALL_CURRENT_TARGET_DIR=$TUE_INSTALL_TARGETS_DIR/$target
    TUE_INSTALL_CURRENT_TARGET=$target

    # If the target has a parent target, add target as a dependency to the parent target
    if [ -n "$parent_target" ] && [ "$parent_target" != "main-loop" ]
    then
        if [ "$parent_target" != "$target" ]
        then
            echo "$target" >> "$TUE_INSTALL_DEPENDENCIES_DIR"/"$parent_target"
            echo "$parent_target" >> "$TUE_INSTALL_DEPENDENCIES_ON_DIR"/"$target"
            sort "$TUE_INSTALL_DEPENDENCIES_DIR"/"$parent_target" -u -o "$TUE_INSTALL_DEPENDENCIES_DIR"/"$parent_target"
            sort "$TUE_INSTALL_DEPENDENCIES_ON_DIR"/"$target" -u -o "$TUE_INSTALL_DEPENDENCIES_ON_DIR"/"$target"
        fi
    fi

    local state_file="$TUE_INSTALL_STATE_DIR"/"$target"
    local state_file_now="${state_file}-now"

    # Determine if this target needs to be executed
    local execution_needed="true"

    if [[ "$CI" == "true" ]] && [[ -f "$TUE_INSTALL_CURRENT_TARGET_DIR"/.ci_ignore ]]
    then
        tue-install-debug "Running installer in CI mode and file $TUE_INSTALL_CURRENT_TARGET_DIR/.ci_ignore exists. No execution is needed"
        execution_needed="false"
    elif [ -f "$state_file_now" ]
    then
        tue-install-debug "File $state_file_now does exist, so installation has already been executed with 'now' option. No execution is needed"
        execution_needed="false"
    elif [ -f "$state_file" ]
    then
        if [ "$now" == "true" ]
        then
            tue-install-debug "File $state_file_now doesn't exist, but file $state_file does. So installation has been executed yet, but not with the 'now' option. Going to execute it with 'now' option."
        else
            tue-install-debug "File $state_file_now does exist. 'now' is not enabled, so no execution needed."
            execution_needed="false"
        fi
    else
        if [ "$now" == "true" ]
        then
            tue-install-debug "Files $state_file_now and $state_file don't exist. Going to execute with 'now' option."
        else
            tue-install-debug "Files $state_file_now and $state_file don't exist. Going to execute without 'now' option."
        fi
    fi

    if [ "$execution_needed" == "true" ]
    then
        tue-install-debug "Starting installation"

        local install_file=$TUE_INSTALL_CURRENT_TARGET_DIR/install

        # Empty the target's dependency file
        tue-install-debug "Emptying $TUE_INSTALL_DEPENDENCIES_DIR/$target"
        truncate -s 0 "$TUE_INSTALL_DEPENDENCIES_DIR"/"$target"
        local target_processed=false

        if [ -f "$install_file".yaml ]
        then
            if [[ "$CI" == "true" ]] && [[ -f "$TUE_INSTALL_CURRENT_TARGET_DIR"/.ci_ignore_yaml ]]
            then
                tue-install-debug "Running in CI mode and found .ci_ignore_yaml file, so skipping install.yaml"
                target_processed=true
            else
                tue-install-debug "Parsing $install_file.yaml"
                # Do not use 'local cmds=' because it does not preserve command output status ($?)
                local now_cmd=""
                [ "$now" == "true" ] && now_cmd="--now"
                # Do not use 'local cmds=' because it does not preserve command output status ($?)
                local cmds
                if cmds=$("$TUE_INSTALL_SCRIPTS_DIR"/parse-install-yaml.py "$install_file".yaml $now_cmd)
                then
                    for cmd in $cmds
                    do
                        # Don't use tue-install-pipe here. As we are calling other tue-install functions, which already
                        # implement tue-install-pipe for their external calls
                        tue-install-debug "Running following command: ${cmd//^/ }"
                        ${cmd//^/ } || tue-install-error "Error while running: ${cmd//^/ }"
                        tue-install-debug "Done: Running following command: ${cmd//^/ }"
                    done
                    target_processed=true
                else
                    tue-install-error "Invalid install.yaml: $cmds"
                fi
            fi
        fi

        if [ -f "$install_file".bash ]
        then
            if [[ "$CI" == "true" ]] && [[ -f "$TUE_INSTALL_CURRENT_TARGET_DIR"/.ci_ignore_bash ]]
            then
                tue-install-debug "Running in CI mode and found .ci_ignore_bash file, so skipping install.bash"
            else
                tue-install-debug "Sourcing $install_file.bash"
                # shellcheck disable=SC1090
                source "$install_file".bash
                tue-install-debug "Done: Sourcing $install_file.bash"
            fi
            target_processed=true
        fi

        if [ "$target_processed" == false ]
        then
            tue-install-warning "Target $target does not contain a valid install.yaml/bash file"
        fi

        if [ "$now" == "true" ]
        then
            touch "$state_file_now"
        else
            touch "$state_file"
        fi

    fi

    TUE_INSTALL_CURRENT_TARGET=$parent_target
    TUE_INSTALL_CURRENT_TARGET_DIR=$TUE_INSTALL_TARGETS_DIR/$parent_target

    tue-install-debug "Finished installing $target"
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function _show_update_message
{
    # shellcheck disable=SC2086,SC2116
    if [ -n "$(echo $2)" ]
    then
        tue-install-tee "\n    \e[1m$1\e[0m"
        tue-install-tee "--------------------------------------------------"
        tue-install-tee "$2"
        tue-install-tee "--------------------------------------------------"
        tue-install-tee ""
    else
        tue-install-tee "\e[1m$1\e[0m: up-to-date"
    fi
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function _try_branch_git
{
    tue-install-debug "_try_branch_git $*"

    if [ -z "$2" ]
    then
        tue-install-error "Invalid _try_branch_git: needs two arguments (repo and branch)."
    fi

    local repo="$1"
    local branch="$2"
    tue-install-debug "git -C $repo checkout $branch --"
    _try_branch_res=$(git -C "$repo" checkout "$branch" -- 2>&1)  # _try_branch_res is a "global" variable from tue-install-git
    tue-install-debug "_try_branch_res: $_try_branch_res"

    local _submodule_sync_res _submodule_sync_error_code
    tue-install-debug "git -C $repo submodule sync --recursive"
    _submodule_sync_res=$(git -C "$repo" submodule sync --recursive 2>&1)
    _submodule_sync_error_code=$?
    tue-install-debug "_submodule_sync_res: $_submodule_sync_res"

    local _submodule_res
    tue-install-debug "git -C $repo submodule update --init --recursive"
    _submodule_res=$(git -C "$repo" submodule update --init --recursive 2>&1)
    tue-install-debug "_submodule_res: $_submodule_res"

    if [[ $_try_branch_res == "Already on "* || $_try_branch_res == "fatal: invalid reference:"* ]]
    then
        _try_branch_res=
    fi
    [ "$_submodule_sync_error_code" -gt 0 ] && [ -n "$_submodule_sync_res" ] && _try_branch_res="${res:+${res}\n}$_submodule_sync_res"
    [ -n "$_submodule_res" ] && _try_branch_res="${_try_branch_res:+${_try_branch_res}\n}$_submodule_res"
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-git
{
    tue-install-debug "tue-install-git $*"

    local repo=$1
    local repo_pre="$repo"
    local targetdir=$2
    local version=$3

    # Change url to https/ssh
    repo=$(_git_https_or_ssh "$repo")
    if ! grep -q "^git@.*\.git$\|^https://.*\.git$" <<< "$repo"
    then
        # shellcheck disable=SC2140
        tue-install-error "repo: '$repo' is invalid. It is generated from: '$repo_pre'\n"\
"The problem will probably be solved by resourcing the setup"
    fi

    if [ ! -d "$targetdir" ]
    then
        tue-install-debug "git clone --recursive $repo $targetdir"
        res=$(git clone --recursive "$repo" "$targetdir" 2>&1)
        TUE_INSTALL_GIT_PULL_Q+=:$targetdir:
    else
        # Check if we have already pulled the repo
        if [[ $TUE_INSTALL_GIT_PULL_Q == *:$targetdir:* ]]
        then
            tue-install-debug "Repo previously pulled, skipping"
            # We have already pulled this repo, skip it
            res=
        else
            # Switch url of origin to use https/ssh if different
            # Get current remote url
            local current_url
            current_url=$(git -C "$targetdir" config --get remote.origin.url)

            # If different, switch url
            if [ "$current_url" != "$repo" ]
            then
                tue-install-pipe git -C "$targetdir" remote set-url origin "$repo" || tue-install-error "Could not change git url of '$targetdir' to '$repo'"
                tue-install-info "URL has switched to $repo"
            fi

            local res
            tue-install-debug "git -C $targetdir pull --ff-only --prune"
            res=$(git -C "$targetdir" pull --ff-only --prune 2>&1)
            tue-install-debug "res: $res"

            TUE_INSTALL_GIT_PULL_Q+=:$targetdir:

            local submodule_sync_res submodule_sync_error_code
            tue-install-debug "git -C $targetdir submodule sync --recursive"
            submodule_sync_res=$(git -C "$targetdir" submodule sync --recursive)
            submodule_sync_error_code=$?
            tue-install-debug "submodule_sync_res: $submodule_sync_res"
            [ "$submodule_sync_error_code" -gt 0 ] && [ -n "$submodule_sync_res" ] && res="${res:+${res}\n}$submodule_sync_res"

            local submodule_res
            tue-install-debug "git -C $targetdir submodule update --init --recursive"
            submodule_res=$(git -C "$targetdir" submodule update --init --recursive 2>&1)
            tue-install-debug "submodule_res: $submodule_res"
            [ -n "$submodule_res" ] && res="${res:+${res}\n}$submodule_res"

            if [ "$res" == "Already up to date." ]
            then
                res=
            fi
        fi
    fi

    tue-install-debug "Desired version: $version"
    local _try_branch_res # Will be used in _try_branch_git
    local version_cache_file="$TUE_ENV_DIR/.env/version_cache/$targetdir"
    if [ -n "$version" ]
    then
        mkdir -p "$(dirname "$version_cache_file")"
        echo "$version" > "$version_cache_file"
        _try_branch_res=""
        _try_branch_git "$targetdir" "$version"
        [ -n "$_try_branch_res" ] && res="${res:+${res}\n}$_try_branch_res"
    else
        rm "$version_cache_file" 2>/dev/null
    fi

    tue-install-debug "Desired branch: $BRANCH"
    if [ -n "$BRANCH" ] # Cannot be combined with version-if because this one might not exist
    then
        _try_branch_res=""
        _try_branch_git "$targetdir" "$BRANCH"
        [ -n "$_try_branch_res" ] && res="${res:+${res}\n}$_try_branch_res"
    fi

    _show_update_message "$TUE_INSTALL_CURRENT_TARGET" "$res"
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-apply-patch
{
    tue-install-debug "tue-install-apply-patch $*"

    if [ -z "$1" ]
    then
        tue-install-error "Invalid tue-install-apply-patch call: needs patch file as argument."
    fi

    if [ -z "$TUE_INSTALL_PKG_DIR" ]
    then
        tue-install-error "Invalid tue-install-apply-patch call: package directory is unknown."
    fi

    patch_file=$TUE_INSTALL_CURRENT_TARGET_DIR/$1

    if [ ! -f "$patch_file" ]
    then
        tue-install-error "Invalid tue-install-apply-patch call: patch file '$1' does not exist."
    fi

    patch -s -N -r - -p0 -d "$TUE_INSTALL_PKG_DIR" < "$patch_file"
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-cp
{
    tue-install-debug "tue-install-cp $*"

    if [ -z "$2" ]
    then
        tue-install-error "Invalid tue-install-cp call: needs two arguments (source and target). The source must be relative to the installer target directory
Command: tue-install-cp $*"
    fi

    local source_files="$TUE_INSTALL_CURRENT_TARGET_DIR"/"$1"

    # Check if user is allowed to write on target destination
    local root_required=true
    if namei -l "$2" | grep -q "$(whoami)"
    then
        root_required=false
    fi

    local cp_target=
    local cp_target_parent_dir=

    if [ -d "$2" ]
    then
        cp_target_parent_dir="${2%%/}"
    else
        cp_target_parent_dir="$(dirname "$2")"
    fi

    for file in $source_files
    do
        if [ ! -f "$file" ]
        then
            tue-install-error "Invalid tue-install-cp call: file '$file' does not exist."
        fi

        if [ -d "$2" ]
        then
            cp_target="$cp_target_parent_dir"/$(basename "$file")
        else
            cp_target="$2"
        fi

        if ! cmp --quiet "$file" "$cp_target"
        then
            tue-install-debug "File $file and $cp_target are different, copying..."
            if "$root_required"
            then
                tue-install-debug "Using elevated privileges (sudo) to copy ${file} to ${cp_target}"
                tue-install-pipe sudo mkdir --parents --verbose "$cp_target_parent_dir" && tue-install-pipe sudo cp --verbose "$file" "$cp_target"
            else
                tue-install-pipe mkdir --parents --verbose "$cp_target_parent_dir" && tue-install-pipe cp --verbose "$file" "$cp_target"
            fi
        else
            tue-install-debug "File $file and $cp_target are the same, no action needed"
        fi

    done
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# Reads SOURCE_FILE and looks in TARGET_FILE for the first and last line of SOURCE_FILE. If these
# are not found, SOURCE_FILE is appended to TARGET_FILE. Otherwise, the appearance of the first and
# last line of SOURCE_FILE in TARGET_FILE, and everything in between, is replaced by the contents
# of SOURCE_FILE.
# This is useful for adding text blocks to files and allowing to change only that part of the file
# on a next update. It is advised to start and end SOURCE_FILE with unique tags, e.g.:
#
#    # BEGIN TU/E BLOCK
#    .... some text ...
#    # END TU/E BLOCK
#
function tue-install-add-text
{
    tue-install-debug "tue-install-add-text $*"

    if [ -z "$2" ]
    then
        tue-install-error "Invalid tue-install-add-text call. Usage: tue-install-add-text SOURCE_FILE TARGET_FILE"
    fi

    local source_file=$1
    # shellcheck disable=SC2088
    if [[ "$source_file" == "/"* ]] || [[ "$source_file" == "~/"* ]]
    then
        tue-install-error "tue-install-add-text: Only relative source files to the target directory are allowed"
    else
        source_file="$TUE_INSTALL_CURRENT_TARGET_DIR"/"$source_file"
    fi
    local target_file=$2
    # shellcheck disable=SC2088
    if [[ "$target_file" != "/"* ]] && [[ "$source_file" != "~/"* ]]
    then
        tue-install-error "tue-install-add-text: target file needs to be absolute or relative to the home directory"
    fi

    local root_required=true
    if namei -l "$target_file" | grep -q "$(whoami)"
    then
        tue-install-debug "tue-install-add-text: NO root required"
        root_required=false
    else
        tue-install-debug "tue-install-add-text: root required"
    fi

    if [ ! -f "$source_file" ]
    then
        tue-install-error "tue-install-add-text: No such source file: $source_file"
    fi

    if [ ! -f "$target_file" ]
    then
        tue-install-error "tue-install-add-text: No such target file: $target_file"
    fi

    local begin_tag end_tag text
    begin_tag=$(head -n 1 "$source_file")
    end_tag=$(awk '/./{line=$0} END{print line}' "$source_file")
    text=$(sed -e :a -e '/^\n*$/{$d;N;};/\n&/ba' "$source_file")
    tue-install-debug "tue-install-add-text: Lines to be added: \n$text"

    if ! grep -q "$begin_tag" "$target_file"
    then
        tue-install-debug "tue-install-add-text: Appending $target_file"
        if $root_required
        then
            echo -e "$text" | sudo tee --append "$target_file" 1> /dev/null
        else
            echo -e "$text" | tee --append "$target_file" 1> /dev/null
        fi
    else
        tue-install-debug "tue-install-add-text: Begin tag already in $target_file, so comparing the files for changed lines"
        local tmp_source_file="/tmp/tue-install-add-text_source_temp_${USER}_${TUE_INSTALL_CURRENT_TARGET}_${stamp}"
        local tmp_target_file="/tmp/tue-install-add-text_target_temp_${USER}_${TUE_INSTALL_CURRENT_TARGET}_${stamp}"

        echo "$text" | tee "$tmp_source_file" > /dev/null
        sed -e "/^$end_tag/r $tmp_source_file" -e "/^$begin_tag/,/^$end_tag/d" "$target_file" | tee "$tmp_target_file" 1> /dev/null

        if ! cmp --quiet "$tmp_target_file" "$target_file"
        then
            tue-install-debug "tue-install-add-text: Lines are changed, so copying"
            if $root_required
            then
                sudo mv "$tmp_target_file" "$target_file"
            else
                mv "$tmp_target_file" "$target_file"
            fi
        else
            tue-install-debug "tue-install-add-text: Lines have not changed, so not copying"
        fi
        rm "$tmp_source_file" "$tmp_target_file"
    fi
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-get-releases
{
    tue-install-debug "tue-install-get-releases $*"

    if test $# -lt 3
    then
        tue-install-error "Invalid tue-install-get-releases call: needs at least 3 input parameters"
    fi

    local repo_short_url=$1
    local filename=$2
    local output_dir=$3
    local tag=

    if [ -z "$4" ]
    then
        tag="-l"
    else
        tag="-t=$4"
    fi

    "$TUE_INSTALL_SCRIPTS_DIR"/github-releases.py --get -u "$repo_short_url" "$tag" -o "$output_dir" "$filename" || \
        tue-install-error "Failed to get '$filename' from '$repo_short_url'"
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-system
{
    tue-install-debug "tue-install-system $*"

    if [ -z "$1" ]
    then
        tue-install-error "Invalid tue-install-system call: needs package as argument."
    fi
    tue-install-debug "Adding $1 to apt list"
    TUE_INSTALL_SYSTEMS="$1 $TUE_INSTALL_SYSTEMS"
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-system-now
{
    tue-install-debug "tue-install-system-now $*"

    if [ -z "$1" ]
    then
        tue-install-error "Invalid tue-install-system-now call: needs package as argument."
    fi

    local pkgs_to_install=""
    local dpkg_query
    # shellcheck disable=SC2016
    dpkg_query=$(dpkg-query -W -f '${package} ${status}\n' 2>/dev/null)
    # shellcheck disable=SC2048
    for pkg in $*
    do
        # Check if pkg is not already installed dpkg -S does not cover previously removed packages
        # Based on https://stackoverflow.com/questions/1298066
        if ! grep -q "^$pkg install ok installed" <<< "$dpkg_query"
        then
            pkgs_to_install="$pkgs_to_install $pkg"
        else
            tue-install-debug "$pkg is already installed"
        fi
    done

    if [ -n "$pkgs_to_install" ]
    then
        tue-install-echo "Going to run the following command:\n\nsudo apt-get install --assume-yes -q $pkgs_to_install\n"

        # Wait for apt-lock first (https://askubuntu.com/a/375031)
        i=0
        tput sc
        while sudo fuser /var/lib/dpkg/lock >/dev/null 2>&1
        do
            case $((i % 4)) in
                0 ) j="-" ;;
                1 ) j="\\" ;;
                2 ) j="|" ;;
                3 ) j="/" ;;
            esac
            tput rc
            echo -en "\r[$j] Waiting for other software managers to finish..."
            sleep 0.5
            ((i=i+1))
        done

        if [ ! -f "$TUE_APT_GET_UPDATED_FILE" ]
        then
            # Update once every boot. Or delete the tmp file if you need an update before installing a pkg.
            tue-install-pipe sudo apt-get update || tue-install-error "An error occurred while updating apt-get."
            touch $TUE_APT_GET_UPDATED_FILE
        fi

        # shellcheck disable=SC2086
        tue-install-pipe sudo apt-get install --assume-yes -q $pkgs_to_install || tue-install-error "An error occurred while installing system packages."
        tue-install-debug "Installed $pkgs_to_install ($?)"
    fi
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-apt-get-update
{
    tue-install-debug "tue-install-apt-get-update"
    tue-install-debug "Requiring an update of apt-get before next 'apt-get install'"
    rm -f $TUE_APT_GET_UPDATED_FILE
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-ppa
{
    tue-install-debug "tue-install-ppa $*"

    if [ -z "$1" ]
    then
        tue-install-error "Invalid tue-install-ppa call: needs ppa as argument."
    fi
    local ppa="$*"

    if [[ $ppa != "ppa:"* && $ppa != "deb"* ]]
    then
        tue-install-error "Invalid tue-install-ppa call: needs to start with 'ppa:' or 'deb ' ($ppa)"
    fi
    tue-install-debug "Adding $ppa to PPA list"
    TUE_INSTALL_PPA="${TUE_INSTALL_PPA} ${ppa// /^}"  # Replace space by ^ to support for-loops later
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-ppa-now
{
    tue-install-debug "tue-install-ppa-now $*"

    if [ -z "$1" ]
    then
        tue-install-error "Invalid tue-install-ppa-now call: needs ppa or deb as argument."
    fi

    local PPA_ADDED=""
    local needs_to_be_added
    # shellcheck disable=SC2048
    for ppa in $*
    do
        ppa="${ppa//^/ }"
        if [[ $ppa != "ppa:"* && $ppa != "deb "* ]]
        then
            tue-install-error "Invalid tue-install-ppa-now call: needs to start with 'ppa:' or 'deb ' ($ppa)"
        fi
        needs_to_be_added="false"
        if [[ "$ppa" == "ppa:"* ]]
        then
            if ! grep -q "^deb.*${ppa#ppa:}" /etc/apt/sources.list.d/* 2>&1
            then
                needs_to_be_added="true"
            fi
        elif [[ "$ppa" == "deb "* ]]
        then
            if ! grep -qF "$ppa" /etc/apt/sources.list 2>&1
            then
                needs_to_be_added="true"
            fi
        else
            tue-install-warning "tue-install-ppa-now: We shouldn't end up here ($ppa)"
        fi

        if [ "$needs_to_be_added" == "true" ]
        then
            tue-install-system-now software-properties-common
            tue-install-info "Adding ppa: $ppa"

            # Wait for apt-lock first (https://askubuntu.com/a/375031)
            i=0
            tput sc
            while sudo fuser /var/lib/apt/lists/lock >/dev/null 2>&1
            do
                case $((i % 4)) in
                    0 ) j="-" ;;
                    1 ) j="\\" ;;
                    2 ) j="|" ;;
                    3 ) j="/" ;;
                esac
                tput rc
                echo -en "\r[$j] Waiting for other software managers to finish..."
                sleep 0.5
                ((i=i+1))
            done

            tue-install-pipe sudo add-apt-repository --yes "$ppa" || tue-install-error "An error occurred while adding ppa: $ppa"
            PPA_ADDED=true
        else
            tue-install-debug "$ppa is already added previously"
        fi
    done
    if [ -n "$PPA_ADDED" ]
    then
        tue-install-apt-get-update
    fi
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function _tue-install-pip
{
    local pv=$1
    shift
    tue-install-debug "tue-install-pip${pv} $*"

    if [ -z "$1" ]
    then
        tue-install-error "Invalid tue-install-pip${pv} call: needs package as argument."
    fi
    tue-install-debug "Adding $1 to pip${pv} list"
    local list=TUE_INSTALL_PIP"${pv}"S
    # shellcheck disable=SC2140
    declare -g "$list"="$1 ${!list}"
}

# Needed for backward compatibility
function tue-install-pip
{
    _tue-install-pip "3" "$@"
}

function tue-install-pip3
{
    _tue-install-pip "3" "$@"
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function _tue-install-pip-now
{
    local pv=$1
    shift
    tue-install-debug "tue-install-pip${pv}-now $*"

    if [ -z "$1" ]
    then
        tue-install-error "Invalid tue-install-pip${pv}-now call: needs package as argument."
    fi

    # Make sure pip is up-to-date before checking version and installing
    local pip_version desired_pip_version
    pip_version=$(python"${pv}" -m pip --version | awk '{print $2}')
    desired_pip_version="21"
    if [ "$pv" == 2 ]
    then
        desired_pip_version="20"
    fi
    if version_gt "$desired_pip_version" "$pip_version"
    then
        tue-install-debug "pip${pv} not yet version >=$desired_pip_version, but $pip_version"
        tue-install-pipe python"${pv}" -m pip install --user --upgrade pip
        hash -r
    else
        tue-install-debug "Already pip${pv}>=$desired_pip_version"
    fi

    local pips_to_check=""
    local pips_to_install=""
    local git_pips_to_install=""
    # shellcheck disable=SC2048
    for pkg in $*
    do
        if [[ "$pkg" == "git+"* ]]
        then
            git_pips_to_install="$git_pips_to_install $pkg"
        else
            pips_to_check="$pips_to_check $pkg"
        fi
    done

    read -r -a pips_to_check <<< "$pips_to_check"
    local installed_versions
    if [ ${#pips_to_check[@]} -gt 0 ]
    then
        installed_versions=$(python"${pv}" "$TUE_INSTALL_SCRIPTS_DIR"/check-pip-pkg-installed-version.py "${pips_to_check[@]}")
        local error_code=$?
        if [ "$error_code" -gt 1 ]
        then
            tue-install-error "tue-install-pip${pv}-now: $installed_versions"
        fi
    fi
    read -r -a installed_versions <<< "$installed_versions"

    if [ "${#pips_to_check[@]}" -ne "${#installed_versions[@]}" ]
    then
        tue-install-error "Lengths of pips_to_check, ${#pips_to_check[@]}, and installed_version, ${#installed_versions[@]}, don't match"
    fi

    for idx in "${!pips_to_check[@]}"
    do
        local pkg_req="${pips_to_check[$idx]}"
        local pkg_installed="${installed_versions[$idx]}"
        pkg_installed="${pkg_installed//^/ }"
        if [[ "$error_code" -eq 1 && "$pkg_installed" == "None" ]]
        then
            pips_to_install="$pips_to_install $pkg_req"
        else
            tue-install-debug "$pkg_req is already installed, $pkg_installed"
        fi
    done

    if [ -n "$pips_to_install" ]
    then
        # shellcheck disable=SC2048,SC2086
        tue-install-pipe python"${pv}" -m pip install --user $pips_to_install <<< yes || tue-install-error "An error occurred while installing pip${pv} packages."
    fi

    if [ -n "$git_pips_to_install" ]
    then
        for pkg in $git_pips_to_install
        do
            # shellcheck disable=SC2048,SC2086
            tue-install-pipe python"${pv}" -m pip install --user $pkg <<< yes || tue-install-error "An error occurred while installing pip${pv} packages."
        done
    fi
}

# Needed for backward compatibility
function tue-install-pip-now
{
    _tue-install-pip-now "3" "$@"
}

function tue-install-pip3-now
{
    _tue-install-pip-now "3" "$@"
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-snap
{
    tue-install-debug "tue-install-snap $*"

    if [ -z "$1" ]
    then
        tue-install-error "Invalid tue-install-snap call: needs package as argument."
    fi
    tue-install-debug "Adding $1 to snap list"
    TUE_INSTALL_SNAPS="$1 $TUE_INSTALL_SNAPS"
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-snap-now
{
    tue-install-debug "tue-install-snap-now $*"

    if [ -z "$1" ]
    then
        tue-install-error "Invalid tue-install-snap-now call: needs package as argument."
    fi

    tue-install-system-now snapd

    local snaps_to_install snaps_installed
    snaps_to_install=""
    snaps_installed=$(snap list)
    # shellcheck disable=SC2048
    for pkg in $*
    do
        if [[ ! $snaps_installed == *$pkg* ]] # Check if pkg is not already installed
        then
            snaps_to_install="$snaps_to_install $pkg"
            tue-install-debug "snap pkg: $pkg is not yet installed"
        else
            tue-install-debug "snap pkg: $pkg is already installed"
        fi
    done

    if [ -n "$snaps_to_install" ]
    then
        for pkg in $snaps_to_install
        do
            tue-install-pipe sudo snap install --classic "$pkg" <<< yes || tue-install-error "An error occurred while installing snap packages."
        done
    fi
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-dpkg-now
{
    tue-install-debug "tue-install-dpkg-now $*"
    tue-install-debug "calling: tue-install-dpkg $*"
    tue-install-dpkg "$@"
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-dpkg
{
    tue-install-debug "tue-install-dpkg $*"

    if [ -z "$1" ]
    then
        tue-install-error "Invalid tue-install-dpkg call: needs package as argument."
    fi
    tue-install-pipe sudo dpkg --install "$1"
    tue-install-pipe sudo apt-get --fix-broken --assume-yes -q install || tue-install-error "An error occured while fixing dpkg install"
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function tue-install-ros
{
    tue-install-debug "tue-install-ros $*"

    local install_type=$1
    local src=$2
    local sub_dir=$3
    local version=$4

    tue-install-debug "Installing ros package: type: $install_type, source: $src"

    [ -n "$TUE_ROS_DISTRO" ] || tue-install-error "Environment variable 'TUE_ROS_DISTRO' is not set."

    local ros_pkg_name=${TUE_INSTALL_CURRENT_TARGET#ros-}
    if [[ $ros_pkg_name == *-* ]]
    then
        tue-install-error "A ROS package cannot contain dashes (${ros_pkg_name}), make sure the package is named '${ros_pkg_name//-/_}' and rename the target to 'ros-${ros_pkg_name//-/_}'"
        return 1
    fi

    # First of all, make sure ROS itself is installed
    tue-install-target ros || tue-install-error "Failed to install target 'ROS'"

    if [ "$install_type" == "system" ]
    then
        tue-install-debug "tue-install-system ros-$TUE_ROS_DISTRO-$src"
        tue-install-system ros-"$TUE_ROS_DISTRO"-"$src"
        return 0
    fi

    if [ -z "$ROS_PACKAGE_INSTALL_DIR" ]
    then
        tue-install-error "Environment variable ROS_PACKAGE_INSTALL_DIR not set."
    fi

    # Make sure the ROS package install dir exists
    tue-install-debug "Creating ROS package install dir: $ROS_PACKAGE_INSTALL_DIR"
    mkdir -p "$ROS_PACKAGE_INSTALL_DIR"

    local ros_pkg_dir="$ROS_PACKAGE_INSTALL_DIR"/"$ros_pkg_name"
    local repos_dir
    if [ "$install_type" == "git" ]
    then
        local output
        output=$(_git_split_url "$src")

        local array
        read -r -a array <<< "$output"
        local domain_name=${array[0]}
        local repo_address=${array[1]}
        repos_dir="$TUE_REPOS_DIR"/"$domain_name"/"$repo_address"
    else
        repos_dir="$TUE_REPOS_DIR"/"$src"
        # replace spaces with underscores
        repos_dir=${repos_dir// /_}
        # now, clean out anything that's not alphanumeric or an underscore
        repos_dir=${repos_dir//[^a-zA-Z0-9\/\.-]/_}
    fi

    tue-install-debug "repos_dir: $repos_dir"

    if [ "$install_type" == "git" ]
    then
        tue-install-git "$src" "$repos_dir" "$version"
    else
        tue-install-error "Unknown ros install type: '${install_type}'"
    fi

    if [ -d "$repos_dir" ]
    then
        if [ ! -d "$repos_dir"/"$sub_dir" ]
        then
            tue-install-error "Subdirectory '$sub_dir' does not exist for URL '$src'."
        fi

        if [ -L "$ros_pkg_dir" ]
        then
            # Test if the current symbolic link points to the same repository dir. If not, give a warning
            # because it means the source URL has changed
            if [ ! "$ros_pkg_dir" -ef "$repos_dir"/"$sub_dir" ]
            then
                tue-install-info "URL has changed to $src/$sub_dir"
                rm "$ros_pkg_dir"
                ln -s "$repos_dir"/"$sub_dir" "$ros_pkg_dir"
            fi
        elif [ -d "$ros_pkg_dir" ]
        then
            tue-install-error "Can not create a symlink at '$ros_pkg_dir' as it is a directory"
        elif [ ! -e "$ros_pkg_dir" ]
        then
            # Create a symbolic link to the system workspace
            ln -s "$repos_dir"/"$sub_dir" "$ros_pkg_dir"
        else
            tue-install-error "'$ros_pkg_dir' should not exist or be a symlink, any other option is incorrect"
        fi

        if [[ "$TUE_INSTALL_SKIP_ROS_DEPS" != "all" ]]
        then
            local pkg_xml="$ros_pkg_dir"/package.xml
            if [ -f "$pkg_xml" ]
            then
                # Catkin
                tue-install-debug "Parsing $pkg_xml"
                local deps
                deps=$("$TUE_INSTALL_SCRIPTS_DIR"/parse-package-xml.py "$pkg_xml")
                tue-install-debug "Parsed package.xml\n$deps"

                for dep in $deps
                do
                    # Preference given to target name starting with ros-
                    tue-install-target ros-"$dep" || tue-install-target "$dep" || \
                        tue-install-error "Targets 'ros-$dep' and '$dep' don't exist"
                done

            else
                tue-install-warning "Does not contain a valid ROS package.xml"
            fi
        else
            tue-install-debug "No need to parse package.xml for dependencies"
        fi

    else
        tue-install-error "Checking out $src was not successful."
    fi

    TUE_INSTALL_PKG_DIR=$ros_pkg_dir
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

function _missing_targets_check
{
    tue-install-debug "_missing_targets_check $*"

    # Check if valid target received as input
    local targets="$1"
    local missing_targets=""
    local target

    for target in $targets
    do
        if [ ! -d "$TUE_INSTALL_TARGETS_DIR"/"$target" ]
        then
            missing_targets="$target${missing_targets:+ ${missing_targets}}"
        fi
    done

    if [ -n "$missing_targets" ]
    then
        missing_targets=$(echo "$missing_targets" | tr " " "\n" | sort)
        tue-install-error "The following installed targets don't exist (anymore):\n$missing_targets"
    fi

    return 0
}

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#                                           MAIN LOOP
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

TUE_INSTALL_CURRENT_TARGET="main-loop"

tue_get_cmd=$1
shift

# idiomatic parameter and option handling in sh
targets=""
BRANCH=""
while test $# -gt 0
do
    case "$1" in
        --debug)
            DEBUG="true"
            ;;
        --no-ros-deps)
            export TUE_INSTALL_SKIP_ROS_DEPS="all"
            ;;
        --doc-depend)
            [[ "$TUE_INSTALL_SKIP_ROS_DEPS" == "all" ]] && export TUE_INSTALL_SKIP_ROS_DEPS="normal"
            export TUE_INSTALL_DOC_DEPEND="true"
            ;;
        --no-doc-depend)
            export TUE_INSTALL_DOC_DEPEND="false"
            ;;
        --test-depend)
            [[ "$TUE_INSTALL_SKIP_ROS_DEPS" == "all" ]] && export TUE_INSTALL_SKIP_ROS_DEPS="normal"
            export TUE_INSTALL_TEST_DEPEND="true"
            ;;
        --no-test-depend)
            export TUE_INSTALL_TEST_DEPEND="false"
            ;;
        --branch*)
            # shellcheck disable=SC2001
            BRANCH=$(echo "$1" | sed -e 's/^[^=]*=//g')
            ;;
        --*)
            echo "unknown option $1"
            ;;
        *)
            targets="$targets $1"
            ;;
    esac
    shift
done


# Create log file
stamp=$(date_stamp)
INSTALL_DETAILS_FILE=/tmp/tue-get-details-$stamp
touch "$INSTALL_DETAILS_FILE"

# Initialize
ROS_PACKAGE_INSTALL_DIR=$TUE_SYSTEM_DIR/src

TUE_INSTALL_SCRIPTS_DIR=$TUE_DIR/installer

TUE_INSTALL_GENERAL_STATE_DIR=/tmp/tue-installer
if [ ! -d $TUE_INSTALL_GENERAL_STATE_DIR ]
then
    tue-install-debug "mkdir $TUE_INSTALL_GENERAL_STATE_DIR"
    mkdir "$TUE_INSTALL_GENERAL_STATE_DIR"
    tue-install-debug "chmod a+rwx $TUE_INSTALL_GENERAL_STATE_DIR"
    chmod a+rwx "$TUE_INSTALL_GENERAL_STATE_DIR"
fi

TUE_INSTALL_STATE_DIR=$TUE_INSTALL_GENERAL_STATE_DIR/$stamp
mkdir -p "$TUE_INSTALL_STATE_DIR"

TUE_INSTALL_GIT_PULL_Q=()

TUE_INSTALL_SYSTEMS=
TUE_INSTALL_PPA=
TUE_INSTALL_PIP3S=
TUE_INSTALL_SNAPS=

TUE_INSTALL_WARNINGS=
TUE_INSTALL_INFOS=

# Make sure tools used by this installer are installed
tue-install-system-now curl git jq python-is-python3 python3-pip wget

tue-install-pip3-now catkin-pkg PyYAML


# Handling of targets
if [[ -z "${targets// }" ]] #If only whitespace
then
    # If no targets are provided, update all installed targets
    targets=$(ls "$TUE_INSTALL_INSTALLED_DIR")
else
    raw_targets=$targets
    targets=""
    for target in $raw_targets
    do
        resolved_targets="$(find "$TUE_INSTALL_TARGETS_DIR" -maxdepth 1 -name "$target" -type d -printf "%P ")"
        if [ -z "$resolved_targets" ] # So the missing target is handled by _missing_targets_check
        then
            resolved_targets="$target"
        fi
        targets="${targets:+$targets }$resolved_targets"
    done
fi


# Check if all installed targets exist in the targets repo
_missing_targets_check "$targets"

for target in $targets
do
    tue-install-debug "Main loop: installing $target"
    # Next line shouldn't error anymore with _missing_targets_check
    tue-install-target "$target" || tue-install-error "Installed target: '$target' doesn't exist (anymore)"

    if [[ "$tue_get_cmd" == "install" ]]
    then
        # Mark as installed
        tue-install-debug "[$target] marked as installed after a successful install"
        touch "$TUE_INSTALL_INSTALLED_DIR"/"$target"
    else
        tue-install-debug "[$target] succesfully updated"
    fi
done


# Display infos
if [ -n "$TUE_INSTALL_INFOS" ]
then
    echo -e "\e[0;36m\nSome information you may have missed:\n\n$TUE_INSTALL_INFOS\e[0m"
fi

# Display warnings
if [ -n "$TUE_INSTALL_WARNINGS" ]
then
    echo -e "\e[33;1m\nOverview of warnings:\n\n$TUE_INSTALL_WARNINGS\e[0m"
fi


# Remove temp directories
rm -rf "$TUE_INSTALL_STATE_DIR"


# Installing all the ppa repo's, which are collected during install
if [ -n "$TUE_INSTALL_PPA" ]
then
    TUE_INSTALL_CURRENT_TARGET="PPA-ADD"

    tue-install-debug "calling: tue-install-ppa-now $TUE_INSTALL_PPA"
    tue-install-ppa-now "$TUE_INSTALL_PPA"
fi


# Installing all system (apt-get) targets, which are collected during the install
if [ -n "$TUE_INSTALL_SYSTEMS" ]
then
    TUE_INSTALL_CURRENT_TARGET="APT-GET"

    tue-install-debug "calling: tue-install-system-now $TUE_INSTALL_SYSTEMS"
    tue-install-system-now "$TUE_INSTALL_SYSTEMS"
fi


# Installing all python3 (pip3) targets, which are collected during the install
if [ -n "$TUE_INSTALL_PIP3S" ]
then
    TUE_INSTALL_CURRENT_TARGET="PIP3"

    tue-install-debug "calling: tue-install-pip3-now $TUE_INSTALL_PIP3S"
    tue-install-pip3-now "$TUE_INSTALL_PIP3S"
fi


# Installing all snap targets, which are collected during the install
if [ -n "$TUE_INSTALL_SNAPS" ]
then
    TUE_INSTALL_CURRENT_TARGET="SNAP"

    tue-install-debug "calling: tue-install-snap-now $TUE_INSTALL_SNAPS"
    tue-install-snap-now "$TUE_INSTALL_SNAPS"
fi

TUE_INSTALL_CURRENT_TARGET="main-loop"

tue-install-echo "Installer completed succesfully"

return 0
