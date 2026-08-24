# shellcheck shell=bash
#
# Common helper for the change-tracking tests:
#
#   load helpers/track
#
#   setup() {
#       tue_track_setup
#   }
#
# Any test variable that holds a snapshot must be named __tue_env_*, or the snapshot will contain it.

TUE_TRACK_REPO_ROOT="$( cd "${BATS_TEST_DIRNAME}/.." && pwd )"

function tue_track_setup
{
    # shellcheck source=/dev/null
    source "${TUE_TRACK_REPO_ROOT}/setup/tue-env-track.bash"

    # bats rewrites BATS_DEBUG_* from its DEBUG trap on every command, and `run` rewrites
    # output/status/lines; without hiding them every snapshot pair would differ.
    __TUE_ENV_TRACK_EXTRA_EXCLUDE=('BATS_*' 'output' 'status' 'lines' 'stderr' 'stderr_lines')
    return 0
}

function tue_track_snapshot
{
    # $1: PRE or POST. Snapshots this shell into the __TUE_ENV_$1_* arrays.
    local __tue_env_snap
    __tue_env_snap="$(__tue_env_track_dump)"
    __tue_env_track_parse "${__tue_env_snap}" "$1"
    return 0
}

function tue_track_added
{
    # $1: variable name. Prints the recorded added entries as "index=entry,index=entry".
    local __tue_env_r="${__TUE_ENV_LEDGER_VAR_ADD[$1]:-}" __tue_env_p __tue_env_o=""
    while [[ -n "${__tue_env_r}" ]]
    do
        # A stream with no more RS cannot be split further; without this, an unterminated tail spins.
        [[ "${__tue_env_r}" == *"${__TUE_ENV_RS}"* ]] || break
        __tue_env_p="${__tue_env_r%%"${__TUE_ENV_RS}"*}"
        __tue_env_r="${__tue_env_r#*"${__TUE_ENV_RS}"}"
        [[ -z "${__tue_env_p}" ]] && continue
        __tue_env_o+="${__tue_env_o:+,}${__tue_env_p%%"${__TUE_ENV_PS}"*}"
        __tue_env_o+="=${__tue_env_p#*"${__TUE_ENV_PS}"}"
    done
    printf '%s' "${__tue_env_o}"
    return 0
}
