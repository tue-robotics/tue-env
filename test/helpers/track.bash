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

function tue_track_min_forks
{
    # $1: how many names of each of the three kinds to put in the shell first. Result in
    # TUE_TRACK_FORKS: the smallest number of processes one begin/commit pair created, over five
    # samples.
    #
    # Measured in a child `bash --norc --noprofile`, never in the bats shell: bats installs a DEBUG
    # trap that runs on every single command and forks of its own, so the figure taken here would be
    # dominated by the harness AND would grow with the amount of work the tracker does - which is
    # exactly the thing under test.
    #
    # Forks are counted from the PIDs the kernel hands out: they are allocated in sequence, so the
    # gap between two bracketing subshells covers every process created in between. Anything else
    # running on the machine can only widen that gap, never narrow it, so the smallest sample is the
    # measurement and every assertion built on it has to be one-sided.
    # shellcheck disable=SC2034
    TUE_TRACK_FORKS="$(bash --norc --noprofile -c '
        # shellcheck source=/dev/null
        source "$1/setup/tue-env-track.bash"
        for (( __tue_env_i = 0; __tue_env_i < $2; __tue_env_i++ ))
        do
            eval "TUE_TEST_MEAS_V${__tue_env_i}=x"
            eval "tue_test_meas_f${__tue_env_i}() { echo ${__tue_env_i}; }"
            eval "alias tue_test_meas_a${__tue_env_i}=\"echo ${__tue_env_i}\""
        done
        __tue_env_m=-1
        for (( __tue_env_k = 0; __tue_env_k < 5; __tue_env_k++ ))
        do
            __tue_env_a="$(echo "${BASHPID}")"
            _tue-env-track-begin
            _tue-env-track-commit
            __tue_env_b="$(echo "${BASHPID}")"
            __tue_env_d=$(( __tue_env_b - __tue_env_a - 1 ))
            (( __tue_env_d < 0 )) && continue
            if (( __tue_env_m < 0 )) || (( __tue_env_d < __tue_env_m ))
            then
                __tue_env_m="${__tue_env_d}"
            fi
        done
        printf "%s" "${__tue_env_m}"
    ' bash "${TUE_TRACK_REPO_ROOT}" "$1")"
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
