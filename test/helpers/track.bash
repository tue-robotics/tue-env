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

    # Keep bats' DEBUG trap out of the tracker's own functions. The trap runs bats_debug_trap on
    # EVERY command executed under `set -T`, and each call captures a whole stack trace; a snapshot
    # parses one record per name in the shell, so a single begin/commit pair fires it thousands of
    # times and costs five seconds here against 0.15 in a plain bash. Excluding the directory the
    # tracker is sourced from makes bats skip that work for commands whose BASH_SOURCE lives there,
    # which is the whole of the tracker. Failures inside the .bats files are still reported with
    # their line, because those files are not excluded.
    #
    # No trailing slash: bats compares the exclude path against the DIRECTORY of the running
    # command's source file, so `.../setup/` never matches `.../setup` and silently buys nothing.
    if declare -F bats_add_debug_exclude_path > /dev/null
    then
        bats_add_debug_exclude_path "${TUE_TRACK_REPO_ROOT}/setup"
    fi
    return 0
}

function tue_track_snapshot
{
    # $1: PRE or POST. Snapshots this shell into the __TUE_ENV_$1_* arrays. The nonce is drawn here
    # for the same reason _tue-env-track-begin draws it: the record token has to be one no already
    # defined function body could be carrying, and __tue_env_track_dump does not draw it itself so
    # that two back-to-back dumps stay byte-identical.
    local __tue_env_snap
    __tue_env_track_nonce
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
        # The entries are escaped on the wire, exactly as an alias value is; print them the way the
        # tracker's own readers see them.
        __tue_env_track_unescape "${__tue_env_p#*"${__TUE_ENV_PS}"}"
        __tue_env_o+="=${__TUE_ENV_UNESCAPED}"
    done
    printf '%s' "${__tue_env_o}"
    return 0
}

function tue_track_parse_cost
{
    # Measures one parse of a large snapshot against a reference pass over the same payload. Results
    # in TUE_TRACK_PARSE_US and TUE_TRACK_BASE_US, both in microseconds.
    #
    # An absolute bound would be a bound on whatever machine ran the suite. The reference is the
    # escape, which is four parameter expansions over the whole payload: it is O(bytes) with a small
    # constant, it is the tracker's own code, and it rises and falls with the same clock the parse
    # does. So the two figures can be compared on any machine, and a parse that reads its stream a
    # byte at a time stands out as a multiple of the reference rather than as a number of seconds.
    #
    # Measured in a child `bash --norc --noprofile` for the reason tue_track_min_forks gives: bats'
    # DEBUG trap runs on every command and would dominate both figures. LC_ALL pins the decimal point
    # in EPOCHREALTIME, which is stripped to get integer microseconds without forking a calculator.
    # The smallest of several samples is the measurement, because anything else on the machine can
    # only ever make a sample slower.
    local __tue_env_out
    __tue_env_out="$(LC_ALL=C bash --norc --noprofile -c '
        # shellcheck source=/dev/null
        source "$1/setup/tue-env-track.bash"

        __tue_env_chunk="$(printf "x%.0s" {1..1000})"
        __tue_env_body=""
        for (( __tue_env_i = 0; __tue_env_i < 1000; __tue_env_i++ ))
        do
            __tue_env_body+="${__tue_env_chunk}"
        done

        __tue_env_track_nonce
        __tue_env_stream=""
        for __tue_env_n in alpha beta gamma
        do
            __tue_env_stream+="${__TUE_ENV_MARK}${__TUE_ENV_FS}V${__TUE_ENV_FS}${__tue_env_n}${__TUE_ENV_FS}declare -x ${__tue_env_n}=\"v\"
${__TUE_ENV_RS}"
        done
        __tue_env_stream+="${__TUE_ENV_MARK}${__TUE_ENV_FS}F${__TUE_ENV_FS}big${__TUE_ENV_FS}${__TUE_ENV_FS}big () { : ${__tue_env_body}; }
${__TUE_ENV_RS}"

        __tue_env_pmin=-1
        __tue_env_bmin=-1
        for (( __tue_env_k = 0; __tue_env_k < 3; __tue_env_k++ ))
        do
            __tue_env_a="${EPOCHREALTIME/./}"
            __tue_env_track_parse "${__tue_env_stream}" PRE
            __tue_env_b="${EPOCHREALTIME/./}"
            __tue_env_d=$(( __tue_env_b - __tue_env_a ))
            if (( __tue_env_pmin < 0 )) || (( __tue_env_d < __tue_env_pmin ))
            then
                __tue_env_pmin="${__tue_env_d}"
            fi

            __tue_env_a="${EPOCHREALTIME/./}"
            __tue_env_track_escape "${__tue_env_body}"
            __tue_env_b="${EPOCHREALTIME/./}"
            __tue_env_d=$(( __tue_env_b - __tue_env_a ))
            if (( __tue_env_bmin < 0 )) || (( __tue_env_d < __tue_env_bmin ))
            then
                __tue_env_bmin="${__tue_env_d}"
            fi
        done
        printf "%s %s" "${__tue_env_pmin}" "${__tue_env_bmin}"
    ' bash "${TUE_TRACK_REPO_ROOT}")"
    # shellcheck disable=SC2034
    TUE_TRACK_PARSE_US="${__tue_env_out%% *}"
    # shellcheck disable=SC2034
    TUE_TRACK_BASE_US="${__tue_env_out##* }"
    return 0
}
