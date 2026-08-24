load helpers/track

setup() {
    tue_track_setup
}

@test "revert: entries are removed one by one and entries the user added survive" {
    export TUE_TEST_LIST="/usr/bin:/bin"
    _tue-env-track-begin
    export TUE_TEST_LIST="/usr/lib/ccache:/opt/ros/jazzy/bin:${TUE_TEST_LIST}"
    _tue-env-track-commit
    export TUE_TEST_LIST="/opt/foo/bin:${TUE_TEST_LIST}"
    __tue_env_track_revert_vars
    [[ "${TUE_TEST_LIST}" == "/opt/foo/bin:/usr/bin:/bin" ]]
}

@test "revert: a duplicate entry is removed exactly once, at the recorded index" {
    export TUE_TEST_LIST="A:X:B"
    _tue-env-track-begin
    export TUE_TEST_LIST="A:X:B:X"
    _tue-env-track-commit
    __tue_env_track_revert_vars
    [[ "${TUE_TEST_LIST}" == "A:X:B" ]]
}

@test "revert: an entry the ledger recorded but that is gone is skipped silently" {
    export TUE_TEST_LIST="/usr/bin"
    _tue-env-track-begin
    export TUE_TEST_LIST="/gone:${TUE_TEST_LIST}"
    _tue-env-track-commit
    export TUE_TEST_LIST="/usr/bin"
    __tue_env_track_revert_vars
    [[ "${TUE_TEST_LIST}" == "/usr/bin" ]]
}

@test "revert: an added list variable is unset when nothing is left of it" {
    _tue-env-track-begin
    export TUE_TEST_PP="/a:/b"
    _tue-env-track-commit
    __tue_env_track_revert_vars
    [[ -z "${TUE_TEST_PP+set}" ]]
}

@test "revert: an added list variable the user appended to is kept whole" {
    # `added` is not `extended`: the spec puts everything that is not entry-wise through the conflict
    # check, so a variable the environment created and the user then changed is handed back untouched.
    _tue-env-track-begin
    export TUE_TEST_PP="/a"
    _tue-env-track-commit
    export TUE_TEST_PP="${TUE_TEST_PP}:/mine"
    run __tue_env_track_revert_vars
    [[ "${output}" == *"kept your value for TUE_TEST_PP"* ]]
    __tue_env_track_revert_vars
    [[ "${TUE_TEST_PP}" == "/a:/mine" ]]
}

@test "revert: a replaced scalar is restored" {
    export TUE_TEST_RMW=rmw_fastrtps_cpp
    _tue-env-track-begin
    export TUE_TEST_RMW=rmw_cyclonedds_cpp
    _tue-env-track-commit
    __tue_env_track_revert_vars
    [[ "${TUE_TEST_RMW}" == "rmw_fastrtps_cpp" ]]
}

@test "revert: an added scalar is unset" {
    _tue-env-track-begin
    export TUE_TEST_NEW=1
    _tue-env-track-commit
    __tue_env_track_revert_vars
    [[ -z "${TUE_TEST_NEW+set}" ]]
}

@test "revert: a variable the environment removed is restored" {
    export TUE_TEST_GONE=mine
    _tue-env-track-begin
    unset TUE_TEST_GONE
    _tue-env-track-commit
    __tue_env_track_revert_vars
    [[ "${TUE_TEST_GONE}" == "mine" ]]
}

@test "revert: the user's later value is kept" {
    _tue-env-track-begin
    export TUE_TEST_RMW=rmw_cyclonedds_cpp
    _tue-env-track-commit
    export TUE_TEST_RMW=rmw_connext
    __tue_env_track_revert_vars
    [[ "${TUE_TEST_RMW}" == "rmw_connext" ]]
}

@test "revert: keeping the user's value prints one note naming it" {
    _tue-env-track-begin
    export TUE_TEST_RMW=rmw_cyclonedds_cpp
    _tue-env-track-commit
    export TUE_TEST_RMW=rmw_connext
    run __tue_env_track_revert_vars
    [[ "${status}" -eq 0 ]]
    [[ "${output}" == *"kept your value for TUE_TEST_RMW"* ]]
}

@test "revert: an array and its export attribute survive the round trip" {
    TUE_TEST_ARR=(one "two three")
    export TUE_TEST_X=original
    _tue-env-track-begin
    TUE_TEST_ARR=(replaced)
    export TUE_TEST_X=fromenv
    _tue-env-track-commit
    __tue_env_track_revert_vars
    [[ "${TUE_TEST_ARR[1]}" == "two three" ]]
    [[ "${#TUE_TEST_ARR[@]}" -eq 2 ]]
    [[ "$(declare -p TUE_TEST_X)" == 'declare -x TUE_TEST_X="original"' ]]
}

@test "revert: a value holding a newline and a quote is restored verbatim" {
    TUE_TEST_NASTY=$'a\nb:c d'\''e'
    local __tue_env_want="${TUE_TEST_NASTY}"
    _tue-env-track-begin
    TUE_TEST_NASTY=changed
    _tue-env-track-commit
    __tue_env_track_revert_vars
    [[ "${TUE_TEST_NASTY}" == "${__tue_env_want}" ]]
}

@test "revert: a variable added then extended is unset when every recorded entry goes" {
    # Reaches the extended branch's unset line: the merge promotes add-then-extend to `extended` while
    # keeping the original absent pre-load state, so stripping every recorded entry leaves nothing and
    # the variable must go away rather than become an empty string.
    _tue-env-track-begin
    export TUE_TEST_PP="/one"
    _tue-env-track-commit
    _tue-env-track-begin
    export TUE_TEST_PP="/two:${TUE_TEST_PP}"
    _tue-env-track-commit
    [[ "${__TUE_ENV_LEDGER_VAR[TUE_TEST_PP]}" == "extended" ]]
    [[ "${__TUE_ENV_LEDGER_VAR_PRE[TUE_TEST_PP]}" == "" ]]
    __tue_env_track_revert_vars
    [[ -z "${TUE_TEST_PP+set}" ]]
}

@test "revert: on a tie between two identical entries, the higher position is removed" {
    # The environment appended a second X at index 3; the user then prepended Y, shifting the
    # original X to index 2 and the environment's added X to index 4 - both now distance 1 from the
    # recorded index. The higher position must go, leaving the user's original X in place.
    export TUE_TEST_LIST="A:X:B"
    _tue-env-track-begin
    export TUE_TEST_LIST="A:X:B:X"
    _tue-env-track-commit
    export TUE_TEST_LIST="Y:A:X:B:X"
    __tue_env_track_revert_vars
    [[ "${TUE_TEST_LIST}" == "Y:A:X:B" ]]
}

@test "revert: an entry the user already removed a duplicate of is not eaten again" {
    # The environment appended a second X (recorded index 3), but the user reduced the value back to
    # exactly its pre-load form before the revert runs. The count guard must see that only as many X
    # copies remain as the pre-load value already held, and leave it alone.
    export TUE_TEST_LIST="A:X:B"
    _tue-env-track-begin
    export TUE_TEST_LIST="A:X:B:X"
    _tue-env-track-commit
    export TUE_TEST_LIST="A:X:B"
    __tue_env_track_revert_vars
    [[ "${TUE_TEST_LIST}" == "A:X:B" ]]
}

@test "revert: a trailing empty field survives the round trip" {
    export TUE_TEST_LIST="/usr/bin:"
    _tue-env-track-begin
    export TUE_TEST_LIST="/new:${TUE_TEST_LIST}"
    _tue-env-track-commit
    __tue_env_track_revert_vars
    [[ "${TUE_TEST_LIST}" == "/usr/bin:" ]]
}

@test "revert: a leading empty field survives the round trip" {
    export TUE_TEST_LIST=":/usr/bin"
    _tue-env-track-begin
    export TUE_TEST_LIST="/new:${TUE_TEST_LIST}"
    _tue-env-track-commit
    __tue_env_track_revert_vars
    [[ "${TUE_TEST_LIST}" == ":/usr/bin" ]]
}

@test "revert: a scalar the environment turned into an array is restored with no leftover element" {
    export TUE_TEST_SC="original"
    _tue-env-track-begin
    TUE_TEST_SC=(one two)
    _tue-env-track-commit
    __tue_env_track_revert_vars
    [[ "$(declare -p TUE_TEST_SC)" == 'declare -x TUE_TEST_SC="original"' ]]
}

@test "revert: an extended variable turned into an associative array is kept, and no command in a key runs" {
    local __tue_env_marker
    __tue_env_marker="$(mktemp -u)"
    export TUE_TEST_LIST="/usr/bin"
    _tue-env-track-begin
    export TUE_TEST_LIST="/new:${TUE_TEST_LIST}"
    _tue-env-track-commit
    declare -gA TUE_TEST_LIST=(["\$(touch ${__tue_env_marker})"]="x")
    run __tue_env_track_revert_vars
    [[ "${output}" == *"kept your value for TUE_TEST_LIST"* ]]
    [[ ! -e "${__tue_env_marker}" ]]
    rm -f "${__tue_env_marker}"
}
