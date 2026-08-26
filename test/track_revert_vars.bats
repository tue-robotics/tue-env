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

@test "revert: the environment's copy of a duplicated entry goes, not the user's, after a prepend" {
    # Every recorded index is an index into the value as it stood at the end of the load. Once the
    # user prepends entries of their own, every position shifts rightwards and "closest to the
    # recorded index" starts picking the user's own occurrence: here the X at position 3, which the
    # value already held before the load, sits exactly at the recorded index while the environment's
    # copy has moved out to 5. No entry is lost either way, which is why a count-based check passes,
    # but the wrong one goes: `:`-separated precedence changes and the environment's entry outlives
    # the unload.
    export TUE_TEST_LIST="A:X:B"
    _tue-env-track-begin
    export TUE_TEST_LIST="A:X:B:X"
    _tue-env-track-commit
    export TUE_TEST_LIST="Y:Z:${TUE_TEST_LIST}"
    __tue_env_track_revert_vars
    [[ "${TUE_TEST_LIST}" == "Y:Z:A:X:B" ]]
}

@test "revert: a PATH-shaped duplicate survives a prepend of several entries" {
    # The same thing in the shape it actually arrives in: a directory that is already on PATH and
    # that the environment appends a second copy of, with the user prepending a few entries
    # afterwards. The shift is larger than the distance between the two copies, so the nearest-index
    # rule on its own removes the copy the user started with.
    export TUE_TEST_LIST="/usr/bin:/opt/x:/bin"
    _tue-env-track-begin
    export TUE_TEST_LIST="${TUE_TEST_LIST}:/opt/x"
    _tue-env-track-commit
    export TUE_TEST_LIST="/p1:/p2:/p3:${TUE_TEST_LIST}"
    __tue_env_track_revert_vars
    [[ "${TUE_TEST_LIST}" == "/p1:/p2:/p3:/usr/bin:/opt/x:/bin" ]]
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

@test "revert: a newline in the current value's last field survives the round trip" {
    export TUE_TEST_LIST="/usr/bin"
    _tue-env-track-begin
    export TUE_TEST_LIST="/new:${TUE_TEST_LIST}"
    _tue-env-track-commit
    TUE_TEST_LIST="/new:/usr/bin:A"$'\n'"B"
    __tue_env_track_revert_vars
    [[ "${TUE_TEST_LIST}" == "/usr/bin:A"$'\n'"B" ]]
}

@test "revert: a newline in the current value's first field does not empty or unset the variable" {
    export TUE_TEST_LIST="/usr/bin"
    _tue-env-track-begin
    export TUE_TEST_LIST="/new:${TUE_TEST_LIST}"
    _tue-env-track-commit
    TUE_TEST_LIST="A"$'\n'"B:/new:/usr/bin"
    __tue_env_track_revert_vars
    [[ -n "${TUE_TEST_LIST+set}" ]]
    [[ -n "${TUE_TEST_LIST}" ]]
    [[ "${TUE_TEST_LIST}" == "A"$'\n'"B:/usr/bin" ]]
}

@test "revert: an empty field the environment appended is removed" {
    # An empty `:`-separated field means the current directory. `IFS=':' read -a`, which the
    # classification side used, silently drops a trailing empty field, so the entry was recorded as
    # no addition at all and the revert left `.` on the value for the life of the shell. The two
    # tests above only cover empty fields the USER already had, which the count guard preserves for
    # an unrelated reason, so neither of them sees this.
    export TUE_TEST_LIST="/usr/bin:/bin"
    export TUE_TEST_PP="/usr/bin"
    _tue-env-track-begin
    export TUE_TEST_LIST="${TUE_TEST_LIST}:"
    export TUE_TEST_PP="/opt/x:${TUE_TEST_PP}:"
    _tue-env-track-commit
    [[ "${__TUE_ENV_LEDGER_VAR[TUE_TEST_LIST]}" == "extended" ]]
    __tue_env_track_revert_vars
    [[ "${TUE_TEST_LIST}" == "/usr/bin:/bin" ]]
    [[ "${TUE_TEST_PP}" == "/usr/bin" ]]
}

@test "revert: unsetting a nameref the environment added leaves its target alone" {
    # Plain `unset` on a nameref follows the reference and destroys the variable it points at, which
    # the load never touched - the one thing the whole design promises cannot happen.
    export TUE_TEST_TARGET="precious user data"
    _tue-env-track-begin
    declare -gn TUE_TEST_NREF=TUE_TEST_TARGET
    _tue-env-track-commit
    [[ "${__TUE_ENV_LEDGER_VAR[TUE_TEST_NREF]}" == "added" ]]
    __tue_env_track_revert_vars
    [[ "${TUE_TEST_TARGET}" == "precious user data" ]]
    [[ -z "$(declare -p TUE_TEST_NREF 2> /dev/null)" ]]
}

@test "revert: restoring over a nameref the environment created does not write through it" {
    # Same trap on the restore path, twice over: the unset destroys the target, and the re-declare
    # then assigns the recorded value THROUGH the surviving reference, so the user's variable ends
    # up holding the restored value and the name that was restored is still a nameref.
    export TUE_TEST_TARGET="precious user data"
    export TUE_TEST_NREF="plain string"
    _tue-env-track-begin
    unset -v TUE_TEST_NREF
    declare -gn TUE_TEST_NREF=TUE_TEST_TARGET
    _tue-env-track-commit
    [[ "${__TUE_ENV_LEDGER_VAR[TUE_TEST_NREF]}" == "replaced" ]]
    __tue_env_track_revert_vars
    [[ "${TUE_TEST_TARGET}" == "precious user data" ]]
    [[ "$(declare -p TUE_TEST_NREF)" == 'declare -x TUE_TEST_NREF="plain string"' ]]
}

@test "revert: a value the environment turned into a nameref is never treated as a list" {
    # A nameref's value is the name it points at, so the entry-wise branch would hand it to
    # `printf -v`, which writes THROUGH the reference. An empty pre-load value has no entries, so a
    # one-entry nameref value is a valid subsequence extension of it and reaches exactly that.
    export TUE_TEST_TARGET="precious user data"
    TUE_TEST_LIST=""
    _tue-env-track-begin
    unset -v TUE_TEST_LIST
    declare -gn TUE_TEST_LIST=TUE_TEST_TARGET
    _tue-env-track-commit
    [[ "${__TUE_ENV_LEDGER_VAR[TUE_TEST_LIST]}" == "replaced" ]]
    __tue_env_track_revert_vars
    [[ "${TUE_TEST_TARGET}" == "precious user data" ]]
}
