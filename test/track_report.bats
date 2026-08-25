load helpers/track

setup() {
    tue_track_setup
}

@test "report: changes lists added entries, added scalars and replaced values" {
    export TUE_TEST_LIST="/usr/bin"
    PS1='\u@\h \$ '
    _tue-env-track-begin
    export TUE_TEST_LIST="/usr/lib/ccache:/opt/ros/jazzy/bin:${TUE_TEST_LIST}"
    export TUE_TEST_RMW=rmw_cyclonedds_cpp
    PS1='\u@\h git \$ '
    alias tue_test_trunk='echo trunk'
    tue_test_fn() {
        echo fn
    }
    _tue-env-track-commit
    run _tue-env-track-report changes
    [[ "${status}" -eq 0 ]]
    [[ "${output}" == *"added   TUE_TEST_LIST entries: /usr/lib/ccache, /opt/ros/jazzy/bin"* ]]
    [[ "${output}" == *"added   TUE_TEST_RMW=rmw_cyclonedds_cpp"* ]]
    [[ "${output}" == *"changed PS1 (was '\u@\h \\$ ')"* ]]
    [[ "${output}" == *"added   alias tue_test_trunk"* ]]
    [[ "${output}" == *"added   function tue_test_fn"* ]]
}

@test "report: revert names what would go, what would come back and what would stay" {
    export TUE_TEST_LIST="/usr/bin"
    PS1='\u@\h \$ '
    _tue-env-track-begin
    export TUE_TEST_LIST="/usr/lib/ccache:${TUE_TEST_LIST}"
    export TUE_TEST_RMW=rmw_cyclonedds_cpp
    PS1='\u@\h git \$ '
    alias tue_test_trunk='echo trunk'
    _tue-env-track-commit
    export TUE_TEST_RMW=rmw_connext
    run _tue-env-track-report revert
    [[ "${status}" -eq 0 ]]
    [[ "${output}" == *"would remove TUE_TEST_LIST entries: /usr/lib/ccache"* ]]
    [[ "${output}" == *"would keep   TUE_TEST_RMW=rmw_connext (changed since load)"* ]]
    [[ "${output}" == *"would restore PS1 to '\u@\h \\$ '"* ]]
    [[ "${output}" == *"would unset  alias tue_test_trunk"* ]]
}

@test "report: an array is reported without trying to render its value" {
    _tue-env-track-begin
    TUE_TEST_ARR=(a b)
    _tue-env-track-commit
    run _tue-env-track-report changes
    [[ "${output}" == *"added   TUE_TEST_ARR=(array)"* ]]
}

@test "report: several objects of one kind are grouped on one line" {
    _tue-env-track-begin
    alias tue_test_a='echo a'
    alias tue_test_b='echo b'
    _tue-env-track-commit
    run _tue-env-track-report changes
    [[ "${output}" == *"added   alias tue_test_a, alias tue_test_b"* ]]
}

@test "report: an empty ledger returns 1 and prints nothing" {
    run _tue-env-track-report changes
    [[ "${status}" -eq 1 ]]
    [[ -z "${output}" ]]
}

@test "report: a dry run mutates nothing" {
    export TUE_TEST_LIST="/usr/bin"
    _tue-env-track-begin
    export TUE_TEST_LIST="/ccache:${TUE_TEST_LIST}"
    export TUE_TEST_NEW=1
    alias tue_test_a='echo a'
    _tue-env-track-commit
    _tue-env-track-report revert > /dev/null
    [[ "${TUE_TEST_LIST}" == "/ccache:/usr/bin" ]]
    [[ "${TUE_TEST_NEW}" == "1" ]]
    [[ "${BASH_ALIASES[tue_test_a]}" == "echo a" ]]
    [[ -n "${__TUE_ENV_LEDGER_VAR[TUE_TEST_NEW]:-}" ]]
}

@test "report: entry_list does not hang on a stream with no terminating RS" {
    run timeout 2 bash -c '
        source "'"${TUE_TRACK_REPO_ROOT}"'/setup/tue-env-track.bash"
        __tue_env_track_entry_list "0${__TUE_ENV_PS}/ccache"
        printf "%s" "${__TUE_ENV_LIST}"
    '
    [[ "${status}" -eq 0 ]]
    [[ -z "${output}" ]]
}

@test "report: revert on an extended var only lists entries still worth removing" {
    export TUE_TEST_A="/usr/bin"
    export TUE_TEST_B="/usr/bin"
    _tue-env-track-begin
    export TUE_TEST_A="/ccache:/opt/ros:${TUE_TEST_A}"
    export TUE_TEST_B="/ccache:${TUE_TEST_B}"
    _tue-env-track-commit
    # The user removes one of the two entries the environment added, and unsets the other
    # variable entirely, both by hand after the load committed.
    export TUE_TEST_A="/ccache:/usr/bin"
    unset TUE_TEST_B
    run _tue-env-track-report revert
    [[ "${status}" -eq 0 ]]
    [[ "${output}" == *"would remove TUE_TEST_A entries: /ccache"* ]]
    [[ "${output}" != *"TUE_TEST_A entries: /ccache, /opt/ros"* ]]
    [[ "${output}" != *"TUE_TEST_B"* ]]
}

@test "report: revert keeps an extended value that is no longer listable" {
    export TUE_TEST_ARR2="/usr/bin"
    _tue-env-track-begin
    export TUE_TEST_ARR2="/ccache:${TUE_TEST_ARR2}"
    _tue-env-track-commit
    unset TUE_TEST_ARR2
    TUE_TEST_ARR2=(a b)
    run _tue-env-track-report revert
    [[ "${status}" -eq 0 ]]
    [[ "${output}" == *"would keep   TUE_TEST_ARR2=(array) (changed since load)"* ]]
}

@test "report: an unrecognized mode is rejected instead of acting like revert" {
    _tue-env-track-begin
    export TUE_TEST_NEW=1
    _tue-env-track-commit
    run _tue-env-track-report chnages
    [[ "${status}" -eq 2 ]]
    [[ "${output}" != *"would"* ]]
}

@test "report: a missing mode argument fails cleanly instead of tripping set -u" {
    run bash -c '
        set -u
        source "'"${TUE_TRACK_REPO_ROOT}"'/setup/tue-env-track.bash"
        _tue-env-track-begin
        export TUE_TEST_NEW=1
        _tue-env-track-commit
        _tue-env-track-report
    '
    [[ "${status}" -eq 2 ]]
    [[ "${output}" != *"unbound variable"* ]]
}

@test "report: changes reports removed variables separately from changed ones" {
    export TUE_TEST_GONE=oldvalue
    _tue-env-track-begin
    unset TUE_TEST_GONE
    _tue-env-track-commit
    run _tue-env-track-report changes
    [[ "${status}" -eq 0 ]]
    [[ "${output}" == *"removed TUE_TEST_GONE (was 'oldvalue')"* ]]
    [[ "${output}" != *"changed TUE_TEST_GONE"* ]]
}

@test "report: changes groups changed and removed aliases and functions separately" {
    alias tue_test_alias='echo old'
    tue_test_fn() {
        echo old
    }
    _tue-env-track-begin
    alias tue_test_alias='echo new'
    unset -f tue_test_fn
    _tue-env-track-commit
    run _tue-env-track-report changes
    [[ "${status}" -eq 0 ]]
    [[ "${output}" == *"changed alias tue_test_alias"* ]]
    [[ "${output}" == *"removed function tue_test_fn"* ]]
}

@test "report: revert restores changed and removed objects, keeping ones drifted since load" {
    alias tue_test_alias='echo old'
    alias tue_test_gone='echo bye'
    tue_test_fn() {
        echo old
    }
    _tue-env-track-begin
    alias tue_test_alias='echo new'
    unalias tue_test_gone
    tue_test_fn() {
        echo new
    }
    _tue-env-track-commit
    alias tue_test_alias='echo drifted'
    run _tue-env-track-report revert
    [[ "${status}" -eq 0 ]]
    [[ "${output}" == *"would keep   alias tue_test_alias (changed since load)"* ]]
    [[ "${output}" == *"would restore alias tue_test_gone"* ]]
    [[ "${output}" == *"would restore function tue_test_fn"* ]]
}

@test "report: a completion registration is reported in both modes" {
    _tue-env-track-begin
    complete -W "foo bar" tue_test_cmd
    _tue-env-track-commit
    run _tue-env-track-report changes
    [[ "${status}" -eq 0 ]]
    [[ "${output}" == *"added   completion for tue_test_cmd"* ]]
    run _tue-env-track-report revert
    [[ "${status}" -eq 0 ]]
    [[ "${output}" == *"would unset  completion for tue_test_cmd"* ]]
}
