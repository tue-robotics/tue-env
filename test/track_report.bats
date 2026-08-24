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
