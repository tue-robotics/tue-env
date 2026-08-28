load helpers/track

setup() {
    tue_track_setup
}

@test "diff: a new variable is added" {
    tue_track_snapshot PRE
    export TUE_TEST_NEW=1
    tue_track_snapshot POST
    __tue_env_track_diff_vars
    [[ "${__TUE_ENV_LEDGER_VAR[TUE_TEST_NEW]}" == "added" ]]
    [[ "${__TUE_ENV_LEDGER_VAR_PRE[TUE_TEST_NEW]}" == "" ]]
    [[ "${__TUE_ENV_LEDGER_VAR_POST[TUE_TEST_NEW]}" == 'declare -x TUE_TEST_NEW="1"' ]]
}

@test "diff: an unset variable is removed" {
    export TUE_TEST_GONE=1
    tue_track_snapshot PRE
    unset TUE_TEST_GONE
    tue_track_snapshot POST
    __tue_env_track_diff_vars
    [[ "${__TUE_ENV_LEDGER_VAR[TUE_TEST_GONE]}" == "removed" ]]
    [[ "${__TUE_ENV_LEDGER_VAR_PRE[TUE_TEST_GONE]}" == 'declare -x TUE_TEST_GONE="1"' ]]
    [[ "${__TUE_ENV_LEDGER_VAR_POST[TUE_TEST_GONE]}" == "" ]]
}

@test "diff: an untouched variable produces no entry" {
    export TUE_TEST_SAME=1
    tue_track_snapshot PRE
    tue_track_snapshot POST
    __tue_env_track_diff_vars
    [[ -z "${__TUE_ENV_LEDGER_VAR[TUE_TEST_SAME]:-}" ]]
}

@test "diff: a prepended list value is extended, with the indices of the added entries" {
    export TUE_TEST_LIST="/usr/bin:/bin"
    tue_track_snapshot PRE
    export TUE_TEST_LIST="/usr/lib/ccache:/opt/ros/jazzy/bin:${TUE_TEST_LIST}"
    tue_track_snapshot POST
    __tue_env_track_diff_vars
    [[ "${__TUE_ENV_LEDGER_VAR[TUE_TEST_LIST]}" == "extended" ]]
    [[ "$(tue_track_added TUE_TEST_LIST)" == "0=/usr/lib/ccache,1=/opt/ros/jazzy/bin" ]]
}

@test "diff: an appended duplicate entry records the new index" {
    export TUE_TEST_LIST="A:X:B"
    tue_track_snapshot PRE
    export TUE_TEST_LIST="A:X:B:X"
    tue_track_snapshot POST
    __tue_env_track_diff_vars
    [[ "${__TUE_ENV_LEDGER_VAR[TUE_TEST_LIST]}" == "extended" ]]
    [[ "$(tue_track_added TUE_TEST_LIST)" == "3=X" ]]
}

@test "diff: a reordered list is replaced, not extended" {
    export TUE_TEST_LIST="A:B"
    tue_track_snapshot PRE
    export TUE_TEST_LIST="B:A"
    tue_track_snapshot POST
    __tue_env_track_diff_vars
    [[ "${__TUE_ENV_LEDGER_VAR[TUE_TEST_LIST]}" == "replaced" ]]
}

@test "diff: a changed scalar is replaced" {
    export TUE_TEST_RMW=rmw_fastrtps_cpp
    tue_track_snapshot PRE
    export TUE_TEST_RMW=rmw_cyclonedds_cpp
    tue_track_snapshot POST
    __tue_env_track_diff_vars
    [[ "${__TUE_ENV_LEDGER_VAR[TUE_TEST_RMW]}" == "replaced" ]]
    [[ "${__TUE_ENV_LEDGER_VAR_PRE[TUE_TEST_RMW]}" == 'declare -x TUE_TEST_RMW="rmw_fastrtps_cpp"' ]]
}

@test "diff: a changed array is replaced, never extended" {
    TUE_TEST_ARR=(a)
    tue_track_snapshot PRE
    TUE_TEST_ARR=(a b)
    tue_track_snapshot POST
    __tue_env_track_diff_vars
    [[ "${__TUE_ENV_LEDGER_VAR[TUE_TEST_ARR]}" == "replaced" ]]
    [[ "${__TUE_ENV_LEDGER_VAR_POST[TUE_TEST_ARR]}" == "$(declare -p TUE_TEST_ARR)" ]]
}

@test "diff: a value holding a newline is replaced, never extended" {
    export TUE_TEST_NL="a:b"
    tue_track_snapshot PRE
    export TUE_TEST_NL=$'a:b\nc'
    tue_track_snapshot POST
    __tue_env_track_diff_vars
    [[ "${__TUE_ENV_LEDGER_VAR[TUE_TEST_NL]}" == "replaced" ]]
}

@test "diff: a readonly variable is not tracked at all" {
    tue_track_snapshot PRE
    readonly TUE_TEST_RO=1
    tue_track_snapshot POST
    __tue_env_track_diff_vars
    [[ -z "${__TUE_ENV_LEDGER_VAR[TUE_TEST_RO]:-}" ]]
}

@test "diff: an added scalar also records its entries, for a later merge" {
    tue_track_snapshot PRE
    export TUE_TEST_PP="/a:/b"
    tue_track_snapshot POST
    __tue_env_track_diff_vars
    [[ "${__TUE_ENV_LEDGER_VAR[TUE_TEST_PP]}" == "added" ]]
    [[ "$(tue_track_added TUE_TEST_PP)" == "0=/a,1=/b" ]]
}

@test "diff: an array replaced by a scalar is replaced, never extended" {
    TUE_TEST_ARR2=(a b)
    tue_track_snapshot PRE
    unset TUE_TEST_ARR2
    TUE_TEST_ARR2=a
    tue_track_snapshot POST
    __tue_env_track_diff_vars
    [[ "${__TUE_ENV_LEDGER_VAR[TUE_TEST_ARR2]}" == "replaced" ]]
}

@test "diff: an array replaced by a list-shaped scalar is replaced, never extended" {
    TUE_TEST_ARR3=(/p)
    tue_track_snapshot PRE
    unset TUE_TEST_ARR3
    export TUE_TEST_ARR3="/new:/p"
    tue_track_snapshot POST
    __tue_env_track_diff_vars
    [[ "${__TUE_ENV_LEDGER_VAR[TUE_TEST_ARR3]}" == "replaced" ]]
    [[ -z "$(tue_track_added TUE_TEST_ARR3)" ]]
}

@test "diff: an associative array replaced by a scalar is replaced" {
    declare -gA TUE_TEST_ARR4=([k]=v)
    tue_track_snapshot PRE
    unset TUE_TEST_ARR4
    TUE_TEST_ARR4=v
    tue_track_snapshot POST
    __tue_env_track_diff_vars
    [[ "${__TUE_ENV_LEDGER_VAR[TUE_TEST_ARR4]}" == "replaced" ]]
}

@test "diff: an associative array key cannot assign a variable through eval" {
    declare -gA TUE_TEST_ARR5=(['x[TUE_TEST_PWNED=99]']=v)
    tue_track_snapshot PRE
    unset TUE_TEST_ARR5
    TUE_TEST_ARR5=v
    tue_track_snapshot POST
    __tue_env_track_diff_vars
    [[ -z "${TUE_TEST_PWNED:-}" ]]
}

@test "diff: an entry inserted in the middle is extended at its own index" {
    export TUE_TEST_LIST="A:B"
    tue_track_snapshot PRE
    export TUE_TEST_LIST="A:X:B"
    tue_track_snapshot POST
    __tue_env_track_diff_vars
    [[ "${__TUE_ENV_LEDGER_VAR[TUE_TEST_LIST]}" == "extended" ]]
    [[ "$(tue_track_added TUE_TEST_LIST)" == "1=X" ]]
}

@test "diff: an entry that only shares a prefix is replaced, not extended" {
    export TUE_TEST_LIST="/opt/ros"
    tue_track_snapshot PRE
    export TUE_TEST_LIST="/opt/ros/bin"
    tue_track_snapshot POST
    __tue_env_track_diff_vars
    [[ "${__TUE_ENV_LEDGER_VAR[TUE_TEST_LIST]}" == "replaced" ]]
}
