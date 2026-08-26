load helpers/track

setup() {
    tue_track_setup
}

@test "diff: a new function is added and its export attribute recorded" {
    tue_track_snapshot PRE
    tue_test_fn() {
        echo new
    }
    export -f tue_test_fn
    tue_track_snapshot POST
    __tue_env_track_diff_funcs
    [[ "${__TUE_ENV_LEDGER_FUNC[tue_test_fn]}" == "added" ]]
    [[ "${__TUE_ENV_LEDGER_FUNC_PRE[tue_test_fn]}" == "" ]]
    [[ "${__TUE_ENV_LEDGER_FUNC_POST[tue_test_fn]}" == "$(declare -f tue_test_fn)" ]]
    [[ "${__TUE_ENV_LEDGER_FUNC_XPRE[tue_test_fn]}" == "" ]]
}

@test "diff: a redefined function is replaced and the pre-load export flag kept" {
    tue_test_fn() {
        echo original
    }
    export -f tue_test_fn
    tue_track_snapshot PRE
    unset -f tue_test_fn
    tue_test_fn() {
        echo replaced
    }
    tue_track_snapshot POST
    __tue_env_track_diff_funcs
    [[ "${__TUE_ENV_LEDGER_FUNC[tue_test_fn]}" == "replaced" ]]
    [[ "${__TUE_ENV_LEDGER_FUNC_XPRE[tue_test_fn]}" == "x" ]]
    [[ "${__TUE_ENV_LEDGER_FUNC_PRE[tue_test_fn]}" == *original* ]]
}

@test "diff: a removed function is removed" {
    tue_test_fn() {
        echo gone
    }
    tue_track_snapshot PRE
    unset -f tue_test_fn
    tue_track_snapshot POST
    __tue_env_track_diff_funcs
    [[ "${__TUE_ENV_LEDGER_FUNC[tue_test_fn]}" == "removed" ]]
    [[ "${__TUE_ENV_LEDGER_FUNC_POST[tue_test_fn]}" == "" ]]
}

@test "diff: an untouched function produces no entry" {
    tue_test_fn() {
        echo same
    }
    tue_track_snapshot PRE
    tue_track_snapshot POST
    __tue_env_track_diff_funcs
    [[ -z "${__TUE_ENV_LEDGER_FUNC[tue_test_fn]:-}" ]]
}

@test "diff: an added alias, a changed alias and a removed alias" {
    alias tue_test_keep='echo keep'
    alias tue_test_gone='echo gone'
    tue_track_snapshot PRE
    alias tue_test_new='echo new'
    alias tue_test_keep='echo changed'
    unalias tue_test_gone
    tue_track_snapshot POST
    __tue_env_track_diff_simple ALIAS
    [[ "${__TUE_ENV_LEDGER_ALIAS[tue_test_new]}" == "added" ]]
    [[ "${__TUE_ENV_LEDGER_ALIAS[tue_test_keep]}" == "replaced" ]]
    [[ "${__TUE_ENV_LEDGER_ALIAS_PRE[tue_test_keep]}" == "echo keep" ]]
    [[ "${__TUE_ENV_LEDGER_ALIAS[tue_test_gone]}" == "removed" ]]
}

@test "diff: an alias whose value is empty is not mistaken for an absent one" {
    # `alias foo=` is an ordinary alias holding the empty string. While presence was tested with
    # `${arr[key]:-}`, an empty alias the user already had was classified `added` - and so destroyed
    # by the unload - while an empty alias the environment created, and an empty one it removed, were
    # not recorded at all and outlived it.
    alias tue_test_empty=''
    alias tue_test_gone=''
    tue_track_snapshot PRE
    alias tue_test_empty='echo fromenv'
    unalias tue_test_gone
    alias tue_test_new=''
    tue_track_snapshot POST
    __tue_env_track_diff_simple ALIAS
    [[ "${__TUE_ENV_LEDGER_ALIAS[tue_test_empty]}" == "replaced" ]]
    [[ "${__TUE_ENV_LEDGER_ALIAS_PRE[tue_test_empty]}" == "" ]]
    [[ "${__TUE_ENV_LEDGER_ALIAS[tue_test_gone]}" == "removed" ]]
    [[ "${__TUE_ENV_LEDGER_ALIAS[tue_test_new]}" == "added" ]]
    [[ "${__TUE_ENV_LEDGER_ALIAS_POST[tue_test_new]}" == "" ]]
}

@test "diff: an added and a removed completion" {
    tue_test_complete() {
        COMPREPLY=()
    }
    complete -F tue_test_complete tue-test-old
    tue_track_snapshot PRE
    complete -o nospace -F tue_test_complete tue-test-new
    complete -r tue-test-old
    tue_track_snapshot POST
    __tue_env_track_diff_simple COMPLETE
    [[ "${__TUE_ENV_LEDGER_COMPLETE[tue-test-new]}" == "added" ]]
    [[ "${__TUE_ENV_LEDGER_COMPLETE_POST[tue-test-new]}" == \
       "complete -o nospace -F tue_test_complete tue-test-new" ]]
    [[ "${__TUE_ENV_LEDGER_COMPLETE[tue-test-old]}" == "removed" ]]
}

@test "diff: a function whose body is unchanged but export flag changed is replaced" {
    tue_test_fn() {
        echo same
    }
    tue_track_snapshot PRE
    export -f tue_test_fn
    tue_track_snapshot POST
    __tue_env_track_diff_funcs
    [[ "${__TUE_ENV_LEDGER_FUNC[tue_test_fn]}" == "replaced" ]]
    [[ "${__TUE_ENV_LEDGER_FUNC_XPRE[tue_test_fn]}" == "" ]]
    [[ "${__TUE_ENV_LEDGER_FUNC_PRE[tue_test_fn]}" == "${__TUE_ENV_LEDGER_FUNC_POST[tue_test_fn]}" ]]
}
