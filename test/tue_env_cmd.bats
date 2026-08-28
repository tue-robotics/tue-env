load helpers/track
load helpers/env

setup() {
    tue_track_setup
    # Before sourcing tue-env.bash, not after: the inherited TUE_ENV would make the "no environment
    # is active" test fail, and cleaning afterwards would unset the `tue-env` this test needs.
    tue_env_clean_shell
    # shellcheck source=/dev/null
    source "${TUE_TRACK_REPO_ROOT}/setup/tue-env.bash"
    TUE_DIR="${BATS_TEST_TMPDIR}/tue"
    export TUE_DIR
}

@test "deactivate: a non-empty ledger is applied and the virtualenv deactivate is not called" {
    deactivate() {
        echo "DEACTIVATE RAN"
        return 1
    }
    export VIRTUAL_ENV="${BATS_TEST_TMPDIR}/venv"
    _tue-env-track-begin
    export TUE_TEST_NEW=1
    _tue-env-track-commit
    run _tue-env-deactivate-current-env
    [[ "${status}" -eq 0 ]]
    [[ "${output}" != *"DEACTIVATE RAN"* ]]
}

@test "deactivate: an empty ledger falls back to the old heuristic" {
    export TUE_ENV=fake
    export TUE_ENV_DIR="${BATS_TEST_TMPDIR}/fake"
    export ROS_DISTRO=jazzy
    export AMENT_PREFIX_PATH=/opt/ros/jazzy
    _tue-env-deactivate-current-env
    [[ -z "${TUE_ENV+set}" ]]
    [[ -z "${TUE_ENV_DIR+set}" ]]
    [[ -z "${ROS_DISTRO+set}" ]]
    [[ -z "${AMENT_PREFIX_PATH+set}" ]]
}

@test "changes: reports what the load added" {
    export TUE_ENV=fake
    _tue-env-track-begin
    export TUE_TEST_RMW=rmw_cyclonedds_cpp
    _tue-env-track-commit
    run tue-env changes
    [[ "${status}" -eq 0 ]]
    [[ "${output}" == *"added   TUE_TEST_RMW=rmw_cyclonedds_cpp"* ]]
}

@test "changes: says so when there is nothing tracked" {
    export TUE_ENV=fake
    run tue-env changes
    [[ "${status}" -eq 1 ]]
    [[ "${output}" == *"no tracked changes"* ]]
}

@test "changes: refuses when no environment is active" {
    run tue-env changes
    [[ "${status}" -eq 1 ]]
    [[ "${output}" == *"No environment is currently active"* ]]
}

@test "deactivate: refuses only when neither the marker nor the ledger says anything is loaded" {
    # The other half of the gate below: with no TUE_ENV and an empty ledger there really is nothing
    # to unload, and the refusal has to stay.
    run tue-env deactivate
    [[ "${status}" -eq 1 ]]
    [[ "${output}" == *"No environment is currently active"* ]]
}

@test "deactivate: a ledger is still unloaded after the user unset TUE_ENV" {
    # The TUE_ENV marker predates the ledger and used to be the only gate, so unsetting one variable
    # by hand disabled the whole feature - while the load's other forty-odd variables, its functions
    # and its aliases were all still in the shell, and the ledger still knew how to undo them.
    # Unsetting a variable by hand is exactly what change tracking exists to tolerate.
    export TUE_ENV=fake
    _tue-env-track-begin
    export TUE_TEST_RMW=rmw_cyclonedds_cpp
    tue_test_fn() {
        echo fromenv
    }
    _tue-env-track-commit
    unset TUE_ENV
    tue-env deactivate
    [[ -z "${TUE_TEST_RMW+set}" ]]
    [[ -z "$(declare -F tue_test_fn)" ]]
    __tue_env_track_empty
}

@test "changes: an unset TUE_ENV does not hide what the load did" {
    export TUE_ENV=fake
    _tue-env-track-begin
    export TUE_TEST_RMW=rmw_cyclonedds_cpp
    _tue-env-track-commit
    unset TUE_ENV
    run tue-env changes
    [[ "${status}" -eq 0 ]]
    [[ "${output}" == *"added   TUE_TEST_RMW=rmw_cyclonedds_cpp"* ]]
}

@test "deactivate --dry-run: an unset TUE_ENV does not hide what the unload would do" {
    # `changes` and `deactivate --dry-run` gate on the same thing, so they answer alike.
    export TUE_ENV=fake
    _tue-env-track-begin
    export TUE_TEST_RMW=rmw_cyclonedds_cpp
    _tue-env-track-commit
    unset TUE_ENV
    run tue-env deactivate --dry-run
    [[ "${status}" -eq 0 ]]
    [[ "${output}" == *"would unset  TUE_TEST_RMW"* ]]
}

@test "changes: --help is accepted" {
    run tue-env changes --help
    [[ "${status}" -eq 1 ]]
    [[ "${output}" == *"Usage: tue-env changes"* ]]
}

@test "deactivate --dry-run: reports and changes nothing" {
    export TUE_ENV=fake
    _tue-env-track-begin
    export TUE_TEST_RMW=rmw_cyclonedds_cpp
    _tue-env-track-commit
    export TUE_TEST_RMW=rmw_connext
    tue-env deactivate --dry-run > /dev/null
    [[ "${TUE_TEST_RMW}" == "rmw_connext" ]]
    [[ -n "${__TUE_ENV_LEDGER_VAR[TUE_TEST_RMW]:-}" ]]
    [[ "${TUE_ENV}" == "fake" ]]
}

@test "deactivate --dry-run: names the user's value as kept" {
    export TUE_ENV=fake
    _tue-env-track-begin
    export TUE_TEST_RMW=rmw_cyclonedds_cpp
    _tue-env-track-commit
    export TUE_TEST_RMW=rmw_connext
    run tue-env deactivate --dry-run
    [[ "${status}" -eq 0 ]]
    [[ "${output}" == *"would keep   TUE_TEST_RMW=rmw_connext (changed since load)"* ]]
}

@test "deactivate: an unknown option is still rejected" {
    export TUE_ENV=fake
    run tue-env deactivate --nonsense
    [[ "${status}" -eq 1 ]]
    [[ "${output}" == *"Unknown option --nonsense"* ]]
}

@test "switch: the old environment is unloaded before the new one is loaded" {
    # envtwo points at a directory of its own. Pointing it at envone's directory, as this test used
    # to, re-loads the same target script under a new name, so everything envone added is added
    # again by envtwo and the test cannot tell an unload from no unload at all.
    tue_env_fixture envone
    tue_env_fixture_target
    tue_env_fixture_second envtwo 'export TUE_TEST_TWO_VAR=1'

    source "${TUE_TEST_DIR}/setup.bash"
    [[ "${TUE_ENV}" == "envone" ]]
    [[ "${TUE_TEST_TARGET_VAR}" == "1" ]]
    [[ -n "${BASH_ALIASES[tue_test_target_alias]:-}" ]]
    [[ -n "$(declare -F tue_test_target_fn)" ]]

    tue-env switch envtwo
    [[ "${TUE_ENV}" == "envtwo" ]]
    [[ "${TUE_ENV_DIR}" == "${TUE_TEST_ENV_DIR_TWO}" ]]
    [[ "${TUE_TEST_TWO_VAR}" == "1" ]]

    # everything envone's target script installed is gone, and so is its PATH entry
    [[ -z "${TUE_TEST_TARGET_VAR+set}" ]]
    [[ -z "${BASH_ALIASES[tue_test_target_alias]:-}" ]]
    [[ -z "$(declare -F tue_test_target_fn)" ]]
    [[ ":${PATH}:" != *":/opt/tue-test/bin:"* ]]

    # and envtwo is itself tracked, so unloading it unsets what it set
    [[ -n "${__TUE_ENV_LEDGER_VAR[TUE_ENV]:-}" ]]
    _tue-env-track-revert
    [[ -z "${TUE_ENV+set}" ]]
    [[ -z "${TUE_ENV_DIR+set}" ]]
    [[ -z "${TUE_TEST_TWO_VAR+set}" ]]
}

@test "switch: the dispatcher's own locals never reach the ledger" {
    # bash scopes locals dynamically, so the tracker's snapshots see every local of the `tue-env`
    # call they run inside. While those were unprefixed, a target script assigning a variable that
    # happened to share a name with one of them wrote to the dispatcher's copy: the ledger recorded
    # tue-env's internal state as the environment's, the unload printed a note about keeping a value
    # the user never set, and the environment's real global was never reverted at all.
    tue_env_fixture envone
    tue_env_fixture_second envtwo 'cmd=target-clobber
tue_env_dir=/target/dir
dry_run=/target/dry'

    source "${TUE_TEST_DIR}/setup.bash"
    tue-env switch envtwo

    # what the target script really set, not the dispatcher's same-named local
    [[ "${cmd}" == "target-clobber" ]]
    [[ "${tue_env_dir}" == "/target/dir" ]]
    [[ "${__TUE_ENV_LEDGER_VAR[cmd]}" == "added" ]]
    [[ "${__TUE_ENV_LEDGER_VAR[tue_env_dir]}" == "added" ]]

    run tue-env deactivate
    [[ "${status}" -eq 0 ]]
    [[ "${output}" != *"kept your value for"* ]]

    tue-env deactivate
    [[ -z "${cmd+set}" ]]
    [[ -z "${tue_env_dir+set}" ]]
    [[ -z "${dry_run+set}" ]]
}

@test "help: the top-level command list names deactivate" {
    run tue-env --help
    [[ "${status}" -eq 1 ]]
    [[ "${output}" == *"deactivate     - "* ]]
}

@test "deactivate: the fallback comment does not claim a case that cannot reach it" {
    # A comment has no behaviour to assert, so this pins the fact it states instead. The empty-ledger
    # fallback claimed to cover a non-interactive child shell that inherited `tue-env` but not the
    # ledger. It cannot: _tue-env-deactivate-current-env is not exported, so such a shell fails
    # before it. Establish that by execution, then require the comment to give that as the reason.
    export TUE_ENV=fake
    run bash --noprofile --norc -c 'tue-env deactivate'
    [[ "${status}" -eq 1 ]]
    [[ "${output}" == *"Failed to deactivate the current environment"* ]]
    # the fallback body never ran: none of what it prints is in the output
    [[ "${output}" != *"Unsetting all TUE_ENV"* ]]

    grep -q 'this function is not exported' "${TUE_TRACK_REPO_ROOT}/setup/tue-env.bash"
    [[ -z "$(grep 'Empty ledger.*non-interactive child shell' \
             "${TUE_TRACK_REPO_ROOT}/setup/tue-env.bash")" ]]
}

@test "deactivate: the spec's child-shell paragraph matches what a child shell does" {
    # The spec said a child `bash` running `tue-env deactivate` "hits the empty-ledger fallback and
    # behaves as it does today". It does not: _tue-env-deactivate-current-env is not exported, so the
    # child dies before any fallback. The README was corrected for exactly this; the spec never was.
    # Same shape as the test above - establish the behaviour by execution, then require the document
    # to describe that behaviour and not the one it cannot have.
    export TUE_ENV=fake
    run bash --noprofile --norc -c 'tue-env deactivate'
    [[ "${status}" -eq 1 ]]
    [[ "${output}" == *"Failed to deactivate the current environment"* ]]
    [[ "${output}" != *"Unsetting all TUE_ENV"* ]]

    local __tue_env_spec
    __tue_env_spec="${TUE_TRACK_REPO_ROOT}/docs/superpowers/specs/2026-08-21-env-change-tracking-design.md"
    grep -q 'never reaches the empty-ledger fallback' "${__tue_env_spec}"
    [[ -z "$(grep 'hits the empty-ledger fallback and behaves as it does today' \
             "${__tue_env_spec}")" ]]
}

@test "completion: changes and --dry-run are offered" {
    COMP_WORDS=(tue-env "")
    COMP_CWORD=1
    _tue-env
    [[ "${COMPREPLY[*]}" == *"changes"* ]]
    COMP_WORDS=(tue-env deactivate "")
    COMP_CWORD=2
    _tue-env
    [[ "${COMPREPLY[*]}" == *"--dry-run"* ]]
}

@test "switch: a ledger is still unloaded after the user unset TUE_ENV" {
    # `switch` gated the unload on the TUE_ENV marker alone, the same mistake `deactivate` used to
    # make: with the marker gone it loaded the new environment straight on top of the old one's
    # variables, aliases, functions and PATH entries instead of reverting them first.
    tue_env_fixture envone
    tue_env_fixture_target
    tue_env_fixture_second envtwo 'export TUE_TEST_TWO_VAR=1'

    source "${TUE_TEST_DIR}/setup.bash"
    unset TUE_ENV

    tue-env switch envtwo
    [[ "${TUE_ENV}" == "envtwo" ]]
    [[ "${TUE_TEST_TWO_VAR}" == "1" ]]
    [[ -z "${TUE_TEST_TARGET_VAR+set}" ]]
    [[ -z "${BASH_ALIASES[tue_test_target_alias]:-}" ]]
    [[ -z "$(declare -F tue_test_target_fn)" ]]
    [[ ":${PATH}:" != *":/opt/tue-test/bin:"* ]]
}
