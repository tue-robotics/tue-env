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

@test "switch: the new TUE_ENV is tracked, so a later deactivate unsets it" {
    tue_env_fixture envone
    printf '%s\n' "${TUE_TEST_ENV_DIR}" > "${TUE_TEST_DIR}/user/envs/envtwo"
    source "${TUE_TEST_DIR}/setup.bash" || true
    [[ "${TUE_ENV}" == "envone" ]]

    tue-env switch envtwo || true
    [[ "${TUE_ENV}" == "envtwo" ]]
    [[ -n "${__TUE_ENV_LEDGER_VAR[TUE_ENV]:-}" ]]

    _tue-env-track-revert
    [[ -z "${TUE_ENV+set}" ]]
    [[ -z "${TUE_ENV_DIR+set}" ]]
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
