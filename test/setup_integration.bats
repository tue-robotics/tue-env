load helpers/track
load helpers/env

setup() {
    tue_track_setup
}

@test "setup: a full load is tracked and unloading undoes exactly it" {
    tue_env_fixture testenv
    tue_env_fixture_target
    source "${TUE_TEST_DIR}/setup.bash" || true

    [[ "${TUE_ENV}" == "testenv" ]]
    [[ "${TUE_TEST_USER_SETUP}" == "1" ]]
    [[ ":${PATH}:" == *":/opt/tue-test/bin:"* ]]

    _tue-env-track-revert

    [[ -z "${TUE_ENV+set}" ]]
    [[ -z "${TUE_ENV_DIR+set}" ]]
    [[ -z "${TUE_ENV_TARGETS_DIR+set}" ]]
    [[ -z "${TUE_TEST_USER_SETUP+set}" ]]
    [[ -z "${TUE_TEST_TARGET_VAR+set}" ]]
    [[ -z "${BASH_ALIASES[tue_test_target_alias]:-}" ]]
    [[ -z "$(declare -F tue_test_target_fn)" ]]
    [[ -z "$(declare -F tue-make)" ]]
    [[ -z "$(complete -p tue-make 2> /dev/null)" ]]
    [[ ":${PATH}:" != *":/opt/tue-test/bin:"* ]]

    # Everything the bootstrap did is outside the tracked span, so it survives
    [[ "${TUE_DIR}" == "${TUE_TEST_DIR}" ]]
    [[ ":${PATH}:" == *":${TUE_TEST_DIR}/bin:"* ]]
    declare -F tue-env > /dev/null
    declare -F _tue-check-env-vars > /dev/null
}

@test "setup: unloading brings back the prompt the user started with" {
    tue_env_fixture testenv
    tue_env_fixture_venv
    tue_env_fixture_target
    PS1='\u@\h \$ '
    local __tue_env_want="${PS1}"

    source "${TUE_TEST_DIR}/setup.bash" || true
    [[ "${PS1}" != "${__tue_env_want}" ]]
    [[ -n "${VIRTUAL_ENV}" ]]
    declare -F deactivate > /dev/null

    _tue-env-track-revert

    # The venv saved the pristine prompt in _OLD_VIRTUAL_PS1 and the target then clobbered it, so the
    # venv's own deactivate would have restored the wrong one. The ledger keeps the pre-load state
    # from first sight, so the prompt comes back pristine.
    [[ "${PS1}" == "${__tue_env_want}" ]]
    [[ -z "$(declare -F deactivate)" ]]
    [[ -z "${VIRTUAL_ENV+set}" ]]
    [[ -z "${_OLD_VIRTUAL_PS1+set}" ]]
    [[ -z "${_OLD_VIRTUAL_PATH+set}" ]]
}

@test "setup: a load that fails after exporting TUE_ENV is still tracked" {
    tue_env_fixture testenv
    rmdir "${TUE_TEST_ENV_DIR}/.env/targets"
    source "${TUE_TEST_DIR}/setup.bash" || true

    [[ "${__TUE_ENV_LEDGER_VAR[TUE_ENV]}" == "added" ]]
    _tue-env-track-revert
    [[ -z "${TUE_ENV+set}" ]]
}

@test "setup: re-sourcing setup.bash keeps one ledger and the original pre-load state" {
    tue_env_fixture testenv
    tue_env_fixture_target
    PS1='\u@\h \$ '
    local __tue_env_want="${PS1}"

    source "${TUE_TEST_DIR}/setup.bash" || true
    source "${TUE_TEST_DIR}/setup.bash" || true

    [[ "${__TUE_ENV_TRACK_DEPTH}" -eq 0 ]]
    _tue-env-track-revert
    [[ "${PS1}" == "${__tue_env_want}" ]]
    [[ -z "${TUE_ENV+set}" ]]
    # /opt/tue-test/bin was added by both loads, and both occurrences have to go
    [[ ":${PATH}:" != *":/opt/tue-test/bin:"* ]]
}

@test "setup: a change made by hand between two loads survives the unload" {
    tue_env_fixture testenv
    tue_env_fixture_target
    source "${TUE_TEST_DIR}/setup.bash" || true
    export PATH="/opt/mine/bin:${PATH}"
    export TUE_TEST_OF_MINE=1
    source "${TUE_TEST_DIR}/setup.bash" || true

    _tue-env-track-revert
    [[ ":${PATH}:" == *":/opt/mine/bin:"* ]]
    [[ "${TUE_TEST_OF_MINE}" == "1" ]]
}
