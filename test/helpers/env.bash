# shellcheck shell=bash
#
# Builds a throw-away tue-env installation under BATS_TEST_TMPDIR so that the integration tests can
# source a real setup.bash without touching the user's own ~/.tue/user tree.
#
# Requires helpers/track to be loaded first, for TUE_TRACK_REPO_ROOT.

function tue_env_clean_shell
{
    # The shell running the tests usually has a tue-env environment of its own loaded, and exports it
    # into every child process: on the machine this was written on a child inherits TUE_ENV, TUE_DIR
    # and sixteen exported tue functions. Left in place they would land in the pre-load snapshot, so
    # `tue-make` would be classified `replaced` instead of `added`, and the inherited TUE_ENV would
    # decide which environment the fixture loads. Clear them, but keep the harness's own TUE_TRACK_*
    # and TUE_TEST_* variables.
    local __tue_env_n __tue_env_d1 __tue_env_d2

    for __tue_env_n in $(compgen -v TUE_) $(compgen -v ROS_) $(compgen -v VIRTUAL_ENV) \
                       $(compgen -v _OLD_VIRTUAL)
    do
        case "${__tue_env_n}" in
            TUE_TRACK_* | TUE_TEST_* )
                continue ;;
        esac
        unset "${__tue_env_n}"
    done
    unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH RMW_IMPLEMENTATION

    # Only exported functions: those are the ones that came in from the parent shell. The tracker's
    # own functions are not exported, so this cannot remove them.
    while read -r __tue_env_d1 __tue_env_d2 __tue_env_n
    do
        case "${__tue_env_n}" in
            tue-* | _tue-* | __tue-* | _git_* )
                unset -f "${__tue_env_n}" ;;
        esac
    done <<< "$(declare -Fx)"

    return 0
}

function tue_env_fixture
{
    # $1: environment name. Sets TUE_TEST_DIR (the TUE_DIR copy) and TUE_TEST_ENV_DIR.
    # The name goes into user/config/default_env rather than into TUE_ENV, so that setup.bash exports
    # TUE_ENV itself, inside the tracked span, exactly as it does for a real interactive shell.
    tue_env_clean_shell
    TUE_TEST_DIR="${BATS_TEST_TMPDIR}/tue"
    TUE_TEST_ENV_DIR="${BATS_TEST_TMPDIR}/env"
    mkdir -p "${TUE_TEST_DIR}/setup" "${TUE_TEST_DIR}/user/envs" "${TUE_TEST_DIR}/user/config" \
             "${TUE_TEST_ENV_DIR}/.env/targets" "${TUE_TEST_ENV_DIR}/.env/setup"
    cp "${TUE_TRACK_REPO_ROOT}/setup.bash" "${TUE_TEST_DIR}/"
    cp "${TUE_TRACK_REPO_ROOT}"/setup/*.bash "${TUE_TEST_DIR}/setup/"
    printf '%s\n' "${TUE_TEST_ENV_DIR}" > "${TUE_TEST_DIR}/user/envs/$1"
    printf '%s\n' "$1" > "${TUE_TEST_DIR}/user/config/default_env"
    printf 'export TUE_TEST_USER_SETUP=1\n' > "${TUE_TEST_ENV_DIR}/.env/setup/user_setup.bash"
    return 0
}

function tue_env_fixture_venv
{
    # A stand-in for the real virtualenv activate script: it makes the changes that matter to the
    # ledger, without the cost of creating a virtualenv.
    mkdir -p "${TUE_TEST_ENV_DIR}/.env/venv/bin"
    cat > "${TUE_TEST_ENV_DIR}/.env/venv/bin/activate" << 'ACTIVATE'
VIRTUAL_ENV="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
export VIRTUAL_ENV
_OLD_VIRTUAL_PATH="${PATH}"
PATH="${VIRTUAL_ENV}/bin:${PATH}"
export PATH
_OLD_VIRTUAL_PS1="${PS1:-}"
PS1="(venv) ${PS1:-}"
VIRTUAL_ENV_PROMPT="(venv) "
export VIRTUAL_ENV_PROMPT
deactivate() {
    PATH="${_OLD_VIRTUAL_PATH}"
    export PATH
    PS1="${_OLD_VIRTUAL_PS1}"
    unset _OLD_VIRTUAL_PATH _OLD_VIRTUAL_PS1 VIRTUAL_ENV VIRTUAL_ENV_PROMPT
    unset -f deactivate
}
ACTIVATE
    return 0
}

function tue_env_fixture_target
{
    # A stand-in for target_setup.bash and the target setup scripts it chains, including the git-ps1
    # target that clobbers the prompt the virtualenv saved. The PATH entry it adds is deliberately
    # unique: a developer's own PATH already holds /usr/lib/ccache from the real ccache target, which
    # would mask the assertion that the entry is gone after the revert.
    cat > "${TUE_TEST_ENV_DIR}/.env/setup/target_setup.bash" << 'TARGET'
export TUE_TEST_TARGET_VAR=1
export PATH="/opt/tue-test/bin${PATH:+:${PATH}}"
alias tue_test_target_alias='echo aliased'
tue_test_target_fn() {
    echo target
}
_OLD_VIRTUAL_PS1="\\u@\\h git \\\$ "
PS1="\\u@\\h git \\\$ "
TARGET
    return 0
}
