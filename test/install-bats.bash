#! /usr/bin/env bash
#
# Install the pinned bats-core into test/.bats. Idempotent; safe to run in CI and by hand.

set -e

BATS_VERSION="v1.13.0"
TEST_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
BATS_DIR="${TEST_DIR}/.bats"

if [[ -x "${BATS_DIR}/bin/bats" ]]
then
    installed="$("${BATS_DIR}"/bin/bats --version)"
    if [[ "${installed}" == "Bats ${BATS_VERSION#v}" ]]
    then
        echo "[test] ${installed} already installed in '${BATS_DIR}'"
        exit 0
    fi
    echo "[test] replacing '${installed}' by bats ${BATS_VERSION}"
    rm -rf "${BATS_DIR}"
fi

echo "[test] cloning bats-core ${BATS_VERSION} into '${BATS_DIR}'"
git clone --depth 1 --branch "${BATS_VERSION}" --quiet https://github.com/bats-core/bats-core.git "${BATS_DIR}"
"${BATS_DIR}"/bin/bats --version
