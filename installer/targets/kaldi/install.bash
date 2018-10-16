#! /usr/bin/env bash
#
# Bash script to install target kaldi

# Clone kaldi fork from tue-robotics (only latest commit)
KALDI_CLONE="--depth=1 https://github.com/tue-robotics/kaldi.git"
tue-install-git "$KALDI_CLONE" ~/src/kaldi_speech

# Run installer
cd ~/src/kaldi_speech
./install.bash
