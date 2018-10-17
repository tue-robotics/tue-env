#! /usr/bin/env bash
#
# Bash script to install target kaldi

# Clone kaldi fork from tue-robotics (only latest commit)
KALDI_REPO="https://github.com/tue-robotics/kaldi.git"
KALDI_HOME=~/src/kaldi_speech

# By default, set the previous commit to -1, which will trigger a 'make'
prev="-1"

# If the directory already exists
if [ -d "$KALDI_HOME" ]
then
    cd "$KALDI_HOME"
    REMOTE=$(git config --get remote.origin.url) # get the remote

    # If the kaldi_speech is pointing to the wrong Remote, correct it
    if [ "$REMOTE" != "$KALDI_REPO" ]
    then
        tue-install-debug "Changing kaldi_speech to point to new remote"
        git remote set-url origin "$KALDI_REPO"
    fi

    # Git is set-up correctly, so record the previous commit
    prev=$(git rev-list HEAD -n 1)
fi

# tue-install-git will decide if clone or pull is needed
tue-install-git "$KALDI_REPO" "$KALDI_HOME"

# Build toolkit if needed
cd "$KALDI_HOME"
if [ "$prev" != "$(git rev-list HEAD -n 1)" ]; then
    tue-install-debug "Building kaldi_speech"
    ./install.bash --build
else
    tue-install-debug "kaldi_speech not updated, so not rebuilding"
fi
