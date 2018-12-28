#! /usr/bin/env bash
#
# Bash script to install target kaldi

# Clone kaldi fork from tue-robotics (only latest commit)
ASR_REPO="https://github.com/tue-robotics/speech_recognition.git"
ASR_HOME=~/src/speech_recognition

# By default, set the previous commit to -1, which will trigger a 'make'
prev="-1"

# If the directory already exists
if [ -d "$ASR_HOME" ]
then
    cd "$ASR_HOME"
    REMOTE=$(git config --get remote.origin.url) # get the remote

    # If the kaldi_speech is pointing to the wrong Remote, correct it
    if [ "$REMOTE" != "$ASR_REPO" ]
    then
        tue-install-debug "Updated kaldi_speech remote from $REMOTE to $ASR_REPO"
        git remote set-url origin "$ASR_REPO"
    fi

    # Git is set-up correctly, so record the previous commit
    prev=$(git rev-list HEAD -n 1)
fi

# tue-install-git will decide if clone or pull is needed
tue-install-git "$ASR_REPO" "$ASR_HOME"

# Build toolkit if needed
cd "$ASR_HOME"
if [ "$prev" != "$(git rev-list HEAD -n 1)" ]; then
    tue-install-debug "Building kaldi_speech"
    ./install.bash --build
else
    tue-install-debug "kaldi_speech not updated, so not rebuilding"
fi
