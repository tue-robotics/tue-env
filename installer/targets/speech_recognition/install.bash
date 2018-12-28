#! /usr/bin/env bash
#
# Bash script to install target speech_recognition

ASR_REPO="https://github.com/tue-robotics/speech_recognition.git"
ASR_HOME=~/src/speech_recognition

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
fi

# tue-install-git will decide if clone or pull is needed
tue-install-git "$ASR_REPO" "$ASR_HOME"

