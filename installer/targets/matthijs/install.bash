#!/bin/bash

if [ ! -f ~/.config/geany/geany.conf ]
then
    tue-install-debug "geany config not existing"
    mkdir -p ~/.config/geany
    cp --verbose $TUE_DIR/installer/targets/matthijs/geany.conf  ~/.config/geany/geany.conf
else
    tue-install-debug "geany config does exists, so not copying matthhijs config"
fi
