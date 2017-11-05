#!/bin/bash

if [ ! -f ~/.config/geany/geany.conf ]
then
    tue-install-debug "geany config not existing"
    mkdir -p ~/.config/geany
    cp --verbose $TUE_DIR/installer/targets/matthijs/geany.conf  ~/.config/geany/geany.conf
else
    tue-install-debug "geany config does exists, so not copying matthijs config"
fi

git_config_items="pull.ff=true color.ui=always"
for item in $git_config_items
do
        option=${item%=*}
        value=${item#*=}
        tue-install-debug "git config --global --replace-all $option $value"
        git config --global --replace-all $option $value
done
