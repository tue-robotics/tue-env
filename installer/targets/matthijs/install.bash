#!/bin/bash

file_diff=$(diff <(head -n 1 $TUE_DIR/installer/targets/matthijs/geany.conf) <(head -n 1 ~/.config/geany/geany.conf))
if [ -n "$file_diff" ]
then
    cp --verbose $TUE_DIR/installer/targets/matthijs/geany.conf  ~/.config/geany/geany.conf
fi
