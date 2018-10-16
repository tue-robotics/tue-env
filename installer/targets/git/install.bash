#!/bin/bash

# check for git-extras
if [ ! -f /usr/local/bin/git-extras ]; then
	# https://github.com/tj/git-extras
	cd /tmp && git clone --depth 1 https://github.com/visionmedia/git-extras.git && cd git-extras && sudo make install

	# enable color
	git config --global color.ui true
fi

git config --global --replace-all core.excludesFile $( dirname "${BASH_SOURCE[0]}")/gitignore
