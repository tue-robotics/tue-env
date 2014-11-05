#!/bin/bash

# check for git-extras
if [ ! -f /usr/local/bin/git-extras ]; then
	# https://github.com/tj/git-extras
	cd /tmp && git clone --depth 1 https://github.com/visionmedia/git-extras.git && cd git-extras && sudo make install

	# enable color
	git config --global color.ui true
fi

tue-install-system git
tue-install-system gitk
tue-install-system git-gui
