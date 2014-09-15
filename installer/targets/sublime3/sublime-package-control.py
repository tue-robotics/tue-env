#!/usr/bin/python3.2

import urllib.request, os, hashlib;

h = '7183a2d3e96f11eeadd761d777e62404e330c659d4bb41d3bdf022e94cab3cd0';
pf = 'Package Control.sublime-package';
ipp = os.path.expanduser('~/.config/sublime-text-3/Installed Packages')

if os.path.exists(ipp):
	urllib.request.install_opener( urllib.request.build_opener( urllib.request.ProxyHandler()) );
	by = urllib.request.urlopen( 'http://sublime.wbond.net/' + pf.replace(' ', '%20')).read();
	dh = hashlib.sha256(by).hexdigest();
	if dh != h:
		print('Error validating download (got %s instead of %s), please try manual install' % (dh, h))
	else:
		open(os.path.join( ipp, pf), 'wb' ).write(by)
		print('Done!')
else:
	print('Error, sublime path not found')