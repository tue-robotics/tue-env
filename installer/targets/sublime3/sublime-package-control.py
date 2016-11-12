#!/usr/bin/env python3
import urllib.request
import os
import hashlib

h = '2915d1851351e5ee549c20394736b4428bc59f460fa1548d1514676163dafc88'
pf = 'Package Control.sublime-package'
ipp = os.path.expanduser('~/.config/sublime-text-3/Installed Packages')

print('Installing Sublime Package Control')

urllib.request.install_opener(
    urllib.request.build_opener(urllib.request.ProxyHandler()))
try:
    by = urllib.request.urlopen(
        'http://packagecontrol.io/' + pf.replace(' ', '%20')).read()
except urllib.error.URLError:
    print('Error, download failed, please try manual install')
else:
    dh = hashlib.sha256(by).hexdigest()
    print('Error validating download (got %s instead of %s), please try manual install' %
        (dh, h)) if dh != h else open(os.path.join(ipp, pf), 'wb').write(by)
