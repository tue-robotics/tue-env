#! /usr/bin/env python3

# Copyright (C) 2019 Joerg Jaspert <joerg@debian.org>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# .
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# .
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
# OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
# NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Copied from https://blog.ganneff.de/2019/04/ssh-known-hosts-merge-by-key.html

import argparse

parser = argparse.ArgumentParser(
    description="Merge ssh known host entries by key",
    epilog="""
Merges entries in given ssh known_hosts file based on the key. One can also merge from multiple files.
The file should NOT use the HashKnownHosts feature.
""",
)

parser.add_argument("files", type=str, nargs="+", help="files that should be merged")
parser.add_argument(
    "-o",
    "--output",
    type=str,
    nargs="?",
    help="output file (defaults is STDOUT)."
    "Only opened after merge is complete, "
    "so can be used for inplace merge.",
)
args = parser.parse_args()

if args.output:
    from io import StringIO

    output = StringIO()
else:
    from sys import stdout

    output = stdout

hostkeys = {}
for kfile in args.files:
    with open(kfile) as kf:
        for line in kf:
            if line[0] == "#":
                continue
            line_splitted = line.rstrip().split(" ")
            hosts = line_splitted.pop(0).split(",")
            key_type = line_splitted.pop(0)
            key = line_splitted[0]
            if key not in hostkeys:
                hostkeys[key] = {}
                hostkeys[key]["hosts"] = []
            hostkeys[key]["key_type"] = key_type
            # Store the host entries, uniquify them
            hostkeys[key]["hosts"].extend(hosts)

# And now output it all
for k, v in hostkeys.items():
    output.write("%s %s %s\n" % (",".join(v["hosts"]), v["key_type"], k))

# Write to output file
if args.output:
    with open(args.output, "w") as f:
        f.write(output.getvalue())
