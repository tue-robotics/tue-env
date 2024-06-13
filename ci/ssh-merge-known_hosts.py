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
from collections import defaultdict
from typing import Dict, List, Optional, Set, Tuple, Union


def key_dict_factory() -> Dict[str, Union[Set[str], Optional[str]]]:
    return {"comments": set(), "leading_comment_lines": set(), "marker": None}


def truncate(s: str, w: int) -> str:
    s = s.strip()
    if len(s) > w:
        s = s[: w - 3].strip() + "..."
    return s


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

hostkeys: Dict[Tuple[str, str, str], Dict[str, Union[Set[str], str]]] = defaultdict(key_dict_factory)
for kfile in args.files:
    with open(kfile) as kf:
        leading_comment_lines = set()
        for line in kf:
            if line[0] == "#":
                leading_comment_lines.add(line)
                continue
            line_splitted: List[str] = line.rstrip().split(" ")
            marker: Optional[str] = None
            if line_splitted[0].startswith("@"):
                marker = line_splitted.pop(0)
            hosts: List[str] = line_splitted.pop(0).split(",")
            key_type: str = line_splitted.pop(0)
            key = line_splitted.pop(0)
            comment: Optional[str] = None
            if line_splitted:
                if not line_splitted[0].startswith("#"):
                    raise ValueError(f"Unknown remainder in line: {line}")
                comment = " ".join(line_splitted)
            for host in hosts:
                unique_key = (host, key_type, key)
                entry = hostkeys[unique_key]
                if comment is not None:
                    entry["comments"].add(comment)
                if leading_comment_lines:
                    entry["leading_comment_lines"].update(leading_comment_lines)
                if marker is not None:
                    if hostkeys[unique_key]["marker"] is not None:
                        raise ValueError(
                            f"Multiple markers for same key: ({truncate(unique_key[0], 25)}, {unique_key[1]}, {truncate(unique_key[2], 25)})"
                        )
                    entry["marker"] = marker

            leading_comment_lines = set()

# And now output it all
for (host, key_type, key), v in hostkeys.items():
    if v["leading_comment_lines"]:
        for line in v["leading_comment_lines"]:
            output.write(line)
    line_items = []
    if v["marker"] is not None:
        line_items.append(v["marker"])
    line_items.append(host)
    line_items.append(key_type)
    line_items.append(key)
    if v["comments"]:
        line_items.append(" ".join(v["comments"]))
    output.write(f"{' '.join(line_items)}\n")

# Write to output file
if args.output:
    with open(args.output, "w") as f:
        f.write(output.getvalue())
