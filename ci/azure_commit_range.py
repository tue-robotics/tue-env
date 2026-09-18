#! /usr/bin/env python3

"""
Script to determine the commit range of a build in azure pipelines.

The output of this script should be captured in bash. The output
will match the format of TRAVIS_COMMIT_RANGE, which is
newest_commit...oldest_commit

If the range is just 1 commit, an empty string is printed.
"""

import json
import os
import sys
from urllib.request import urlopen

TEAM_FOUNDATION_URI = os.environ["SYSTEM_TEAMFOUNDATIONCOLLECTIONURI"].rstrip("/")
TEAM_PROJECT = os.environ["SYSTEM_TEAMPROJECT"]
BUILD_ID = os.environ["BUILD_BUILDID"]

json_url = "{}/{}/_apis/build/builds/{}/changes?&$top=500&includeSourceChange=true&api-version=5.0".format(
    TEAM_FOUNDATION_URI, TEAM_PROJECT, BUILD_ID
)

json_response = urlopen(json_url)
json_data = json.loads(json_response.read())
number_commits = json_data["count"]
if number_commits == 1:
    print("")
    exit(0)
newest_commit = json_data["value"][0]["id"]
oldest_commit = json_data["value"][number_commits - 1]["id"]
commit_range = "{}^...{}".format(oldest_commit, newest_commit)
print(commit_range)
