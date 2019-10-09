# commit_range.py

import json
import os
import sys
import urllib

TEAM_FOUNDATION_URI = os.getenv("SYSTEM_TEAMFOUNDATIONCOLLECTIONURI")
TEAM_PROJECT = os.getenv("SYSTEM_TEAMPROJECT")
BUILD_ID = os.getenv("BUILD_BUILDID")

TEAM_FOUNDATION_URI = TEAM_FOUNDATION_URI.rstrip("/")

json_url = "{}/{}/_apis/build/builds/{}/changes?&$top=500&includeSourceChange=true&api-version=5.0".format(
    TEAM_FOUNDATION_URI, TEAM_PROJECT, BUILD_ID
)

json_response = urllib.urlopen(json_url)
json_data = json.loads(json_response.read())
number_commits = json_data["count"]
newest_commit = json_data["value"][0]["id"]
oldest_commit = json_data["value"][number_commits-1]["id"]
commit_range = "{}...{}".format(newest_commit, oldest_commit)
print(commit_range)







