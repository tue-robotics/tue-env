#! /usr/bin/env python3

import sys
import urllib.request
import json
import re
import argparse


def get_release(url, filename, output):
    """Function to get a release

    :param url: URL of the release tag
    :param filename: Name of the file to be downloaded. Accepts a regular expression string.
    :param output: Path of the output location
    """
    parsed_json = json.loads(urllib.request.urlopen(url).read())
    asset_re = re.compile(r'{}'.format(filename))

    assets = [asset for asset in parsed_json['assets'] if asset_re.match(asset['name'])]

    download_urls = [asset['browser_download_url'] for asset in assets]
    print(download_urls)


def create_release(url, tag, filename, data_dir):
    """Function to upload a new release"""
    pass


def main():
    """Function to parse arguments and select between creating or getting a release"""
    parser = argparse.ArgumentParser()
    release_group = parser.add_mutually_exclusive_group()
    tags_group = parser.add_mutually_exclusive_group()

    release_group.add_argument("--create",
                               help="Create new release",
                               action="store_true")

    release_group.add_argument("--get",
                               help="Get the latest release if no version is specified",
                               action="store_true")

    tags_group.add_argument("-l", "--latest",
                            help="Get the latest release",
                            action="store_true")

    tags_group.add_argument("-t", "--tag",
                            help="Release tag (default=latest for --get)",
                            type=str)

    parser.add_argument("-u", "--url",
                        help="Short url of the github repository without .git extension, eg: tue-robotics/tue-env",
                        required=True)

    parser.add_argument("-o", "--output",
                        help="Absolute path of the data directory (used for both creating"
                        "and getting releases)")

    parser.add_argument("filename",
                        help="Short name of the tar file to upload or download")

    args = parser.parse_args()

    if not (args.get or args.create):
        print("Either --get or --create needs to be set")
        return 1

    url = "https://api.github.com/repos/{}/releases".format(args.url)

    if args.get:
        if args.latest:
            url += '/latest'
        elif args.tag:
            url += '/tags/{}'.format(args.tag)
        else:
            print("With --get option either specify --latest or a specific tag using --tag")
            return 1

        get_release(url, args.filename, args.output)

    else:
        if args.tag:
            url += '/tags/{}'.format(args.tag)
        else:
            print("With --create option, --tag is a required argument")
            return 1

        create_release(url, args.tag, args.filename, args.output)

    print(args)
    print(url)

    return 0


if __name__ == "__main__":
    sys.exit(main())
