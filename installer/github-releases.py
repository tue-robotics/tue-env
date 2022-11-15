#! /usr/bin/env python3

import sys
import urllib.request
import json
import re
import argparse
import os
import hashlib
import time


def get_release(url, filename, output) -> int:
    """Function to get a release

    :param url: URL of the release tag
    :param filename: Name of the file to be downloaded. Accepts a regular expression string.
    :param output: Path of the output location
    """
    parsed_json = json.loads(urllib.request.urlopen(url).read())
    asset_re = re.compile(rf"{filename}")

    assets = [asset for asset in parsed_json["assets"] if asset_re.match(asset["name"])]

    browser_download_urls = [asset["browser_download_url"] for asset in assets]

    if not browser_download_urls:
        return 1

    for web_url in browser_download_urls:
        download_url(web_url, output)

    return 0


def download_url(url, root, filename=None, md5=None) -> None:
    """Download a file from a url and place it in root.

    Args:
        url (str): URL to download file from
        root (str): Directory to place downloaded file in
        filename (str, optional): Name to save the file under. If None, use the basename of the URL
        md5 (str, optional): MD5 checksum of the download. If None, do not check
    """
    root = os.path.expanduser(root)
    if not filename:
        filename = os.path.basename(url)
    fpath = os.path.join(root, filename)

    os.makedirs(root, exist_ok=True)

    # check if file is already present locally
    if check_integrity(fpath, md5):
        print("Using downloaded and verified file: " + fpath)
    else:  # download the file
        try:
            print("Downloading " + url + " to " + fpath)
            urllib.request.urlretrieve(url, fpath, reporthook=reporthook)
            print()
        except (urllib.error.URLError, IOError) as error:
            if url[:5] == "https":
                url = url.replace("https:", "http:")
                print("Failed download. Trying https -> http instead. Downloading " + url + " to " + fpath)
                urllib.request.urlretrieve(url, fpath, reporthook=reporthook)
                print()
            else:
                raise error
        # check integrity of downloaded file
        if not check_integrity(fpath, md5):
            raise RuntimeError("File not found or corrupted.")


def reporthook(count, block_size, total_size) -> None:
    """Function to create a progress bar"""
    global START_TIME
    if count == 0:
        START_TIME = time.time()
        return
    duration = time.time() - START_TIME
    progress_size = int(count * block_size)
    speed = int(progress_size / (1024 * duration))
    percent = int(count * block_size * 100 / total_size)
    sys.stdout.write(
        "\r...%d%%, %d MB, %d KB/s, %d seconds passed" % (percent, progress_size / (1024 * 1024), speed, duration)
    )
    sys.stdout.flush()


def calculate_md5(fpath, chunk_size=1024 * 1024) -> str:
    """Function to calculate md5 checksum"""
    md5 = hashlib.md5()
    with open(fpath, "rb") as f:
        for chunk in iter(lambda: f.read(chunk_size), b""):
            md5.update(chunk)
    return md5.hexdigest()


def check_md5(fpath, md5, **kwargs) -> bool:
    """Function to check md5 checksum of a file"""
    return md5 == calculate_md5(fpath, **kwargs)


def check_integrity(fpath, md5=None) -> bool:
    """Function to check if the given filepath has a file with the right checksum"""
    if not os.path.isfile(fpath):
        return False
    if md5 is None:
        return True
    return check_md5(fpath, md5)


def create_release(url, tag, filename, data_dir):
    """Function to upload a new release"""
    raise NotImplementedError("This functionality is not available yet.")


def main() -> int:
    """Function to parse arguments and select between creating or getting a release"""
    parser = argparse.ArgumentParser()
    release_group = parser.add_mutually_exclusive_group()
    tags_group = parser.add_mutually_exclusive_group()

    release_group.add_argument("--create", help="Create new release", action="store_true")

    release_group.add_argument("--get", help="Get the latest release if no version is specified", action="store_true")

    tags_group.add_argument("-l", "--latest", help="Get the latest release", action="store_true")

    tags_group.add_argument("-t", "--tag", help="Release tag (default=latest for --get)", type=str)

    parser.add_argument(
        "-u",
        "--url",
        help="Short url of the github repository without .git extension, eg: tue-robotics/tue-env",
        required=True,
    )

    parser.add_argument(
        "-o", "--output", help="Absolute path of the data directory (used for both creating" "and getting releases)"
    )

    parser.add_argument("filename", help="Short name of the tar file to upload or download")

    args = parser.parse_args()

    if not (args.get or args.create):
        print("Either --get or --create needs to be set")
        return 1

    url = f"https://api.github.com/repos/{args.url}/releases"

    # Get release
    if args.get:
        if args.latest:
            url += "/latest"
        elif args.tag:
            url += f"/tags/{args.tag}"
        else:
            print("With --get option either specify --latest or a specific tag using --tag")
            return 1

        return get_release(url, args.filename, args.output)

    # Create release
    if args.tag:
        url += f"/tags/{args.tag}"
    else:
        print("With --create option, --tag is a required argument")
        return 1

    return create_release(url, args.tag, args.filename, args.output)


if __name__ == "__main__":
    sys.exit(main())
