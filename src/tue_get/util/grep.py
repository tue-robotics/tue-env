from typing import List
import os
import re

text_characters = b"".join(map(lambda x: bytes((x,)), range(32, 127))) + b"\n\r\t\f\b"
_null_trans = bytes.maketrans(b"", b"")


def istextfile(filename: str, blocksize: int = 512) -> bool:
    return istext(open(filename, "rb").read(blocksize))


def istext(b: bytes) -> bool:
    if b"\0" in b:
        return False

    if not b:  # Empty files are considered text
        return True

    # Get the non-text characters (maps a character to itself then
    # use the 'remove' option to get rid of the text characters.)
    t = b.translate(_null_trans, text_characters)

    # If more than 30% non-text characters, then
    # this is considered a binary file
    if len(t) / len(b) > 0.30:
        return False
    return True


def grep_directory(
    directory: str, pattern: re.Pattern, recursive: bool = False, include_binary: bool = False
) -> List[str]:
    """
    Searches for a regex in a directory.

    :param directory: The directory to search in.
    :param pattern: The regex to search with.
    :param recursive: (optional) Whether to search recursively. Defaults to False.
    :param include_binary: (optional) Whether to include binary files. Defaults to False.

    :return: A list of files that match the regex.
    """
    files = []
    for root, dirs, filenames in os.walk(directory):
        for filename in filenames:
            full_path = os.path.join(root, filename)
            if not include_binary and not istextfile(full_path):
                continue
            if grep_file(full_path, pattern):
                files.append(full_path)
        if not recursive:
            break
    return files


def grep_file(file: str, pattern: re.Pattern) -> bool:
    """
    Searches for a regex in a file.

    :param file: The file to search in.
    :param pattern: The regex to search with.

    :return: Whether the regex was found in the file.
    """
    with open(file, "r") as f:
        lines = f.readlines()
    for line in lines:
        if pattern.search(line):
            return True
    return False
