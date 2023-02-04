#! /usr/bin/env python3

"""
Inspired by https://github.com/avoinea/DockerHub-API
"""

import argparse
import json
import requests
import sys

DH_API = "https://hub.docker.com/v2"


class AuthenticationError(Exception):
    pass


class DockerHubAuth(requests.auth.AuthBase):
    def __init__(self, session, username=None, password=None, token=None):
        self._session = session
        if token is not None:
            self._token = token
        else:
            self._token = self._get_authorization_token(username, password)

    @property
    def token(self):
        return self._token

    def __eq__(self, other):
        return self._token == getattr(other, "_token", None)

    def __ne__(self, other):
        return not self == other

    def __call__(self, r):
        r.headers["Authorization"] = "JWT {}".format(self._token)
        return r

    def _get_authorization_token(self, username, password):
        """
        Actually gets the authentication token

        Raises:
            AuthenticationError: didn't login right
        """
        resp = self._session.post(
            f"{DH_API}/users/login/",
            json={"username": username, "password": password},
            headers={"Content-Type": "application/json"},
        )
        if not resp.ok:
            content = json.dumps(resp.json(), indent=4)
            raise AuthenticationError(f"Error Status {resp.status_code}:\n{content}")
        return resp.json()["token"]


class DockerHub(object):
    def __init__(self, username, password):
        self._session = requests.Session()
        self._auth = DockerHubAuth(self._session, username, password)

    def get(self, url):
        url = f"{DH_API}/{url}"
        resp = self._session.get(url, auth=self._auth)
        if not resp.ok:
            content = json.dumps(resp.json(), indent=4)
            print(f'Call to "{url}" resulted in error status {resp.status_code}:\n{content}')
            return None
        return resp.json()

    def namespaces(self):
        resp = self.get("repositories/namespaces/")
        if resp is None:
            return []
        return resp.get("namespaces", [])

    def images(self, namespace, page_size=10000):
        resp = self.get(f"repositories/{namespace}/?page_size={page_size}")
        if resp is None:
            return []
        return resp.get("results", [])

    def image_names(self, namespace, page_size=10000):
        images = self.images(namespace, page_size)
        return [image["name"] for image in images]

    def tags(self, namespace, image, page_size=10000):
        resp = self.get(f"repositories/{namespace}/{image}/tags/?page_size={page_size}")
        if resp is None:
            return []
        return resp.get("results", [])

    def tag_names(self, namespace, image, page_size=10000):
        tags = self.tags(namespace, image, page_size)
        return [tag["name"] for tag in tags]

    def delete_tag(self, namespace, image, tag):
        self._session.cookies.clear()
        resp = self._session.delete(f"{DH_API}/repositories/{namespace}/{image}/tags/{tag}/", auth=self._auth)
        if not resp.ok:
            content = json.dumps(resp.json(), indent=4)
            print(
                f'Failed to delete tag: "{tag}" of image: "{namespace}/{image}", '
                f"error status {resp.status_code}:\n{content}"
            )
        return resp.ok


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog="delete-docker-tags",
        description="Delete tags from a set of images in a single namespace",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument("-u", "--username", type=str, required=True, help="Docker hub username")
    parser.add_argument("-p", "--password", type=str, required=True, help="Docker hub password")
    parser.add_argument("-o", "--org", type=str, required=True, help="Docker hub organisation name")
    parser.add_argument("-i", "--image", type=str, nargs="+", help="Docker hub images")
    parser.add_argument("-t", "--tag", type=str, nargs="+", help="Docker hub images")

    args = parser.parse_args()

    namespace = args.org
    images = args.image
    tags = [t.lower().replace("/", "_") for t in args.tag]
    hub = DockerHub(args.username, args.password)

    error = False

    namespaces_found = hub.namespaces()
    if namespace not in namespaces_found:
        print('No access to "{namespace}" organization')
        sys.exit(1)

    images_found = hub.image_names(namespace)
    for image in images:
        if image not in images_found:
            print(f'Image: "{image}" not found. Available images: {images_found}')
            continue

        tags_found = hub.tag_names(namespace, image)
        for tag in tags:
            if tag not in tags_found:
                print(f'Tag: "{tag}" not found for image: "{namespace}/{image}". Available tags: {tags_found}')
                continue

            print(f'Going to delete tag: "{tag}" of image: "{namespace}/{image}"')
            ok = hub.delete_tag(namespace, image, tag)
            if ok:
                print(f'Successfully deleted tag: "{tag}" image: "{namespace}/{image}"')
            else:
                error = True

    sys.exit(error)
