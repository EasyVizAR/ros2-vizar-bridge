import uuid

import requests


def is_uuid(x):
    try:
        uuid.UUID(x)
        return True
    except:
        return False


class ServerResource:
    def __init__(self, collection_url: str, identifier, parent=None, prototype: dict = None):
        """
        ServerResource tracks a named object on a REST API server.

        collection_url: base URL for the collection, e.g. "https://example.org/books"
        identifier: object ID (string or numeric) or object name, in which case, names are assumed unique
        parent: parent resource, used for subresources
        prototype: optional dictionary containing fields to be set if the object needs to be created
        """
        self.collection_url = collection_url
        self.identifier = identifier
        self.parent = parent

        if prototype is None:
            self.prototype = dict()
        else:
            self.prototype = prototype

        # Will be set to True after we have confirmed a valid resource ID on the server.
        self.verified = False

    @property
    def id(self):
        if self.verified:
            return self.identifier

        else:
            self.resolve()
            return self.identifier

    @property
    def url(self):
        if not self.verified:
            self.resolve()

        return f"{self.collection_url}/{self.identifier}"

    def patch(self, data):
        res = requests.patch(self.url, json=data)
        return res.json()

    def resolve(self):
        if self.verified:
            url = f"{self.collection_url}/{self.identifier}"
            obj = requests.get(url)
            return obj

        if self.parent is not None:
            collection_url = f"{self.parent.url}/{self.collection_url}"
        else:
            collection_url = self.collection_url

        res = requests.get(collection_url)
        for obj in res.json():
            if obj['id'] == self.identifier or obj.get("name") == self.identifier:
                self.collection_url = collection_url
                self.identifier = obj['id']
                self.verified = True
                return obj

        if is_uuid(self.identifier):
            url = f"{collection_url}/{self.identifier}"
            res = requests.put(url, json=self.prototype)
            self.collection_url = collection_url
            self.verified = True
            return res.json()

        else:
            new_obj = self.prototype
            new_obj['name'] = self.identifier
            res = requests.post(collection_url, json=new_obj)
            obj = res.json()
            self.collection_url = collection_url
            self.identifier = obj['id']
            self.verified = True
            return res.json()

    def subresource(self, subpath, identifier, prototype=None):
        return ServerResource(subpath, identifier, parent=self, prototype=prototype)
