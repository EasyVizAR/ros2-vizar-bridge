import os
import uuid

import requests

from .server_resource import ServerResource


class VizarClient:
    def __init__(self, server: str, device_id: str, location_id: str):
        """
        Helper client for communicating with VizAR server.

        server: base URL for the server, e.g. "https://example.org"
        device_id: device UUID (a.k.a. headset ID) or name, in which case, the name is assumed to be unique
        location_id: location UUId or name, in which case, the name is assumed to be unique
        """
        self.server = server
        self.device_id = device_id
        self.location_id = location_id

        self.device = ServerResource(f"{server}/headsets", device_id, prototype=dict(type="robot"))
        self.location = ServerResource(f"{server}/locations", location_id)
        self.layer = self.location.subresource("layers", "ROS", prototype=dict(type="uploaded"))

    def update_layer(self, image_data, box):
        """
        Upload map image and boundary information.
        """
        url = f"{self.layer.url}/image"
        res = requests.put(url, data=image_data)

        data = dict()
        for k, v in box.items():
            data["boundary."+k] = v
        return self.layer.patch(data)

    def update_pose(self, position, orientation):
        """
        Update device position and orientation.

        ROS coordinate system:
            x - forward
            y - left
            z - up

        VizAR coordinate system:
            x - right
            y - up
            z - forward
        """
        data = dict(
            location_id=self.location.id,
            position=dict(
                x=-position.y,
                y=position.z,
                z=position.x
            ),
            orientation=dict(
                x=-orientation.y,
                y=orientation.z,
                z=orientation.x,
                w=-orientation.w
            )
        )

        return self.device.patch(data)
