# OSVR-ARToolKit
[ARToolKit](http://artoolkit.org/) Analysis Plugin for OSVR.

This plugin receives imaging reports from video plugins, such as OpenCV, and performs marker detection. This initial implemention tracks one pattern, patt.hiro (located in the Data folder), and sends a tracker pose showing the position and orientation of the marker on the path /me/hands/left.

![Hiro pattern](https://github.com/OSVR/OSVR-ARToolKit/blob/master/images/hiro.jpg?raw=true)

# Instructions
osvr_server_config.artoolkit.json is an example server config which takes the OpenCV plugin as input.

Place the Data folder next to osvr_server.exe, and place org_osvr_artoolkit.dll in the osvr-plugins-0 directory.

Run the server and point a camera at the Hiro target image. When the marker is detected, the server will report marker pose, and you can also see the pose in Tracker View.
