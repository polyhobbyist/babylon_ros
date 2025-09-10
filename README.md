# babylon_ros

## Overview

Babylon_ros is a Node.JS API for rendering [ROS 2](https://ros.org) based URDFs and Xacro in a web browser or Visual Studio Code compatible extension using [the Babylon.js graphics engine](https://www.babylonjs.com/).


<div align="center">
  
[![Mule Robot Demo](https://img.shields.io/badge/🤖_Interactive_Demo-View_3D_Robot-blue?style=for-the-badge&logo=github)](https://ranch-hand-robotics.github.io/babylon_ros/docs/urdf-viewer.html?urdf=https://raw.githubusercontent.com/Ranch-Hand-Robotics/babylon_ros/main/test/testdata/mule.urdf)

</div>

### Universal URDF Viewer

The generic URDF viewer accepts any URDF or Xacro file via URL parameters:

```
https://ranch-hand-robotics.github.io/babylon_ros/docs/urdf-viewer.html?urdf=YOUR_URDF_URL
```

**Example URLs:**
- **Mule Robot**: `https://ranch-hand-robotics.github.io/babylon_ros/docs/urdf-viewer.html?urdf=https://raw.githubusercontent.com/Ranch-Hand-Robotics/babylon_ros/main/test/testdata/mule.urdf`
- **R2 Robot**: `https://ranch-hand-robotics.github.io/babylon_ros/docs/urdf-viewer.html?urdf=https://raw.githubusercontent.com/Ranch-Hand-Robotics/babylon_ros/main/test/testdata/r2.urdf`

This makes it easy to embed live robot visualizations in any README by simply linking to the viewer with your URDF file URL.


## Features

- 🤖 **URDF and Xacro Object Model**: Loads and validates URDF and Xacro files into an object model you can access
- 🌐 **Web Rendering Interface**: Access your robot's visualization from any device with a web browser
- 📸 **Screenshot API**: Capture clean screenshots of your robot scenes without UI elements as base64 PNG images

## Non-Features
- **No Real-time Visualization**: This package does not provide real-time visualization capabilities. It is focused on static visualization and interaction.
- **No Simulation**: Babylon ROS does not include a physics engine for simulating dynamics or collisions.
- **No Sensor visualization**: The package does not simulate or visualize sensors.

## Installation
Babylon_ros is available via the [Node Package Manager](https://npmjs.com) package that can be installed in your web application. To install, run:

```bash
npm install --save @ranchhandrobotics/babylon_ros
```

## Usage
To use Babylon ROS in your web application, you need to set up a basic HTML page and include the Babylon.js library along with the Babylon ROS package. 

Here’s a simple example which renders a the Test Page included in this package:

```html
<!DOCTYPE html>
<html lang="en">
<head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <style nonce="${nonce}">
    html,
    body {
    overflow: hidden;
    width: 100%;
    height: 100%;
    margin: 0;
    padding: 0;
    }

    #renderCanvas {
    width: 100%;
    height: 100%;
    touch-action: none;
    }
</style>
<title>URDF Preview</title>
</head>
<body>
    <canvas id="renderCanvas" touch-action="none"></canvas>    
    <script src="../node_modules/babylonjs/babylon.js"></script>
    <script src="./ros.js"></script>
    <script>
        
        window.addEventListener("load", babylon_ros.RenderTestMain);

    </script>

</body>
</html>
```

## Support
If you encounter any issues with this package, the following resources are provided:

### Github Issues
Bugs and feature requests are handled through [Github Issues in the Repository](https://github.com/Ranch-Hand-Robotics/babylon_ros/issues). 

If you find that you are hitting the same issue as someone else, please give a :+1: or comment on an existing issue.

Please provide as much details as possible, including an isolated reproduction of the issue or a pointer to an online repository.

### Discussions
[Github Discussions](https://github.com/orgs/Ranch-Hand-Robotics/discussions) are provided for community driven general guidance, walkthroughs, or support.

## Sponsor
If you find this package useful, please consider [sponsoring Ranch Hand Robotics](https://github.com/sponsors/Ranch-Hand-Robotics) to help support the development of this package and other open source projects.

