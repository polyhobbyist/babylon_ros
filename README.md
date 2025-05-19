# babylon_ros

## Overview

Babylon_ROS is a Node.JS API for rendering ROS 2 based URDFs and Xacro in a web browser using Babylon.js 3D.

## Features

- 🤖 **URDF and Xacro Object Model**: Loads and validates URDF and Xacro files into an object model you can access
- 🌐 **Web Rendering Interface**: Access your robot's visualization from any device with a web browser

## Non-Features
- **No Real-time Visualization**: This package does not provide real-time visualization capabilities. It is focused on static visualization and interaction.
- **No Simulation**: Babylon ROS does not include a physics engine for simulating dynamics or collisions.
- **No Sensor visualization**: The package does not simulate or visualize sensors.

## Installation
Babylon ROS is an npm package that can be installed in your web application. To install, run:

```bash
npm install --save @polyhobbyist/babylon_ros
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
To use your own hosting application, I suggest looking at how the Test Page is implemented in the `test` directory.

## Support
Support is available through the [GitHub Discussion at Ranch Hand Robotics](https://github.com/orgs/Ranch-Hand-Robotics/discussions).

