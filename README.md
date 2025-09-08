# babylon_ros

## Overview

Babylon_ROS is a Node.JS API for rendering ROS 2 based URDFs and Xacro in a web browser using Babylon.js 3D.

## Features

- 🤖 **URDF and Xacro Object Model**: Loads and validates URDF and Xacro files into an object model you can access
- 🌐 **Web Rendering Interface**: Access your robot's visualization from any device with a web browser
- 📸 **Screenshot API**: Capture clean screenshots of your robot scenes without UI elements as base64 PNG images

## Non-Features
- **No Real-time Visualization**: This package does not provide real-time visualization capabilities. It is focused on static visualization and interaction.
- **No Simulation**: Babylon ROS does not include a physics engine for simulating dynamics or collisions.
- **No Sensor visualization**: The package does not simulate or visualize sensors.

## Installation
Babylon ROS is an npm package that can be installed in your web application. To install, run:

```bash
npm install --save @ranch-hand-robotics/babylon_ros
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

### Rendering URDF Files from GitHub
You can directly render URDF files hosted on GitHub using unpkg to load babylon_ros. This is perfect for embedding live robot visualizations in README files or documentation.

Here's a complete example that renders the mule robot from this repository:

```html
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Mule Robot URDF Viewer</title>
    <style>
        html, body {
            overflow: hidden;
            width: 100%;
            height: 100%;
            margin: 0;
            padding: 0;
            font-family: Arial, sans-serif;
        }
        #renderCanvas {
            width: 100%;
            height: 100%;
            touch-action: none;
        }
        #controls {
            position: absolute;
            top: 10px;
            left: 10px;
            background: rgba(0, 0, 0, 0.7);
            color: white;
            padding: 10px;
            border-radius: 5px;
            z-index: 100;
        }
        button {
            margin: 2px;
            padding: 5px 10px;
            background: #4CAF50;
            color: white;
            border: none;
            border-radius: 3px;
            cursor: pointer;
        }
        button:hover {
            background: #45a049;
        }
    </style>
</head>
<body>
    <canvas id="renderCanvas" touch-action="none"></canvas>
    <div id="controls">
        <h3>Mule Robot</h3>
        <button onclick="resetCamera()">Reset Camera</button>
        <button onclick="takeScreenshot()">Screenshot</button>
    </div>

    <!-- Load dependencies from unpkg -->
    <script src="https://unpkg.com/babylonjs@7.16.1/babylon.js"></script>
    <script src="https://unpkg.com/babylonjs-gui@7.16.1/babylon.gui.min.js"></script>
    <script src="https://unpkg.com/@ranch-hand-robotics/babylon_ros/web/ros.js"></script>

    <script>
        let robotScene;

        async function loadMuleRobot() {
            try {
                // Fetch the URDF file from GitHub
                const urdfUrl = 'https://raw.githubusercontent.com/Ranch-Hand-Robotics/babylon_ros/main/test/testdata/mule.urdf';
                const response = await fetch(urdfUrl);
                const urdfText = await response.text();

                // Create the robot scene
                const canvas = document.getElementById('renderCanvas');
                robotScene = new babylon_ros.RobotScene();
                await robotScene.createScene(canvas);
                
                // Load and render the URDF
                await robotScene.applyURDF(urdfText);
                
                console.log('Mule robot loaded successfully!');
            } catch (error) {
                console.error('Error loading robot:', error);
                alert('Failed to load robot: ' + error.message);
            }
        }

        function resetCamera() {
            if (robotScene) {
                robotScene.resetCamera();
            }
        }

        async function takeScreenshot() {
            if (robotScene) {
                try {
                    const base64Data = await robotScene.takeScreenshot();
                    const link = document.createElement('a');
                    link.href = `data:image/png;base64,${base64Data}`;
                    link.download = 'mule_robot.png';
                    link.click();
                } catch (error) {
                    console.error('Screenshot failed:', error);
                }
            }
        }

        // Load the robot when the page loads
        window.addEventListener('load', loadMuleRobot);
    </script>
</body>
</html>
```

### GitHub README Integration
To embed a live robot viewer in your GitHub README, you can use GitHub Pages or any static hosting service. Here's how to set it up:

1. **Create the viewer HTML file** (as shown above) in your repository
2. **Enable GitHub Pages** for your repository  
3. **Link to the live viewer** in your README:

```markdown
## Live Robot Visualization

[🚀 View the Mule Robot in 3D](https://ranch-hand-robotics.github.io/babylon_ros/mule_viewer.html)

![Mule Robot Preview](https://via.placeholder.com/800x400/f0f0f0/333?text=Click+above+to+view+live+3D+model)
```

## Screenshot API

The library includes built-in screenshot functionality to capture clean images of your robot scenes:

```javascript
// Take a screenshot and get base64 PNG data
const base64Data = await robotScene.takeScreenshot();

// Take a screenshot with custom dimensions
const base64Data = await robotScene.takeScreenshot(1920, 1080);

// Get a data URL for immediate use
const dataUrl = await robotScene.takeScreenshotDataURL();

// Save screenshot as file (browser)
const link = document.createElement('a');
link.href = `data:image/png;base64,${base64Data}`;
link.download = 'robot_scene.png';
link.click();
```

The screenshot API automatically hides UI elements (buttons, gizmos, etc.) during capture, ensuring clean robot visualizations. See [SCREENSHOT_API.md](./SCREENSHOT_API.md) for complete documentation.

## Support
Support is available through the [GitHub Discussion at Ranch Hand Robotics](https://github.com/Ranch-Hand-Robotics/babylon_ros/discussions).

