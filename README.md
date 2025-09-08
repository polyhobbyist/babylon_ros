# babylon_ros

## Overview

Babylon_ROS is a Node.JS API for rendering ROS 2 based URDFs and Xacro in a web browser using Babylon.js 3D.

## Live Demo

Here's the Mule robot (6-wheeled rover) rendered live using babylon_ros:

<!-- GitHub live demo - works on GitHub.com -->
<div align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="https://via.placeholder.com/800x400/1a1a1a/ffffff?text=🐴+Mule+Robot+3D+Viewer%0A%0A🚀+Click+to+view+live+on+GitHub">
    <img alt="Mule Robot 3D Viewer" src="https://via.placeholder.com/800x400/f8f9fa/000000?text=🐴+Mule+Robot+3D+Viewer%0A%0A🚀+Click+to+view+live+on+GitHub" width="800" height="400">
  </picture>
</div>

<!-- Interactive viewer (works on GitHub Pages, blocked in VS Code preview due to CSP) -->
<div style="position: relative; width: 100%; height: 500px; border: 2px solid #ddd; border-radius: 8px; overflow: hidden; margin: 20px 0; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);">
    <canvas id="babylonCanvas" style="width: 100%; height: 100%; display: block;"></canvas>
    <div id="robotControls" style="position: absolute; top: 10px; right: 10px; background: rgba(0,0,0,0.8); color: white; padding: 8px 12px; border-radius: 6px; font-family: Arial, sans-serif; font-size: 12px; z-index: 100;">
        <div style="margin-bottom: 8px; font-weight: bold;">🐴 Mule Robot</div>
        <button onclick="resetRobotCamera()" style="margin: 2px; padding: 4px 8px; background: #4CAF50; color: white; border: none; border-radius: 3px; cursor: pointer; font-size: 11px;">🎯 Reset View</button>
        <button onclick="takeRobotScreenshot()" style="margin: 2px; padding: 4px 8px; background: #4CAF50; color: white; border: none; border-radius: 3px; cursor: pointer; font-size: 11px;">📸 Screenshot</button>
    </div>
    <div id="robotLoading" style="position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%); background: rgba(0,0,0,0.9); color: white; padding: 20px; border-radius: 8px; text-align: center; font-family: Arial, sans-serif;">
        <div style="font-size: 18px; margin-bottom: 10px;">🤖 Loading Mule Robot...</div>
        <div style="font-size: 12px; opacity: 0.8;">Powered by babylon_ros</div>
        <div style="font-size: 10px; margin-top: 8px; opacity: 0.6;">Note: Interactive viewer blocked in VS Code preview due to CSP.<br>View on GitHub.com for full functionality.</div>
    </div>
</div>

<script async src="https://unpkg.com/babylonjs@7.16.1/babylon.js"></script>
<script async src="https://unpkg.com/babylonjs-gui@7.16.1/babylon.gui.min.js"></script>
<script async src="https://unpkg.com/@ranch-hand-robotics/babylon_ros/web/ros.js"></script>

<script>
(function() {
    let globalRobotScene;
    let scriptsLoaded = 0;
    const totalScripts = 3;

    function checkAllScriptsLoaded() {
        scriptsLoaded++;
        if (scriptsLoaded >= totalScripts && typeof babylon_ros !== 'undefined') {
            loadEmbeddedRobot();
        }
    }

    async function loadEmbeddedRobot() {
        try {
            const canvas = document.getElementById('babylonCanvas');
            if (!canvas) return;
            
            const urdfUrl = 'https://raw.githubusercontent.com/Ranch-Hand-Robotics/babylon_ros/main/test/testdata/mule.urdf';
            const response = await fetch(urdfUrl);
            const urdfText = await response.text();

            globalRobotScene = new babylon_ros.RobotScene();
            await globalRobotScene.createScene(canvas);
            await globalRobotScene.applyURDF(urdfText);
            
            const loading = document.getElementById('robotLoading');
            if (loading) loading.style.display = 'none';
            
            console.log('🎉 Embedded Mule robot loaded successfully!');
        } catch (error) {
            console.error('❌ Error loading embedded robot:', error);
            const loading = document.getElementById('robotLoading');
            if (loading) {
                loading.innerHTML = `
                    <div style="color: #ff6b6b; font-size: 16px;">⚠️ Failed to load robot</div>
                    <div style="font-size: 12px; margin-top: 8px; opacity: 0.8;">${error.message}</div>
                    <div style="font-size: 10px; margin-top: 8px; opacity: 0.6;">Try viewing on GitHub.com or GitHub Pages</div>
                `;
            }
        }
    }

    window.resetRobotCamera = function() {
        if (globalRobotScene) {
            globalRobotScene.resetCamera();
        }
    };

    window.takeRobotScreenshot = async function() {
        if (globalRobotScene) {
            try {
                const base64Data = await globalRobotScene.takeScreenshot();
                const link = document.createElement('a');
                link.href = 'data:image/png;base64,' + base64Data;
                link.download = 'mule_robot_' + new Date().toISOString().slice(0,10) + '.png';
                link.click();
            } catch (error) {
                console.error('Screenshot failed:', error);
            }
        }
    };

    // Try to load when DOM is ready
    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', function() {
            setTimeout(checkAllScriptsLoaded, 2000);
        });
    } else {
        setTimeout(checkAllScriptsLoaded, 2000);
    }
})();
</script>

> **📝 Note:** The interactive 3D viewer above works on GitHub.com and GitHub Pages, but is blocked in VS Code's markdown preview due to Content Security Policy restrictions. For the full interactive experience, view this README on GitHub.com.

*🎯 Interactive 3D robot viewer • 🔄 Rotate with mouse • 📸 Download screenshots*

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

### Embedding Robot Viewers Directly in Markdown

You can embed live robot visualizations directly in GitHub README files using inline HTML and JavaScript. This repository demonstrates this approach with the Mule robot shown above.

**Example Inline Embedding:**

```html
<div style="position: relative; width: 100%; height: 500px; border: 2px solid #ddd; border-radius: 8px; overflow: hidden;">
    <canvas id="robotCanvas" style="width: 100%; height: 100%; display: block;"></canvas>
    <!-- Controls and loading UI here -->
</div>

<script src="https://unpkg.com/babylonjs@7.16.1/babylon.js"></script>
<script src="https://unpkg.com/@ranch-hand-robotics/babylon_ros/web/ros.js"></script>
<script>
// Robot loading and control code here
async function loadYourRobot() {
    const canvas = document.getElementById('robotCanvas');
    const robotScene = new babylon_ros.RobotScene();
    await robotScene.createScene(canvas);
    
    // Fetch your URDF
    const response = await fetch('https://raw.githubusercontent.com/Ranch-Hand-Robotics/babylon_ros/refs/heads/main/test/testdata/mule.urdf');
    const urdfText = await response.text();
    await robotScene.applyURDF(urdfText);
}
</script>
```

**Benefits of Inline Embedding:**
- ✅ No separate HTML files needed
- ✅ Everything contained in the README  
- ✅ Direct interaction without leaving GitHub
- ✅ Easy to customize and maintain

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

