
# Installation
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
