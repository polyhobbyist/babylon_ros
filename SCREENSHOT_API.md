# Screenshot API Documentation

The Babylon ROS library now includes screenshot functionality that allows you to capture the 3D scene without UI elements and export it as a base64 encoded PNG image.

## API Methods

### `takeScreenshot(width?: number, height?: number): Promise<string>`

Takes a screenshot of the scene without UI elements and returns it as a base64 encoded PNG string.

**Parameters:**
- `width` (optional): Width for the screenshot. If not provided, uses current canvas width
- `height` (optional): Height for the screenshot. If not provided, uses current canvas height

**Returns:**
- `Promise<string>`: Base64 encoded PNG data string (without the `data:image/png;base64,` prefix)

**Example:**
```typescript
const robotScene = new RobotScene();
// ... initialize scene and load URDF ...

// Take screenshot with default canvas dimensions
const base64Data = await robotScene.takeScreenshot();

// Take screenshot with custom dimensions
const base64DataCustom = await robotScene.takeScreenshot(1920, 1080);

// Save as file (browser environment)
const link = document.createElement('a');
link.href = `data:image/png;base64,${base64Data}`;
link.download = 'robot_scene.png';
link.click();
```

### `takeScreenshotDataURL(width?: number, height?: number): Promise<string>`

Takes a screenshot of the scene without UI elements and returns it as a data URL.

**Parameters:**
- `width` (optional): Width for the screenshot. If not provided, uses current canvas width
- `height` (optional): Height for the screenshot. If not provided, uses current canvas height

**Returns:**
- `Promise<string>`: Data URL string (`data:image/png;base64,...`)

**Example:**
```typescript
const robotScene = new RobotScene();
// ... initialize scene and load URDF ...

// Take screenshot as data URL
const dataUrl = await robotScene.takeScreenshotDataURL();

// Display in an img element
const img = document.createElement('img');
img.src = dataUrl;
document.body.appendChild(img);
```

## Features

### UI Element Hiding
The screenshot functionality automatically hides the following UI elements during capture:
- GUI controls (buttons, panels, selection menus)
- Utility layers (gizmos, axis indicators)
- Any meshes with names containing "gizmo", "GUI", or "ui"

### Scene Content
The screenshot captures:
- Robot meshes (links, visuals, collision geometries)
- Ground plane and environment
- Lighting and materials
- Camera perspective

### Error Handling
The methods include comprehensive error handling for:
- Uninitialized scene or engine
- Canvas access issues
- Render target creation problems
- Pixel data reading failures

## Demo Implementation

A screenshot button has been added to the test interface in `RenderTest.ts`. When clicked, it:
1. Takes a screenshot of the current scene
2. Automatically downloads it as a PNG file
3. Logs the operation status to the console

## Technical Details

The implementation uses:
- `BABYLON.RenderTargetTexture` for off-screen rendering
- Canvas 2D context for PNG conversion
- Proper Y-axis flipping for correct image orientation
- Support for both Uint8Array and Float32Array pixel formats
- Automatic UI state restoration after capture

## Browser Compatibility

This feature works in modern browsers that support:
- Canvas 2D API
- HTML5 download attribute
- Babylon.js WebGL rendering
- Async/await syntax

## Error Recovery

The screenshot methods use try/finally blocks to ensure that UI elements are always restored to their original state, even if an error occurs during the screenshot process.