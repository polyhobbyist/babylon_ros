# URDF Viewer 

The generic URDF viewer accepts any URDF or Xacro file via URL parameters:

```
https://ranchhandrobotics.com/babylon_ros/urdf-viewer.html?urdf=YOUR_URDF_URL
```

**Example URLs:**

- **Mule Robot**: `https://ranch-hand-robotics.github.io/babylon_ros/urdf-viewer.html?urdf=https://raw.githubusercontent.com/Ranch-Hand-Robotics/babylon_ros/main/test/testdata/mule.urdf`

- **R2 Robot**: `https://ranch-hand-robotics.github.io/babylon_ros/urdf-viewer.html?urdf=https://raw.githubusercontent.com/Ranch-Hand-Robotics/babylon_ros/main/test/testdata/r2.urdf`

This makes it easy to embed live robot visualizations in any README by simply linking to the viewer with your URDF file URL.