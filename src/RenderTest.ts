import * as BABYLON from 'babylonjs';
import * as Materials from 'babylonjs-materials';
import * as urdf from './urdf';
import {Robot} from './Robot';
import {Joint} from './Joint';
import {Link} from './Link';
import {Visual} from './Visual';
import { RobotScene } from './RobotScene';

import * as GUI from 'babylonjs-gui';
import * as ColladaFileLoader from '@polyhobbyist/babylon-collada-loader';

let currentRobotScene : RobotScene | undefined = undefined;

function addTestToRobotScene(robotScene : RobotScene) {
  if (robotScene.UILayer === undefined) {
    return
  }

  var testSelection = new GUI.SelectionPanel("tests");
  testSelection.color = "white";
  testSelection.width = 0.25;
  testSelection.height = .75;
  testSelection.horizontalAlignment = GUI.Control.HORIZONTAL_ALIGNMENT_LEFT;
  testSelection.verticalAlignment = GUI.Control.VERTICAL_ALIGNMENT_BOTTOM;

  robotScene.UILayer.addControl(testSelection);

  var basicGroup = new GUI.RadioGroup("Basic Tests");
  testSelection.addGroup(basicGroup);

  basicGroup.groupPanel.color = "white";

  var basicTestList = [ 
    {name: "Basic", url: "https://raw.githubusercontent.com/polyhobbyist/babylon_ros/main/test/testdata/basic.urdf"},
    {name: "Basic Joint", url: "https://raw.githubusercontent.com/polyhobbyist/babylon_ros/main/test/testdata/basic_with_joint.urdf"},
    {name: "Basic Revolute Joint", url: "https://raw.githubusercontent.com/polyhobbyist/babylon_ros/main/test/testdata/basic_with_joint_with_effort.urdf"},
    {name: "Basic Material", url: "https://raw.githubusercontent.com/polyhobbyist/babylon_ros/main/test/testdata/basic_with_material.urdf"},
    {name: "Basic Remote Mesh", url: "https://raw.githubusercontent.com/polyhobbyist/babylon_ros/main/test/testdata/basic_with_remote_mesh.urdf"},
    {name: "Basic with STL", url: "https://raw.githubusercontent.com/polyhobbyist/babylon_ros/main/test/testdata/basic_with_stl_mesh.urdf"},
    {name: "Orientation", url: "https://raw.githubusercontent.com/polyhobbyist/babylon_ros/main/test/testdata/orientation.urdf"},
    {name: "Bad", url: "https://raw.githubusercontent.com/polyhobbyist/babylon_ros/main/test/testdata/bad.urdf"},
    {name: "DAE", url: "https://raw.githubusercontent.com/polyhobbyist/babylon_ros/main/test/testdata/leo_chassis.urdf"},
  ];

  var robotTestList = [ 
    {name: "leo", url: "https://raw.githubusercontent.com/polyhobbyist/babylon_ros/main/test/testdata/leo.urdf"},
    {name: "BB", url: "https://raw.githubusercontent.com/polyhobbyist/babylon_ros/main/test/testdata/bb.urdf"},
    {name: "Motoman", url: "https://raw.githubusercontent.com/polyhobbyist/babylon_ros/main/test/testdata/motoman.urdf"},
    {name: "Arti Robot", url: "https://raw.githubusercontent.com/polyhobbyist/babylon_ros/main/test/testdata/arti.urdf"},
    {name: "inline", url: ""},
  ];

  basicTestList.forEach((t) => {
    basicGroup.addRadio(t.name, async () => {
      const response = await fetch(t.url);
      const urdfText = await response.text();
      
      robotScene.applyURDF(urdfText);
    });
  });
  
  var robotTestGroup = new GUI.RadioGroup("Robot Tests");
  testSelection.addGroup(robotTestGroup);

  robotTestGroup.groupPanel.color = "white";

  robotTestList.forEach((t) => {
    robotTestGroup.addRadio(t.name, async () => {
      if (t.name === "inline") {
        const inlineURDF = `<?xml version="1.0"?>
<robot name="roarm_description">
  <link name="base_link">
    <inertial>
      <origin xyz="0.00373847392098165 -1.52445893424664E-10 0.0264265327292804" rpy="0 0 0"/>
      <mass value="0.326392759093976"/>
      <inertia ixx="0.000224202818963409" ixy="-1.96773409840189E-13" ixz="-2.99224798427881E-05" iyy="0.000286616921839535" iyz="-3.10964171549742E-13" izz="0.000349957607258242"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="https://raw.githubusercontent.com/pondersome/roarm_ws_em0/refs/heads/ros2-humble/src/roarm_main/roarm_description//meshes/base_link.STL"/>
      </geometry>
      <material name="">
        <color rgba="0.752941176470588 0.752941176470588 0.752941176470588 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="https://raw.githubusercontent.com/pondersome/roarm_ws_em0/refs/heads/ros2-humble/src/roarm_main/roarm_description//meshes/base_link.STL"/>
      </geometry>
    </collision>
  </link>
  <link name="link1">
    <inertial>
      <origin xyz="-3.11559626639357E-12 9.70240468164796E-06 -0.0146667419724465" rpy="0 0 0"/>
      <mass value="0.0928887601998397"/>
      <inertia ixx="6.09210980255232E-05" ixy="-2.79207542479444E-18" ixz="4.24841739939559E-15" iyy="2.53493566630784E-05" iyz="2.16442830142331E-08" izz="4.88050835919342E-05"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="https://raw.githubusercontent.com/pondersome/roarm_ws_em0/refs/heads/ros2-humble/src/roarm_main/roarm_description//meshes/link1.STL"/>
      </geometry>
      <material name="">
        <color rgba="0.752941176470588 0.752941176470588 0.752941176470588 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="https://raw.githubusercontent.com/pondersome/roarm_ws_em0/refs/heads/ros2-humble/src/roarm_main/roarm_description//meshes/link1.STL"/>
      </geometry>
    </collision>
  </link>
  <joint name="base_link_to_link1" type="revolute">
    <origin xyz="0.0100000008759151 0 0.123059270461044" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="link1"/>
    <axis xyz="0 0 -1"/>
    <limit lower="-3.1416" upper="3.1416" effort="0" velocity="0"/>
  </joint>
  <link name="link2">
    <inertial>
      <origin xyz="0.122021157535333 6.61338425981053E-05 -9.50311342843365E-05" rpy="0 0 0"/>
      <mass value="0.0895817175833772"/>
      <inertia ixx="2.6269354271334E-05" ixy="-2.55344668379587E-06" ixz="-3.90237379833709E-09" iyy="0.000350369536011831" iyz="-6.82396195680563E-09" izz="0.000333056833230846"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="https://raw.githubusercontent.com/pondersome/roarm_ws_em0/refs/heads/ros2-humble/src/roarm_main/roarm_description//meshes/link2.STL"/>
      </geometry>
      <material name="">
        <color rgba="0.752941176470588 0.752941176470588 0.752941176470588 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="https://raw.githubusercontent.com/pondersome/roarm_ws_em0/refs/heads/ros2-humble/src/roarm_main/roarm_description//meshes/link2.STL"/>
      </geometry>
    </collision>
  </link>
  <joint name="link1_to_link2" type="revolute">
    <origin xyz="0 0 0" rpy="-1.5707963267949 -1.5707963267949 0"/>
    <parent link="link1"/>
    <child link="link2"/>
    <axis xyz="0 0 -1"/>
    <limit lower="-1.5708" upper="1.5708" effort="0" velocity="0"/>
  </joint>
  <link name="link3">
    <inertial>
      <origin xyz="-0.00076280945740961 -0.114419656440929 -0.000177274968057332" rpy="0 0 0"/>
      <mass value="0.123009044986454"/>
      <inertia ixx="0.000104111883333949" ixy="1.06443789581137E-07" ixz="1.54925816846055E-07" iyy="1.46357151076022E-05" iyz="-1.6435520156444E-07" izz="0.000104009297710106"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="https://raw.githubusercontent.com/pondersome/roarm_ws_em0/refs/heads/ros2-humble/src/roarm_main/roarm_description//meshes/link3.STL"/>
      </geometry>
      <material name="">
        <color rgba="0.752941176470588 0.752941176470588 0.752941176470588 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="https://raw.githubusercontent.com/pondersome/roarm_ws_em0/refs/heads/ros2-humble/src/roarm_main/roarm_description//meshes/link3.STL"/>
      </geometry>
    </collision>
  </link>
  <joint name="link2_to_link3" type="revolute">
    <origin xyz="0.236815132922094 0.0300023995170449 0" rpy="0 0 1.5707963267949"/>
    <parent link="link2"/>
    <child link="link3"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="3.1416" effort="0" velocity="0"/>
  </joint>
  <link name="gripper_link">
    <inertial>
      <origin xyz="0.0288857415118809 -0.000776842039187049 -0.000779764534254559" rpy="0 0 0"/>
      <mass value="0.00365697683069397"/>
      <inertia ixx="1.15139272656909E-06" ixy="1.31250385773157E-07" ixz="-2.41954534039415E-13" iyy="2.33034171341163E-06" iyz="4.1392257505497E-13" izz="1.47179863096272E-06"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="https://raw.githubusercontent.com/pondersome/roarm_ws_em0/refs/heads/ros2-humble/src/roarm_main/roarm_description//meshes/gripper_link.STL"/>
      </geometry>
      <material name="">
        <color rgba="0.219607843137255 0.219607843137255 0.219607843137255 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="https://raw.githubusercontent.com/pondersome/roarm_ws_em0/refs/heads/ros2-humble/src/roarm_main/roarm_description//meshes/gripper_link.STL"/>
      </geometry>
    </collision>
  </link>
  <joint name="link3_to_gripper_link" type="revolute">
    <origin xyz="0.002906 -0.21599 -0.00066683" rpy="-1.5708 0 -1.5708"/>
    <parent link="link3"/>
    <child link="gripper_link"/>
    <axis xyz="0 0 1"/>
    <limit lower="0" upper="1.5" effort="0" velocity="0"/>
  </joint>
  <link name="hand_tcp"/>
  <joint name="link3_to_hand_tcp" type="fixed">
    <origin xyz="0.002 -0.2802 0" rpy="-1.5708 0 -1.5708"/>
    <parent link="link3"/>
    <child link="hand_tcp"/>
    <axis xyz="0 0 0"/>
  </joint>
</robot>`;
        robotScene.applyURDF(inlineURDF);
        return;
      }
      
      const response = await fetch(t.url);
      const urdfText = await response.text();

      robotScene.applyURDF(urdfText);
    });
  });

  // Recursively set the color to white
  function setControlColor(control : GUI.Control) {
    control.color = "white";
    if (control instanceof GUI.Container) {
      control.children.forEach((c) => {
        setControlColor(c);
      });
    }
  }

  setControlColor(testSelection);
}

// Main function that gets executed once the webview DOM loads
export async function RenderTestMain() {
  const canvas = document.getElementById("renderCanvas");
  const canvasElement = canvas as unknown as HTMLCanvasElement;

  currentRobotScene = new RobotScene();
  currentRobotScene.createScene(canvasElement);
  if (currentRobotScene.scene === undefined || currentRobotScene.engine === undefined) {
    return;
  }

  currentRobotScene.scene.debugLayer.show();
  currentRobotScene.createUI();
  addTestToRobotScene(currentRobotScene);

  currentRobotScene.engine.runRenderLoop(function () {
    if (currentRobotScene !== undefined && currentRobotScene.scene !== undefined) {
      currentRobotScene.scene.render();
    }
  });
  
  currentRobotScene.engine.resize();
  
  window.addEventListener("resize", function () {
    if (currentRobotScene !== undefined && currentRobotScene.engine !== undefined) {
      currentRobotScene.engine.resize();
    }
  });  
}
