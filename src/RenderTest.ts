/*
 * Copyright (c) 2025 Lou Amadio and Ranch Hand Robotics, LLC
 * All rights reserved.
 */

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
    {name: "Basic", url: "https://raw.githubusercontent.com/Ranch-Hand-Robotics/babylon_ros/main/test/testdata/basic.urdf"},
    {name: "Basic Joint", url: "https://raw.githubusercontent.com/Ranch-Hand-Robotics/babylon_ros/main/test/testdata/basic_with_joint.urdf"},
    {name: "Basic Revolute Joint", url: "https://raw.githubusercontent.com/Ranch-Hand-Robotics/babylon_ros/main/test/testdata/basic_with_joint_with_effort.urdf"},
    {name: "Planar Joint", url: "https://raw.githubusercontent.com/Ranch-Hand-Robotics/babylon_ros/main/test/testdata/basic_with_joint_planar.urdf"},
    {name: "Prismatic Joint", url: "https://raw.githubusercontent.com/Ranch-Hand-Robotics/babylon_ros/main/test/testdata/basic_with_joint_prismatic.urdf"},
    {name: "Basic Material", url: "https://raw.githubusercontent.com/Ranch-Hand-Robotics/babylon_ros/main/test/testdata/basic_with_material.urdf"},
    {name: "Basic Remote Mesh", url: "https://raw.githubusercontent.com/Ranch-Hand-Robotics/babylon_ros/main/test/testdata/basic_with_remote_mesh.urdf"},
    {name: "Basic with STL", url: "https://raw.githubusercontent.com/Ranch-Hand-Robotics/babylon_ros/main/test/testdata/basic_with_stl_mesh.urdf"},
    {name: "Orientation", url: "https://raw.githubusercontent.com/Ranch-Hand-Robotics/babylon_ros/main/test/testdata/orientation.urdf"},
    {name: "Bad", url: "https://raw.githubusercontent.com/Ranch-Hand-Robotics/babylon_ros/main/test/testdata/bad.urdf"},
    {name: "DAE", url: "https://raw.githubusercontent.com/Ranch-Hand-Robotics/babylon_ros/main/test/testdata/leo_chassis.urdf"},
  ];

  var robotTestList = [ 
    {name: "leo", url: "https://raw.githubusercontent.com/Ranch-Hand-Robotics/babylon_ros/main/test/testdata/leo.urdf"},
    {name: "BB", url: "https://raw.githubusercontent.com/Ranch-Hand-Robotics/babylon_ros/main/test/testdata/bb.urdf"},
    {name: "Motoman", url: "https://raw.githubusercontent.com/Ranch-Hand-Robotics/babylon_ros/main/test/testdata/motoman.urdf"},
    {name: "Arti Robot", url: "https://raw.githubusercontent.com/Ranch-Hand-Robotics/babylon_ros/main/test/testdata/arti.urdf"},
    {name: "Mule", url: "https://raw.githubusercontent.com/Ranch-Hand-Robotics/babylon_ros/main/test/testdata/mule.urdf"},
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
<robot name="planar_joint_example">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.25 0.25 0.1"/>
      </geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.1"/>
      </geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Moving Link -->
  <link name="moving_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.2"/>
      </geometry>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.2"/>
      </geometry>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <!-- Prismatic Joint -->
  <joint name="prismatic_joint" type="prismatic">
    <parent link="base_link"/>
    <child link="moving_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="1000" velocity="1.0" lower="-0.25" upper="0.25"/> <!-- Limits for translation -->
  </joint>

</robot>
`;
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
