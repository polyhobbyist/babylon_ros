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

// Test menu state variables
let testMenuButton: GUI.Button | undefined = undefined;
let testMenuPanel: GUI.StackPanel | undefined = undefined;
let testMenuScrollViewer: GUI.ScrollViewer | undefined = undefined;
let isTestMenuExpanded: boolean = false;
let testMenuContainer: GUI.Rectangle | undefined = undefined;

function createTestMenuButton(name: string, text: string, onClick: () => void): GUI.Button {
  var button = GUI.Button.CreateSimpleButton(name, text);
  button.widthInPixels = 200;
  button.heightInPixels = 28;
  button.color = "white";
  button.cornerRadius = 4;
  button.background = "rgba(0, 120, 215, 0.8)";
  button.fontSize = "11px";
  button.paddingTopInPixels = 3;
  button.paddingBottomInPixels = 3;
  button.onPointerUpObservable.add(onClick);
  return button;
}

function createTestGroupHeader(text: string): GUI.TextBlock {
  const header = new GUI.TextBlock();
  header.text = text;
  header.color = "white";
  header.fontSize = "14px";
  header.fontWeight = "bold";
  header.heightInPixels = 25;
  header.textHorizontalAlignment = GUI.Control.HORIZONTAL_ALIGNMENT_CENTER;
  header.textVerticalAlignment = GUI.Control.VERTICAL_ALIGNMENT_CENTER;
  return header;
}

function createTestGroupSeparator(): GUI.Rectangle {
  const separator = new GUI.Rectangle();
  separator.widthInPixels = 160;
  separator.heightInPixels = 1;
  separator.color = "rgba(255, 255, 255, 0.3)";
  separator.background = "rgba(255, 255, 255, 0.3)";
  separator.thickness = 0;
  return separator;
}

function toggleTestMenu() {
  isTestMenuExpanded = !isTestMenuExpanded;
  
  if (testMenuContainer) {
    testMenuContainer.isVisible = isTestMenuExpanded;
  }
  
  // Update hamburger button text
  if (testMenuButton) {
    testMenuButton.textBlock!.text = isTestMenuExpanded ? "✕" : "Tests";
  }
}

function addTestToRobotScene(robotScene : RobotScene) {
  if (robotScene.UILayer === undefined) {
    return
  }

  // Create hamburger-style test menu button
  testMenuButton = GUI.Button.CreateSimpleButton("testMenuButton", "Tests");
  testMenuButton.widthInPixels = 60;
  testMenuButton.heightInPixels = 40;
  testMenuButton.color = "white";
  testMenuButton.cornerRadius = 5;
  testMenuButton.background = "rgba(0, 0, 0, 0.8)";
  testMenuButton.fontSize = "12px";
  testMenuButton.fontWeight = "bold";
  testMenuButton.horizontalAlignment = GUI.Control.HORIZONTAL_ALIGNMENT_RIGHT;
  testMenuButton.verticalAlignment = GUI.Control.VERTICAL_ALIGNMENT_TOP;
  testMenuButton.leftInPixels = -10;
  testMenuButton.topInPixels = 10;
  
  testMenuButton.onPointerUpObservable.add(() => {
    toggleTestMenu();
  });
  
  robotScene.UILayer.addControl(testMenuButton);

  // Create menu panel container
  testMenuContainer = new GUI.Rectangle("testMenuContainer");
  testMenuContainer.widthInPixels = 250;
  testMenuContainer.height = "80%";
  testMenuContainer.cornerRadius = 8;
  testMenuContainer.color = "white";
  testMenuContainer.thickness = 2;
  testMenuContainer.background = "rgba(0, 0, 0, 0.9)";
  testMenuContainer.horizontalAlignment = GUI.Control.HORIZONTAL_ALIGNMENT_RIGHT;
  testMenuContainer.verticalAlignment = GUI.Control.VERTICAL_ALIGNMENT_CENTER;
  testMenuContainer.leftInPixels = -10;
  testMenuContainer.isVisible = false;
  
  robotScene.UILayer.addControl(testMenuContainer);

  // Create scroll viewer for the menu
  testMenuScrollViewer = new GUI.ScrollViewer("testMenuScrollViewer");
  testMenuScrollViewer.thickness = 0;
  testMenuScrollViewer.color = "transparent";
  testMenuScrollViewer.background = "transparent";
  testMenuScrollViewer.widthInPixels = 240;
  testMenuScrollViewer.height = "95%";
  testMenuScrollViewer.verticalAlignment = GUI.Control.VERTICAL_ALIGNMENT_TOP;
  
  testMenuContainer.addControl(testMenuScrollViewer);

  // Create the menu panel (vertical stack)
  testMenuPanel = new GUI.StackPanel("testMenuPanel");
  testMenuPanel.isVertical = true;
  testMenuPanel.spacing = 3;
  testMenuPanel.paddingTopInPixels = 10;
  testMenuPanel.paddingBottomInPixels = 10;
  testMenuPanel.paddingLeftInPixels = 10;
  testMenuPanel.paddingRightInPixels = 10;
  
  testMenuScrollViewer.addControl(testMenuPanel);

  // Test data
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

  // Add Basic Tests group
  const basicTestsHeader = createTestGroupHeader("Basic Tests");
  testMenuPanel.addControl(basicTestsHeader);
  
  const basicTestsSeparator = createTestGroupSeparator();
  testMenuPanel.addControl(basicTestsSeparator);

  basicTestList.forEach((t) => {
    const button = createTestMenuButton(t.name, t.name, async () => {
      toggleTestMenu(); // Close menu after selection
      const response = await fetch(t.url);
      const urdfText = await response.text();
      robotScene.applyURDF(urdfText);
    });
    if (testMenuPanel) {
      testMenuPanel.addControl(button);
    }
  });

  // Add some spacing between groups
  const spacer1 = new GUI.Rectangle();
  spacer1.heightInPixels = 15;
  spacer1.color = "transparent";
  spacer1.background = "transparent";
  spacer1.thickness = 0;
  testMenuPanel.addControl(spacer1);

  // Add Robot Tests group
  const robotTestsHeader = createTestGroupHeader("Robot Tests");
  testMenuPanel.addControl(robotTestsHeader);
  
  const robotTestsSeparator = createTestGroupSeparator();
  testMenuPanel.addControl(robotTestsSeparator);

  robotTestList.forEach((t) => {
    const button = createTestMenuButton(t.name, t.name, async () => {
      toggleTestMenu(); // Close menu after selection
      
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
    if (testMenuPanel) {
      testMenuPanel.addControl(button);
    }
  });
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
