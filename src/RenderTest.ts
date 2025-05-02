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
<robot name="origins">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
  </link>

  <link name="right_leg">
    <visual>
      <geometry>
        <box size="0.6 0.1 0.2"/>
      </geometry>
      <origin rpy="0 1.57075 0" xyz="0 0 -0.3"/>
    </visual>
  </link>

  <joint name="base_to_right_leg" type="revolute">
    <parent link="base_link"/>
    <child link="right_leg"/>
    <origin xyz="0 -0.22 0.25"/>
    <limit effort="1.1369" lower="-1" upper="1" velocity="4.36" />
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
