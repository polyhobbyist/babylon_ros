import * as BABYLON from 'babylonjs';
import * as Materials from 'babylonjs-materials';
import * as urdf from './urdf';
import {Robot} from './Robot';
import {Joint} from './Joint';
import {Link} from './Link';
import {Visual} from './Visual';
import { RobotScene } from './RobotScene';

import * as GUI3D from 'babylonjs-gui';
import * as ColladaFileLoader from '@polyhobbyist/babylon-collada-loader';

let currentRobotScene : RobotScene | undefined = undefined;

function addTestToRobotScene(robotScene : RobotScene) {
  if (!robotScene.uiScene || !robotScene.UI3DManager) {
    return;
  }

  
  var testList = [ 
    {name: "Basic", url: "https://raw.githubusercontent.com/polyhobbyist/babylon_ros/main/test/testdata/basic.urdf"},
    {name: "Basic Joint", url: "https://raw.githubusercontent.com/polyhobbyist/babylon_ros/main/test/testdata/basic_with_joint.urdf"},
    {name: "Basic Revolute Joint", url: "https://raw.githubusercontent.com/polyhobbyist/babylon_ros/main/test/testdata/basic_with_joint_with_effort.urdf"},
    {name: "Basic Material", url: "https://raw.githubusercontent.com/polyhobbyist/babylon_ros/main/test/testdata/basic_with_material.urdf"},
    {name: "Basic Remote Mesh", url: "https://raw.githubusercontent.com/polyhobbyist/babylon_ros/main/test/testdata/basic_with_remote_mesh.urdf"},
    {name: "Basic with STL", url: "https://raw.githubusercontent.com/polyhobbyist/babylon_ros/main/test/testdata/basic_with_stl_mesh.urdf"},
    {name: "Orientation", url: "https://raw.githubusercontent.com/polyhobbyist/babylon_ros/main/test/testdata/orientation.urdf"},
    {name: "Bad", url: "https://raw.githubusercontent.com/polyhobbyist/babylon_ros/main/test/testdata/bad.urdf"},
    {name: "DAE", url: "https://raw.githubusercontent.com/polyhobbyist/babylon_ros/main/test/testdata/leo_chassis.urdf"},
    {name: "leo", url: "https://raw.githubusercontent.com/polyhobbyist/babylon_ros/main/test/testdata/leo.urdf"},
    {name: "BB", url: "https://raw.githubusercontent.com/polyhobbyist/babylon_ros/main/test/testdata/bb.urdf"},
    {name: "Motoman", url: "https://raw.githubusercontent.com/polyhobbyist/babylon_ros/main/test/testdata/motoman.urdf"},
    {name: "Arti Robot", url: "https://raw.githubusercontent.com/polyhobbyist/babylon_ros/main/test/testdata/arti.urdf"},
  ];

  // Add buttons for basic tests
  testList.forEach((t) => {
    const button = new GUI3D.TouchHolographicButton(t.name);
    const buttonText = new GUI3D.TextBlock();
    buttonText.text = t.name;
    buttonText.color = "white";
    buttonText.fontSize = 12;
    button.content = buttonText;
    
    button.onPointerUpObservable.add(async () => {
      const response = await fetch(t.url);
      const urdfText = await response.text();
      
      robotScene.applyURDF(urdfText);
    });
    
    robotScene.acitonMenu?.addButton(button);
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
  await currentRobotScene.createUI();
  addTestToRobotScene(currentRobotScene);

  currentRobotScene.engine.runRenderLoop(function () {
    if (currentRobotScene !== undefined && currentRobotScene.scene !== undefined && currentRobotScene.uiScene !== undefined) {
      // Update the scene
      currentRobotScene.scene.render();
      currentRobotScene.uiScene.render();
    }
  });
  
  currentRobotScene.engine.resize();
  
  window.addEventListener("resize", function () {
    if (currentRobotScene !== undefined && currentRobotScene.engine !== undefined) {
      currentRobotScene.engine.resize();
    }
  });  
}
