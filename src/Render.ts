import * as BABYLON from 'babylonjs';
import * as Materials from 'babylonjs-materials';
import * as urdf from './urdf';
import {Robot} from './Robot';

function applyAxisToTransform(scene : BABYLON.Scene, t : BABYLON.TransformNode | undefined) {
  if (t) {
    let a = new BABYLON.AxesViewer(scene, .5);
    a.xAxis.parent = t;
    a.yAxis.parent = t;
    a.zAxis.parent = t;
  }
}

function applyAxisToLink(scene : BABYLON.Scene, robot : Robot, l : string) {
  let r = robot.links.get(l);
  if (r && r.visuals[0].transform) {
    applyAxisToTransform(scene, r.visuals[0].transform);
  }
}

function applyAxisToJoint(scene : BABYLON.Scene, robot : Robot, j : string) {
  let r = robot.joints.get(j);
  if (r && r.transform) {
    applyAxisToTransform(scene, r.transform);
  }
}

var createScene = async function (engine : BABYLON.Engine, canvas : HTMLCanvasElement) : Promise<BABYLON.Scene>{
  let scene = new BABYLON.Scene(engine);

  scene.useRightHandedSystem = true;
  scene.clearColor = BABYLON.Color4.FromColor3(BABYLON.Color3.Black());// TODO (polyhobbyist) Make this configurable

  var radius = 5; // TODO (polyhobbyist): make this configurable

  // This creates and positions a free camera (non-mesh)
  var camera = new BABYLON.ArcRotateCamera("camera1", - Math.PI / 3, 5 * Math.PI / 12, radius, new BABYLON.Vector3(0, 0, 0), scene);
  camera.wheelDeltaPercentage = 0.01;
  camera.minZ = 0.1;

  const light = new BABYLON.HemisphericLight("light", new BABYLON.Vector3(0, 1, 0), scene);

  // This attaches the camera to the canvas
  camera.attachControl(canvas, true);

  var groundMaterial = new Materials.GridMaterial("groundMaterial", scene);
  groundMaterial.majorUnitFrequency = 1;
  groundMaterial.minorUnitVisibility = 0.5;
  groundMaterial.gridRatio = 2;
  groundMaterial.opacity = 0.8;
  groundMaterial.useMaxLine = true;
  groundMaterial.lineColor = BABYLON.Color3.Green();  // TODO (polyhobbyist) Make this configurable

  var ground = BABYLON.MeshBuilder.CreateGround("ground", {width: 50, height: 50}, scene);
  ground.material = groundMaterial;


  return scene;
};

async function applyURDF(scene : BABYLON.Scene, urdfText : string) {
  if (scene == undefined) {
    return;
  }
  
  try {
    let currentRobot = await urdf.deserializeUrdfToRobot(urdfText);
    currentRobot.create(scene);

  } catch (err) {
  }

  //applyAxisToTransform(scene, robot.transform);

  //applyAxisToLink(scene, robot, "base_link");
  //applyAxisToLink(scene, robot, "right_leg");

  //applyAxisToJoint(scene, robot, "base_to_right_leg");

}
// Main function that gets executed once the webview DOM loads
async function RenderMain() {

  let u = /*xml*/ 
  `<?xml version="1.0" ?>
  <robot name="robot">

  </robot>
  `;
  
  const canvas = document.getElementById("renderCanvas") as HTMLCanvasElement; // Get the canvas element
  const engine = new BABYLON.Engine(canvas, true); // Generate the BABYLON 3D engine
  let scene = await createScene(engine, canvas);

  applyURDF(scene, u);

  engine.runRenderLoop(function () {
    if (scene != undefined) {
      scene.render();
    }
  });
  
  engine.resize();
  
  window.addEventListener("resize", function () {
      engine.resize();
  });  
  
  
}

  // Just like a regular webpage we need to wait for the webview
  // DOM to load before we can reference any of the HTML elements
  // or toolkit components
  window.addEventListener("load", RenderMain);
  
