import * as BABYLON from 'babylonjs';
import * as Materials from 'babylonjs-materials';
import * as urdf from './urdf';
import {Robot} from './Robot';
import * as GUI from 'babylonjs-gui';
import * as ColladaFileLoader from '@polyhobbyist/babylon-collada-loader';

var statusLabel = new GUI.TextBlock();
let axisList : BABYLON.PositionGizmo[] = [];

function addAxisToTransform(scene : BABYLON.Scene, layer: BABYLON.UtilityLayerRenderer, transform : BABYLON.TransformNode | undefined) {
  if (transform) {
    let axis = new BABYLON.PositionGizmo(layer);
    axis.scaleRatio = 0.5
    axis.attachedNode = transform;
    axisList.push(axis);

    let drag = () => {
      if (transform) {
      statusLabel.text = transform.name + 
      "\nX: " + transform.position.x.toFixed(5) + 
      "\nY: " + transform.position.y.toFixed(5) + 
      "\nZ: " + transform.position.z.toFixed(5);
      statusLabel.linkOffsetY = -100;
      statusLabel.linkWithMesh(transform);
      }
    }

    axis.xGizmo.dragBehavior.onDragObservable.add(drag);
    axis.yGizmo.dragBehavior.onDragObservable.add(drag);
    axis.zGizmo.dragBehavior.onDragObservable.add(drag);
      
  }
}

function toggleAxisOnRobot(scene : BABYLON.Scene, layer: BABYLON.UtilityLayerRenderer, robot : Robot) {

  if (axisList.length == 0) {
    robot.joints.forEach((j) => {
      addAxisToTransform(scene, layer, j.transform);
    });
    robot.links.forEach((l) => {
      l.visuals.forEach((v) => {
        addAxisToTransform(scene, layer, l.transform);
      });
    });
  } else {
    axisList.forEach((a) => {
      a.dispose();
    });
    axisList = [];
  }
}

let rotationGizmos : BABYLON.RotationGizmo[] = [];

function addRotationToTransform(scene : BABYLON.Scene, layer: BABYLON.UtilityLayerRenderer, transform : BABYLON.TransformNode | undefined) {
  if (transform) {
    let rotationGizmo = new BABYLON.RotationGizmo(layer);
    rotationGizmo.scaleRatio = 0.5
    rotationGizmo.attachedNode = transform;
    rotationGizmos.push(rotationGizmo);

    let drag = () => {
      if (transform) {
      statusLabel.text = transform.name + 
      "\nR:" + transform.rotation.x.toFixed(5) + 
      "\nP:" + transform.rotation.y.toFixed(5) + 
      "\nY:" + transform.rotation.z.toFixed(5);
      statusLabel.linkOffsetY = -100;
      statusLabel.linkWithMesh(transform);
      }
    }

    rotationGizmo.xGizmo.dragBehavior.onDragObservable.add(drag);
    rotationGizmo.yGizmo.dragBehavior.onDragObservable.add(drag);
    rotationGizmo.zGizmo.dragBehavior.onDragObservable.add(drag);
  }

}

function toggleAxisRotationOnRobot(ui: GUI.AdvancedDynamicTexture, scene : BABYLON.Scene, layer: BABYLON.UtilityLayerRenderer, robot : Robot) {
  if (rotationGizmos.length == 0) {
    robot.joints.forEach((j) => {
      addRotationToTransform(scene, layer, j.transform);
    });

    robot.links.forEach((l) => {
      l.visuals.forEach((v) => {
        addRotationToTransform(scene, layer, v.transform);
      });
    });
  } else {
    rotationGizmos.forEach((a) => {
      a.dispose();
    });
    rotationGizmos = [];
  }
}

var createScene = async function (engine : BABYLON.Engine, canvas : HTMLCanvasElement) : Promise<BABYLON.Scene>{
  let scene = new BABYLON.Scene(engine);
  if (BABYLON.SceneLoader) {
    //Add this loader into the register plugin
    BABYLON.SceneLoader.RegisterPlugin(new ColladaFileLoader.DAEFileLoader());
  }

  scene.useRightHandedSystem = true;
  scene.clearColor = new BABYLON.Color4(0.4, 0.4, 0.4, 1.0);// TODO (polyhobbyist) Make this configurable

  var radius = 30; // TODO (polyhobbyist): make this configurable

  // This creates and positions a free camera (non-mesh)
  var camera = new BABYLON.ArcRotateCamera("camera1", - Math.PI / 3, 5 * Math.PI / 12, radius, new BABYLON.Vector3(0, 0, 0), scene);
  camera.wheelDeltaPercentage = 0.01;
  camera.minZ = 0.1;

  const light = new BABYLON.HemisphericLight("light", new BABYLON.Vector3(0, 20, 20), scene);

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
  ground.isPickable = false;


  return scene;
};

async function applyURDF(scene : BABYLON.Scene, urdfText : string) : Promise<Robot | undefined>{
  if (scene == undefined) {
    return;
  }
  
  try {
    let currentRobot = await urdf.deserializeUrdfToRobot(urdfText);
    currentRobot.create(scene);

    return currentRobot;
    } catch (err) {
  }

  return undefined;
}

function createUI(scene : BABYLON.Scene, robot : Robot) {
  var advancedTexture = GUI.AdvancedDynamicTexture.CreateFullscreenUI("UI");

  statusLabel.color = "white";
  statusLabel.textHorizontalAlignment = GUI.Control.HORIZONTAL_ALIGNMENT_LEFT;
  statusLabel.horizontalAlignment = GUI.Control.HORIZONTAL_ALIGNMENT_CENTER;
  statusLabel.resizeToFit = true;
  statusLabel.outlineColor = "green";
  statusLabel.outlineWidth = 2.0;
  advancedTexture.addControl(statusLabel);

  var toolbar = new GUI.StackPanel();
  toolbar.paddingTop = "10px";
  toolbar.paddingLeft = "10px";
  toolbar.width = "300px";
  toolbar.height = "50px";
  toolbar.fontSize = "14px";
  toolbar.horizontalAlignment = GUI.Control.HORIZONTAL_ALIGNMENT_LEFT;
  toolbar.verticalAlignment = GUI.Control.VERTICAL_ALIGNMENT_TOP;
  toolbar.isVertical = false;
  advancedTexture.addControl(toolbar);


  var utilLayer = new BABYLON.UtilityLayerRenderer(scene);

  const gizmoManager = new BABYLON.GizmoManager(scene);
  gizmoManager.usePointerToAttachGizmos = false;

  var button = GUI.Button.CreateSimpleButton("axisButton", "Axis");
  button.width = 0.2;
  button.height = "40px";
  button.color = "white";
  button.background = "green";
  button.onPointerClickObservable.add(function() {

    toggleAxisOnRobot(scene, utilLayer, robot);
  });
  toolbar.addControl(button);

  var buttonRotate = GUI.Button.CreateSimpleButton("rotatioButton", "Rotation");
  buttonRotate.width = 0.2;
  buttonRotate.height = "40px";
  buttonRotate.color = "white";
  buttonRotate.background = "green";
  buttonRotate.onPointerClickObservable.add(function() {
    toggleAxisRotationOnRobot(advancedTexture, scene, utilLayer, robot);
  });  

  toolbar.addControl(buttonRotate);

}
// Main function that gets executed once the webview DOM loads
export async function RenderMain() {

  let u = /*xml*/ 
  `<?xml version="1.0"?>
  <robot name="bb_head">
  
    <material name="red">
      <color rgba=".8 0 0 1"/>
    </material>
  
    <link name="base_link">
      <visual>
        <geometry>
          <mesh filename="https://raw.githubusercontent.com/polyhobbyist/babylon-collada-loader/main/test/testdata/leo.dae" scale="0.001 0.001 0.001" />
        </geometry>
      </visual>
    </link>
     </robot>
  `;
  
  const canvas = document.getElementById("renderCanvas") as HTMLCanvasElement; // Get the canvas element
  const engine = new BABYLON.Engine(canvas, true); // Generate the BABYLON 3D engine
  let scene = await createScene(engine, canvas);
  //scene.debugLayer.show();

  let robot = await applyURDF(scene, u);

  if (robot != undefined) {
    createUI(scene, robot);
  }

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
