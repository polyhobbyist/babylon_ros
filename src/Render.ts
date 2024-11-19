import * as BABYLON from 'babylonjs';
import * as Materials from 'babylonjs-materials';
import * as urdf from './urdf';
import {Robot} from './Robot';
import {Joint} from './Joint';
import {Link} from './Link';
import {Visual} from './Visual';

import * as GUI from 'babylonjs-gui';
import * as ColladaFileLoader from '@polyhobbyist/babylon-collada-loader';

let ground : BABYLON.GroundMesh | undefined = undefined;
let camera : BABYLON.ArcRotateCamera | undefined = undefined;
let currentRobot : Robot | undefined = undefined;
let worldAxis : BABYLON.TransformNode | undefined = undefined;

var statusLabel = new GUI.TextBlock();
let jointAxisList : BABYLON.PositionGizmo[] = [];
let linkAxisList : BABYLON.PositionGizmo[] = [];

function clearAxisGizmos() {
  linkAxisList.forEach((a) => {
    a.dispose();
  });
  linkAxisList = [];

  jointAxisList.forEach((a) => {
    a.dispose();
  });
  jointAxisList = [];
}

function clearStatus() {
  statusLabel.text = "";
}

function addAxisToTransform(list : BABYLON.PositionGizmo[], scene : BABYLON.Scene, layer: BABYLON.UtilityLayerRenderer, transform : BABYLON.TransformNode | undefined) {
  if (transform) {
    let axis = new BABYLON.PositionGizmo(layer);
    axis.scaleRatio = 0.5
    axis.attachedNode = transform;
    list.push(axis);

    let drag = () => {
      if (transform) {
        statusLabel.text = transform.name + 
        "\nX: " + transform.position.x.toFixed(6) + 
        "\nY: " + transform.position.y.toFixed(6) + 
        "\nZ: " + transform.position.z.toFixed(6);
        statusLabel.linkOffsetY = -100;
      statusLabel.linkWithMesh(transform);
      }
    }

    axis.xGizmo.dragBehavior.onDragObservable.add(drag);
    axis.yGizmo.dragBehavior.onDragObservable.add(drag);
    axis.zGizmo.dragBehavior.onDragObservable.add(drag);
      
  }
}

function toggleAxisOnRobot(jointOrLink : Boolean, scene : BABYLON.Scene, layer: BABYLON.UtilityLayerRenderer, robot : Robot | undefined) {
  let whichAxis = jointOrLink ? jointAxisList : linkAxisList;

  if (robot == undefined) {
    return;
  }

  if (whichAxis.length == 0) {
    if (jointOrLink) {
      robot.joints.forEach((j: Joint) => {
        addAxisToTransform(whichAxis, scene, layer, j.transform);
      });
    } else {
      robot.links.forEach((l) => {
        l.visuals.forEach((v) => {
          addAxisToTransform(whichAxis, scene, layer, v.transform);
        });
      });
    }
  } else {
    clearAxisGizmos();
    clearStatus();
  }
}

let jointRotationGizmos : BABYLON.RotationGizmo[] = [];
let linkRotationGizmos : BABYLON.RotationGizmo[] = [];

function clearRotationGizmos() {
  jointRotationGizmos.forEach((a) => {
    a.dispose();
  });
  jointRotationGizmos = [];
  linkRotationGizmos.forEach((a) => {
    a.dispose();
  });
  linkRotationGizmos = [];
}

function addRotationToTransform(list : BABYLON.RotationGizmo[], scene : BABYLON.Scene, layer: BABYLON.UtilityLayerRenderer, transform : BABYLON.TransformNode | undefined) {
  if (transform) {
    let rotationGizmo = new BABYLON.RotationGizmo(layer);
    rotationGizmo.scaleRatio = 0.5
    rotationGizmo.attachedNode = transform;
    list.push(rotationGizmo);

    let drag = () => {
      if (transform) {
        statusLabel.text = transform.name + 
        "\nR:" + transform.rotation.x.toFixed(6) + 
        "\nP:" + transform.rotation.y.toFixed(6) + 
        "\nY:" + transform.rotation.z.toFixed(6);
        statusLabel.linkOffsetY = -100;
      statusLabel.linkWithMesh(transform);
      }
    }

    rotationGizmo.xGizmo.dragBehavior.onDragObservable.add(drag);
    rotationGizmo.yGizmo.dragBehavior.onDragObservable.add(drag);
    rotationGizmo.zGizmo.dragBehavior.onDragObservable.add(drag);
  }

}

function toggleAxisRotationOnRobot(jointOrLink : Boolean, ui: GUI.AdvancedDynamicTexture, scene : BABYLON.Scene, layer: BABYLON.UtilityLayerRenderer, robot : Robot | undefined) {
  if (robot == undefined) {
    return;
  }

  let whichList = jointOrLink ? jointRotationGizmos : linkRotationGizmos;
  if (whichList.length == 0) {
    if (jointOrLink) {
      robot.joints.forEach((j: Joint) => {
        addRotationToTransform(whichList, scene, layer, j.transform);
      });
    } else {
      robot.links.forEach((l: Link) => {
        l.visuals.forEach((v: Visual) => {
          addRotationToTransform(whichList, scene, layer, v.transform);
        });
      });
    }
  } else {
    clearRotationGizmos();
    clearStatus();
  }
}

var createScene = async function (engine : BABYLON.Engine, canvas : HTMLCanvasElement) : Promise<BABYLON.Scene>{
  let scene = new BABYLON.Scene(engine);
  if (BABYLON.SceneLoader) {
    //Add this loader into the register plugin
    BABYLON.SceneLoader.RegisterPlugin(new ColladaFileLoader.DAEFileLoader());
  }

  const environment = scene.createDefaultEnvironment({
    createGround: true,
    createSkybox: false,
    enableGroundMirror: true,
    groundMirrorSizeRatio: 0.15
  });

  scene.useRightHandedSystem = true;
  scene.clearColor = new BABYLON.Color4(0.4, 0.4, 0.4, 1.0);// TODO (polyhobbyist) Make this configurable

  var radius = 10; // TODO (polyhobbyist): make this configurable

  // This creates and positions a free camera (non-mesh)
  camera = new BABYLON.ArcRotateCamera("camera1", - Math.PI / 3, 5 * Math.PI / 12, radius, new BABYLON.Vector3(0, 0, 0), scene);
  camera.wheelDeltaPercentage = 0.01;
  camera.minZ = 0.1;

  const light = new BABYLON.HemisphericLight("light", new BABYLON.Vector3(0, 20, 20), scene);

  // This attaches the camera to the canvas
  camera.attachControl(canvas, true);

  var groundMaterial = new Materials.GridMaterial("groundMaterial", scene);
  groundMaterial.majorUnitFrequency = 5;
  groundMaterial.minorUnitVisibility = 0.5;
  groundMaterial.gridRatio = 2;
  groundMaterial.opacity = 0.8;
  groundMaterial.useMaxLine = true;
  groundMaterial.lineColor = BABYLON.Color3.Green();
  groundMaterial.mainColor = BABYLON.Color3.Green();

  var ground = BABYLON.MeshBuilder.CreateGround("ground", {width: 50, height: 50}, scene);
  ground.material = groundMaterial;
  ground.isPickable = false;


  return scene;
};

async function applyURDF(scene : BABYLON.Scene, urdfUrl : string) : Promise<Robot | undefined>{
  if (scene == undefined) {
    return;
  }
  
  try {
    clearAxisGizmos();
    clearRotationGizmos();
    clearStatus();
    resetCamera();

    if (currentRobot) {
      currentRobot.dispose();
      currentRobot = undefined;
    }

    // fetch the URDF file from the URL
    const response = await fetch(urdfUrl);
    const urdfText = await response.text();

    currentRobot = await urdf.deserializeUrdfToRobot(urdfText);
    currentRobot.create(scene);

    return currentRobot;
    } catch (err: any) {
      console.error(err.message);
  }

  return undefined;
}

function resetCamera() {
  if (camera != undefined) {
    camera.alpha = - Math.PI / 3;
    camera.beta = 5 * Math.PI / 12;
    camera.target = new BABYLON.Vector3(0, 0, 0);
  }
}

function createButton(toolbar: GUI.StackPanel, name : string, text : string, scene : BABYLON.Scene, onClick : () => void) {
  var button = GUI.Button.CreateSimpleButton(name, text);
  button.width = "100px"
  button.height = "20px";
  button.color = "white";
  button.cornerRadius = 5;
  button.background = "green";
  button.onPointerUpObservable.add(onClick);
  toolbar.addControl(button);
  return button;
}

function createUI(scene : BABYLON.Scene) {
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
  toolbar.width = "700px";
  toolbar.height = "50px";
  toolbar.fontSize = "14px";
  toolbar.horizontalAlignment = GUI.Control.HORIZONTAL_ALIGNMENT_LEFT;
  toolbar.verticalAlignment = GUI.Control.VERTICAL_ALIGNMENT_TOP;
  toolbar.isVertical = false;
  advancedTexture.addControl(toolbar);


  var utilLayer = new BABYLON.UtilityLayerRenderer(scene);

  const gizmoManager = new BABYLON.GizmoManager(scene);
  gizmoManager.usePointerToAttachGizmos = false;

  createButton(toolbar, "jointAxisButton", "Joint Axis", scene, () => {
    toggleAxisOnRobot(true, scene, utilLayer, currentRobot);
  });

  createButton(toolbar, "linkAxisButton", "Link Axis", scene, () => {
    toggleAxisOnRobot(false, scene, utilLayer, currentRobot);
  });

  createButton(toolbar, "jointRotationButton", "Joint Rotation", scene, () => {  
    toggleAxisRotationOnRobot(true, advancedTexture, scene, utilLayer, currentRobot);
  });

  createButton(toolbar, "linkRotationButton", "Link Rotation", scene, () => {  
    toggleAxisRotationOnRobot(false, advancedTexture, scene, utilLayer, currentRobot);
  });

  createButton(toolbar, "worldAxis", "World Axis", scene, () => {  
    toggleWorldAxis();
  });

  var testSelection = new GUI.SelectionPanel("tests");
  testSelection.width = 0.25;
  testSelection.height = 0.48;
  testSelection.horizontalAlignment = GUI.Control.HORIZONTAL_ALIGNMENT_LEFT;
  testSelection.verticalAlignment = GUI.Control.VERTICAL_ALIGNMENT_BOTTOM;

  advancedTexture.addControl(testSelection);

  var basicGroup = new GUI.RadioGroup("Basic Tests");

  var basicTestList = [ 
    {name: "Basic", url: "https://raw.githubusercontent.com/polyhobbyist/babylon_ros/main/test/testdata/basic.urdf"},
    {name: "Basic Joint", url: "https://raw.githubusercontent.com/polyhobbyist/babylon_ros/main/test/testdata/basic_with_joint.urdf"},
    {name: "Basic Material", url: "https://raw.githubusercontent.com/polyhobbyist/babylon_ros/main/test/testdata/basic_with_material.urdf"},
    {name: "Basic Remote Mesh", url: "https://raw.githubusercontent.com/polyhobbyist/babylon_ros/main/test/testdata/basic_with_remote_mesh.urdf"},
    {name: "Basic with STL", url: "https://raw.githubusercontent.com/polyhobbyist/babylon_ros/main/test/testdata/basic_with_stl_mesh.urdf"},
    {name: "Orientation", url: "https://raw.githubusercontent.com/polyhobbyist/babylon_ros/main/test/testdata/orientation.urdf"},
    {name: "Bad", url: "https://raw.githubusercontent.com/polyhobbyist/babylon_ros/main/test/testdata/bad.urdf"},
  ];

  var robotTestList = [ 
    {name: "leo", url: "https://raw.githubusercontent.com/polyhobbyist/babylon_ros/main/test/testdata/leo.urdf"},
    {name: "BB", url: "https://raw.githubusercontent.com/polyhobbyist/babylon_ros/main/test/testdata/bb.urdf"},
    {name: "Motoman", url: "https://raw.githubusercontent.com/polyhobbyist/babylon_ros/main/test/testdata/motoman.urdf"},
    {name: "Arti Robot", url: "https://raw.githubusercontent.com/polyhobbyist/babylon_ros/main/test/testdata/arti.urdf"},
  ];

  basicTestList.forEach((t) => {
    basicGroup.addRadio(t.name, () => {
      applyURDF(scene, t.url);
    });
  });
  testSelection.addGroup(basicGroup);

  var robotTestGroup = new GUI.RadioGroup("Robot Tests");
  robotTestList.forEach((t) => {
    robotTestGroup.addRadio(t.name, () => {
      applyURDF(scene, t.url);
    });
  });

  testSelection.addGroup(robotTestGroup);

  let size = 8;

  var makeTextPlane = function (text : string, color : string, size : number) {
    var dynamicTexture = new BABYLON.DynamicTexture("DynamicTexture", 50, scene, true);
    dynamicTexture.hasAlpha = true;
    dynamicTexture.drawText(text, 5, 40, "bold 36px Arial", color, "transparent", true);
    var plane = BABYLON.MeshBuilder.CreatePlane("TextPlane", {size: size}, scene);
    let material = new BABYLON.StandardMaterial("TextPlaneMaterial", scene);
    material.backFaceCulling = false;
    material.specularColor = new BABYLON.Color3(0, 0, 0);
    material.diffuseTexture = dynamicTexture;

    plane.material = material;
    return plane;
  };

  worldAxis = new BABYLON.TransformNode("worldAxis", scene);

  // Don't show it by default?
  // Disabling since this is debug ui
  // worldAxis.setEnabled(false);

  // Babylon.JS coordinate system to ROS transform
  worldAxis.rotation =  new BABYLON.Vector3(-Math.PI/2, 0, 0);

  var axisX = BABYLON.MeshBuilder.CreateLines("axisX", {points: [
      new BABYLON.Vector3(0, 0, 0), new BABYLON.Vector3(size, 0, 0), new BABYLON.Vector3(size * 0.95, 0.05 * size, 0),
      new BABYLON.Vector3(size, 0, 0), new BABYLON.Vector3(size * 0.95, -0.05 * size, 0)
  ]}, scene);
  axisX.color = new BABYLON.Color3(1, 0, 0);
  axisX.parent = worldAxis;

  var xChar = makeTextPlane("X", "red", size / 10);
  xChar.position = new BABYLON.Vector3(0.9 * size, -0.05 * size, 0);
  xChar.parent = worldAxis; 

  var axisY = BABYLON.MeshBuilder.CreateLines("axisY", {points: [
      new BABYLON.Vector3(0, 0, 0), new BABYLON.Vector3(0, size, 0), new BABYLON.Vector3(-0.05 * size, size * 0.95, 0),
      new BABYLON.Vector3(0, size, 0), new BABYLON.Vector3(0.05 * size, size * 0.95, 0)
  ]}, scene);
  axisY.color = new BABYLON.Color3(0, 1, 0);
  axisY.parent = worldAxis;

  var yChar = makeTextPlane("Y", "green", size / 10);
  yChar.position = new BABYLON.Vector3(-0.05 * size, 0.9 * size, 0);
  yChar.parent = worldAxis;

  var axisZ = BABYLON.MeshBuilder.CreateLines("axisZ", { points: [
      new BABYLON.Vector3(0, 0, 0), new BABYLON.Vector3(0, 0, size), new BABYLON.Vector3(0, -0.05 * size, size * 0.95),
      new BABYLON.Vector3(0, 0, size), new BABYLON.Vector3(0, 0.05 * size, size * 0.95)
  ]}, scene);
  axisZ.color = new BABYLON.Color3(0, 0, 1);
  axisZ.parent = worldAxis;

  var zChar = makeTextPlane("Z", "blue", size / 10);
  zChar.position = new BABYLON.Vector3(0, 0.05 * size, 0.9 * size);
  zChar.rotation =  new BABYLON.Vector3(-Math.PI/2, 0, 0);
  zChar.parent = worldAxis;

  worldAxis.position = new BABYLON.Vector3(0, 0, 0);
}
// Main function that gets executed once the webview DOM loads
export async function RenderMain() {
  const canvas = document.getElementById("renderCanvas") as HTMLCanvasElement; // Get the canvas element
  const engine = new BABYLON.Engine(canvas, true); // Generate the BABYLON 3D engine
  let scene = await createScene(engine, canvas);
  scene.debugLayer.show();

  createUI(scene);

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
function toggleWorldAxis() {
  if (worldAxis != undefined) {
    if (worldAxis.isEnabled()) {
      worldAxis.setEnabled(false);
    } else {
      worldAxis.setEnabled(true);
    }
  }
}

