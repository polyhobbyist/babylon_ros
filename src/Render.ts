import * as BABYLON from 'babylonjs';
import * as Materials from 'babylonjs-materials';
import * as urdf from './urdf';
import {Robot} from './Robot';
import * as GUI from 'babylonjs-gui';

function applyAxisToTransform(scene : BABYLON.Scene, t : BABYLON.TransformNode | undefined) : BABYLON.AxesViewer | undefined {
  if (t) {
    let a = new BABYLON.AxesViewer(scene, .1);
    a.xAxis.parent = t;
    a.yAxis.parent = t;
    a.zAxis.parent = t;

    return a;
  }

  return undefined;
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

var statusLabel = new GUI.TextBlock();

let axisList : BABYLON.PositionGizmo[] = [];

function toggleAxisOnRobot(scene : BABYLON.Scene, layer: BABYLON.UtilityLayerRenderer,robot : Robot) {

  if (axisList.length == 0) {
    robot.joints.forEach((j) => {
        if (j.transform) {
          let axis = new BABYLON.PositionGizmo(layer);
          axis.scaleRatio = 0.5
          axis.attachedNode = j.transform;
          axisList.push(axis);

          let drag = () => {
            if (j.transform) {
            statusLabel.text = j.transform.name + "\nX: " + j.transform.position.x + "\nY: " + j.transform.position.y + "\nZ: " + j.transform.position.z;
            statusLabel.linkOffsetY = -100;
            statusLabel.linkWithMesh(j.transform);
            }
          }
  
          axis.xGizmo.dragBehavior.onDragObservable.add(drag);
          axis.yGizmo.dragBehavior.onDragObservable.add(drag);
          axis.zGizmo.dragBehavior.onDragObservable.add(drag);
            
        }
    });
  } else {
    axisList.forEach((a) => {
      a.dispose();
    });
    axisList = [];
  }
}

let rotationGizmos : BABYLON.RotationGizmo[] = [];

function toggleAxisRotationOnRobot(ui: GUI.AdvancedDynamicTexture, scene : BABYLON.Scene, layer: BABYLON.UtilityLayerRenderer, robot : Robot) {
  if (rotationGizmos.length == 0) {
    robot.joints.forEach((j) => {
      if (j.transform) {
        let rotationGizmo = new BABYLON.RotationGizmo(layer);
        rotationGizmo.scaleRatio = 0.5
        rotationGizmo.attachedNode = j.transform;
        rotationGizmos.push(rotationGizmo);

        let drag = () => {
          if (j.transform) {
          statusLabel.text = j.transform.name + "\nR:" + j.transform.rotation.x + "\nP:" + j.transform.rotation.y + "\nY:" + j.transform.rotation.z;
          statusLabel.linkOffsetY = -100;
          statusLabel.linkWithMesh(j.transform);
          }
        }

        rotationGizmo.xGizmo.dragBehavior.onDragObservable.add(drag);
        rotationGizmo.yGizmo.dragBehavior.onDragObservable.add(drag);
        rotationGizmo.zGizmo.dragBehavior.onDragObservable.add(drag);
      }
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

  scene.useRightHandedSystem = true;
  scene.clearColor = BABYLON.Color4.FromColor3(BABYLON.Color3.Black());// TODO (polyhobbyist) Make this configurable

  var radius = 1; // TODO (polyhobbyist): make this configurable

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
async function RenderMain() {

  let u = /*xml*/ 
  `<?xml version="1.0"?>
  <robot name="bb_head">
  
    <material name="red">
      <color rgba=".8 0 0 1"/>
    </material>
    
    <material name="purple">
      <color rgba=".8 0 .8 1"/>
    </material>
  
    <link name="base_link">
      <visual>
        <geometry>
          <mesh filename="https://raw.githubusercontent.com/polyhobbyist/bb_description/main/meshes/dome.stl" scale="0.001 0.001 0.001" />
        </geometry>
        <material name="purple"/>
      </visual>
    </link>
  
  
    <link name="camera_link">
      <visual>
        <geometry>
          <box size="0.01 0.01 0.001"/>
        </geometry>
      </visual>
    </link>
  
    <joint name="dome_to_camera_joint" type="fixed">
      <parent link="base_link"/>
      <child link="camera_link"/>
      <origin xyz="0 0.12 0.08" rpy="2.093 0 0"/>
    </joint>
  
    <link name="front_depth_link">
      <visual>
        <geometry>
          <box size="0.01 0.01 0.001"/>
        </geometry>
        <material name="red"/>
      </visual>
    </link>
    
    <joint name="dome_to_front_depth_joint" type="fixed">
      <parent link="base_link"/>
      <child link="front_depth_link"/>
      <origin xyz="0 0.11 0.09" rpy="2.093 0 0"/>
    </joint>
  
    <link name="left_depth_link">
      <visual>
        <geometry>
          <box size="0.01 0.01 0.001"/>
        </geometry>
        <material name="red"/>
      </visual>
    </link>
    
    <joint name="dome_to_left_depth_joint" type="fixed">
      <parent link="base_link"/>
      <child link="left_depth_link"/>
      <origin xyz="0.11693 -0.067 0.05" rpy="1.047198 0.959931 0.436332"/>
    </joint>
  
    <link name="right_depth_link">
      <visual>
        <geometry>
          <box size="0.01 0.01 0.001"/>
        </geometry>
        <material name="red"/>
      </visual>
    </link>
    
    <joint name="dome_to_right_depth_joint" type="fixed">
      <parent link="base_link"/>
      <child link="right_depth_link"/>
      <origin xyz="-0.11693 -0.067 0.05" rpy="1.047198 -0.959931 -0.436332"/>
    </joint>
  
  
    <link name="depth_link">
      <visual>
        <geometry>
          <sphere radius="0.001"/>
        </geometry>
      </visual>
    </link>
  
    <joint name="dome_to_depth_joint" type="fixed">
      <parent link="base_link"/>
      <child link="depth_link"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </joint>
  
  </robot>
  `;
  
  const canvas = document.getElementById("renderCanvas") as HTMLCanvasElement; // Get the canvas element
  const engine = new BABYLON.Engine(canvas, true); // Generate the BABYLON 3D engine
  let scene = await createScene(engine, canvas);
  scene.debugLayer.show();

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

  // Just like a regular webpage we need to wait for the webview
  // DOM to load before we can reference any of the HTML elements
  // or toolkit components
  window.addEventListener("load", RenderMain);
  
