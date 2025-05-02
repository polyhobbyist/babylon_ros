import * as BABYLON from 'babylonjs';
import * as Materials from 'babylonjs-materials';
import * as urdf from './urdf';
import {Robot} from './Robot';
import {Joint, JointType} from './Joint';
import {Link} from './Link';
import {Visual} from './Visual';
import { JointRotationGizmo } from './JointRotationGizmo';

import * as GUI from 'babylonjs-gui';
import * as ColladaFileLoader from '@polyhobbyist/babylon-collada-loader';

export class RobotScene {
  public engine : BABYLON.Engine | undefined = undefined;

  public scene : BABYLON.Scene | undefined = undefined;
  
  public currentURDF : string | undefined = undefined;
  public currentRobot : Robot | undefined = undefined;
  public UILayer : GUI.AdvancedDynamicTexture | undefined = undefined;
  
  public ground : BABYLON.GroundMesh | undefined = undefined;
  public camera : BABYLON.ArcRotateCamera | undefined = undefined;
  private statusLabel = new GUI.TextBlock();
  public readyToRender : Boolean = false;

  private jointAxisList : BABYLON.PositionGizmo[] = [];
  private linkAxisList : BABYLON.PositionGizmo[] = [];
  private jointRotationGizmos : BABYLON.RotationGizmo[] = [];
  private linkRotationGizmos : BABYLON.RotationGizmo[] = [];
  private worldAxis : BABYLON.TransformNode | undefined = undefined;
  private worldAxisSize = 8.0;
  private selectedVisual : Visual | undefined = undefined;
  private hoveredJoint : Joint | undefined = undefined;
  private utilLayer : BABYLON.UtilityLayerRenderer | undefined = undefined;
  private planeRotationGizmo: JointRotationGizmo | undefined = undefined;
  private jointGizmo: BABYLON.Gizmo | undefined = undefined;
      

  clearStatus() {
    this.statusLabel.text = "";
  }

  
  clearAxisGizmos() {
    this.linkAxisList.forEach((a) => {
      a.dispose();
    });
    this.linkAxisList = [];
  
    this.jointAxisList.forEach((a) => {
      a.dispose();
    });
    this.jointAxisList = [];
  }
  
  addAxisToTransform(list : BABYLON.PositionGizmo[], scene : BABYLON.Scene, layer: BABYLON.UtilityLayerRenderer, transform : BABYLON.TransformNode | undefined) {
    if (transform) {
      let axis = new BABYLON.PositionGizmo(layer);
      axis.scaleRatio = 0.5;
      axis.attachedNode = transform;
      list.push(axis);
  
      let drag = () => {
        if (transform) {
          this.statusLabel.text = transform.name + 
          "\nX: " + transform.position.x.toFixed(6) + 
          "\nY: " + transform.position.y.toFixed(6) + 
          "\nZ: " + transform.position.z.toFixed(6);
          this.statusLabel.linkOffsetY = -100;
        this.statusLabel.linkWithMesh(transform);
        }
      };
  
      axis.xGizmo.dragBehavior.onDragObservable.add(drag);
      axis.yGizmo.dragBehavior.onDragObservable.add(drag);
      axis.zGizmo.dragBehavior.onDragObservable.add(drag);
        
    }
  }
  
  toggleAxisOnRobot(jointOrLink : Boolean, scene : BABYLON.Scene | undefined, layer: BABYLON.UtilityLayerRenderer) {
    if (!this.currentRobot || !scene) {
      return;
    }
  
    let whichAxis = jointOrLink ? this.jointAxisList : this.linkAxisList;
  
    if (whichAxis.length === 0) {
      if (jointOrLink) {
        // Use Array.from to correctly iterate through the Map
        Array.from(this.currentRobot.joints.entries()).forEach(([name, j]) => {
          this.addAxisToTransform(whichAxis, scene, layer, j.transform);
        });
      } else {
        // Use Array.from to correctly iterate through the Map
        Array.from(this.currentRobot.links.entries()).forEach(([name, l]) => {
          l.visuals.forEach((v: Visual) => {
            this.addAxisToTransform(whichAxis, scene, layer, v.transform);
          });
        });
      }
    } else {
      this.clearAxisGizmos();
      this.clearStatus();
    }
  }

  clearRotationGizmos() {
    this.jointRotationGizmos.forEach((a) => {
      a.dispose();
    });
    this.jointRotationGizmos = [];
    this.linkRotationGizmos.forEach((a) => {
      a.dispose();
    });
    this.linkRotationGizmos = [];
  }
  
  addRotationToTransform(list : BABYLON.RotationGizmo[], scene : BABYLON.Scene | undefined, layer: BABYLON.UtilityLayerRenderer, transform : BABYLON.TransformNode | undefined) {
    if (!scene) {
      return;
    }

    if (transform) {
      let rotationGizmo = new BABYLON.RotationGizmo(layer);
      rotationGizmo.scaleRatio = 0.5;
      rotationGizmo.attachedNode = transform;
      list.push(rotationGizmo);
  
      let drag = () => {
        if (transform) {
          this.statusLabel.text = transform.name + 
          "\nR:" + transform.rotation.x.toFixed(6) + 
          "\nP:" + transform.rotation.y.toFixed(6) + 
          "\nY:" + transform.rotation.z.toFixed(6);
          this.statusLabel.linkOffsetY = -100;
          this.statusLabel.linkWithMesh(transform);
        }
      };
  
      rotationGizmo.xGizmo.dragBehavior.onDragObservable.add(drag);
      rotationGizmo.yGizmo.dragBehavior.onDragObservable.add(drag);
      rotationGizmo.zGizmo.dragBehavior.onDragObservable.add(drag);
    }
  
  }
  
  toggleAxisRotationOnRobot(jointOrLink : Boolean, ui: GUI.AdvancedDynamicTexture | undefined, scene : BABYLON.Scene | undefined, layer: BABYLON.UtilityLayerRenderer) {
    if (!this.currentRobot || !scene || !ui) {
      return;
    }
  
    let whichList = jointOrLink ? this.jointRotationGizmos : this.linkRotationGizmos;
    if (whichList.length === 0) {
      if (jointOrLink) {
        // Use Array.from to correctly iterate through the Map
        Array.from(this.currentRobot.joints.entries()).forEach(([name, j]) => {
          this.addRotationToTransform(whichList, scene, layer, j.transform);
        });
      } else {
        // Use Array.from to correctly iterate through the Map
        Array.from(this.currentRobot.links.entries()).forEach(([name, l]) => {
          l.visuals.forEach((v: Visual) => {
            this.addRotationToTransform(whichList, scene, layer, v.transform);
          });
        });
      }
    } else {
      this.clearRotationGizmos();
      this.clearStatus();
    }
  }
  
  clearJointExerciseGizmos() {
    this.planeRotationGizmo?.dispose();
    this.planeRotationGizmo = undefined;
    this.jointGizmo?.dispose();
    this.jointGizmo = undefined;
  }

  addExerciseGizmoToJoint(joint: Joint, scene: BABYLON.Scene, layer: BABYLON.UtilityLayerRenderer) {
    if (!joint.transform) {
      console.log(`No transform for joint: ${joint.name}`);
      return;
    }

    console.log(`Creating gizmo for joint: ${joint.name}, type: ${joint.type}`);

    // Only create gizmos for non-fixed joints
    if (joint.type === JointType.Fixed) {
      return;
    }
    
    switch (joint.type) {
      case JointType.Revolute:
      case JointType.Continuous:
        if (Math.abs(joint.axis.y) > 0.5) {
          console.log(`Joint ${joint.name} is primarily rotating around y-axis`);
          // Create a rotation gizmo for the XZ plane (rotating around Y axis)
          this.planeRotationGizmo = new JointRotationGizmo(
            new BABYLON.Vector3(0, 1, 0), // Y axis
            BABYLON.Color3.Green(),
            layer,
            joint.lowerLimit,
            joint.upperLimit
          );
        } else if (Math.abs(joint.axis.z) > 0.5) {
          console.log(`Joint ${joint.name} is primarily rotating around z-axis`);
          // Create a rotation gizmo for the XY plane (rotating around Z axis)
          this.planeRotationGizmo = new JointRotationGizmo(
            new BABYLON.Vector3(0, 0, 1), // Z axis
            BABYLON.Color3.Blue(),
            layer,
            joint.lowerLimit,
            joint.upperLimit
          );
        } else {
          console.log(`Joint ${joint.name} rotating around x-axis`);
          this.planeRotationGizmo = new JointRotationGizmo(
            new BABYLON.Vector3(1, 0, 0), // X axis
            BABYLON.Color3.Red(),
            layer,
            joint.lowerLimit,
            joint.upperLimit
          );
        }
        
        // Configure the rotation gizmo
        this.planeRotationGizmo.scaleRatio = 0.75; // Much larger for better visibility
        this.planeRotationGizmo.attachedNode = joint.transform;
        this.planeRotationGizmo.enableLimits = true;
        
        this.planeRotationGizmo.dragBehavior.onDragObservable.add(() => {
          if (joint.transform) {
            this.updateJointStatusLabel(joint);
          }
        });
        
        break;
        
      case JointType.Prismatic:
        // For prismatic joints, create a position gizmo limited to one axis
        const positionGizmo = new BABYLON.PositionGizmo(layer);
        positionGizmo.scaleRatio = .5;
        positionGizmo.attachedNode = joint.transform;
        
        // Enable only the axis that aligns with the joint's translation axis
        if (Math.abs(joint.axis.x) < 0.5) {
          positionGizmo.xGizmo.isEnabled = false;
        }

        if (Math.abs(joint.axis.y) < 0.5) {
          positionGizmo.yGizmo.isEnabled = false;
        }

        if (Math.abs(joint.axis.z) < 0.5) {
          positionGizmo.zGizmo.isEnabled = false;
        }
        
        this.jointGizmo = positionGizmo;
        break;
        
      case JointType.Planar:
      case JointType.Floating:
        // For planar and floating joints, simplified implementation
        const floatingGizmo = new BABYLON.PositionGizmo(layer);
        floatingGizmo.scaleRatio = .5;
        floatingGizmo.attachedNode = joint.transform;
        
        this.jointGizmo = floatingGizmo;
        break;
    }
  }
  
  updateJointStatusLabel(joint: Joint) {
    if (!joint.transform) return;
    
    let limitsText = "";

    let rotationText = "";
    if (joint.type === JointType.Revolute || joint.type === JointType.Continuous) {
      rotationText = "\nRotation: " + joint.transform.rotation.x.toFixed(3) + "," +
              joint.transform.rotation.y.toFixed(3) + "," +
              joint.transform.rotation.z.toFixed(3);
      if (!isNaN(joint.lowerLimit) && !isNaN(joint.upperLimit)) {
        limitsText = "\nLimits: " + joint.lowerLimit.toFixed(2) + " to " + joint.upperLimit.toFixed(2);
      }
    }

    let positionText = "";
    if (joint.type === JointType.Prismatic) {
      positionText = "\nPosition: " + joint.transform.position.x.toFixed(3) + "," +
              joint.transform.position.y.toFixed(3) + "," +
              joint.transform.position.z.toFixed(3);
    }


    this.statusLabel.text = joint.name + 
      "\nType: " + joint.type +
      limitsText +
      rotationText +
      positionText;
    this.statusLabel.linkOffsetY = -100;
    this.statusLabel.linkWithMesh(joint.transform);
  }
  
  toggleCollision() {
    if (this.currentRobot) {
      // Use Array.from to safely iterate through Map entries
      Array.from(this.currentRobot.links.entries()).forEach(([name, link]) => {
        link.collisions.forEach((c: Visual) => {
            c.setEnabled(!c.isEnabled());
        });
      });
    }
  }

  toggleVisuals() {
    if (this.currentRobot) {
      // Use Array.from to safely iterate through Map entries  
      Array.from(this.currentRobot.links.entries()).forEach(([name, link]) => {
        link.visuals.forEach((v: Visual) => {
            v.setEnabled(!v.isEnabled());
        });
      });
    }
  }

  toggleBoundingBoxes() {
    if (this.currentRobot) {
      // Use Array.from to safely iterate through Map entries
      Array.from(this.currentRobot.links.entries()).forEach(([name, link]) => {
        link.visuals.forEach((v: Visual) => {
            v.geometry?.meshes?.forEach((m: BABYLON.AbstractMesh) => {
              m.showBoundingBox = !m.showBoundingBox;
            });
        });
      });
    }
  }

  public resetCamera() {
    if (this.camera) {
      this.camera.alpha = - Math.PI / 3;
      this.camera.beta = 5 * Math.PI / 12;
      this.camera.target = new BABYLON.Vector3(0, 0, 0);
    }
  }
  
  createButton(toolbar: GUI.StackPanel, name : string, text : string, scene : BABYLON.Scene, onClick : () => void) {
    var button = GUI.Button.CreateSimpleButton(name, text);
    button.width = "100px";
    button.height = "20px";
    button.color = "white";
    button.cornerRadius = 5;
    button.background = "green";
    button.onPointerUpObservable.add(onClick);
    toolbar.addControl(button);
    return button;
  }


  makeTextPlane(text : string, color : string, size : number) {
    if (!this.scene) {
      return;
    }

    var dynamicTexture = new BABYLON.DynamicTexture("DynamicTexture", 50, this.scene, true);
    dynamicTexture.hasAlpha = true;
    dynamicTexture.drawText(text, 5, 40, "bold 36px Arial", color, "transparent", true);
    var plane = BABYLON.MeshBuilder.CreatePlane("TextPlane", {size: size}, this.scene);
    let material = new BABYLON.StandardMaterial("TextPlaneMaterial", this.scene);
    material.backFaceCulling = false;
    material.specularColor = new BABYLON.Color3(0, 0, 0);
    material.diffuseTexture = dynamicTexture;

    plane.material = material;
    return plane;
  };

  createWorldAxis() {
    if (!this.scene) {
      return;
    }

    this.worldAxis = new BABYLON.TransformNode("worldAxis", this.scene);

    // Babylon.JS coordinate system to ROS transform
    this.worldAxis.rotation =  new BABYLON.Vector3(-Math.PI/2, 0, 0);
  
    var axisX = BABYLON.MeshBuilder.CreateLines("axisX", {points: [
        new BABYLON.Vector3(0, 0, 0), new BABYLON.Vector3(this.worldAxisSize, 0, 0), new BABYLON.Vector3(this.worldAxisSize * 0.95, 0.05 * this.worldAxisSize, 0),
        new BABYLON.Vector3(this.worldAxisSize, 0, 0), new BABYLON.Vector3(this.worldAxisSize * 0.95, -0.05 * this.worldAxisSize, 0)
    ]}, this.scene);
    axisX.color = new BABYLON.Color3(1, 0, 0);
    axisX.parent = this.worldAxis;
  
    var xChar = this.makeTextPlane("X", "red", this.worldAxisSize / 10);
    if (xChar !== undefined) {
      xChar.position = new BABYLON.Vector3(0.9 * this.worldAxisSize, -0.05 * this.worldAxisSize, 0);
      xChar.parent = this.worldAxis;
    } 
  
    var axisY = BABYLON.MeshBuilder.CreateLines("axisY", {points: [
        new BABYLON.Vector3(0, 0, 0), new BABYLON.Vector3(0, this.worldAxisSize, 0), new BABYLON.Vector3(-0.05 * this.worldAxisSize, this.worldAxisSize * 0.95, 0),
        new BABYLON.Vector3(0, this.worldAxisSize, 0), new BABYLON.Vector3(0.05 * this.worldAxisSize, this.worldAxisSize * 0.95, 0)
    ]}, this.scene);
    axisY.color = new BABYLON.Color3(0, 1, 0);
    axisY.parent = this.worldAxis;
  
    var yChar = this.makeTextPlane("Y", "green", this.worldAxisSize / 10);
    if (yChar !== undefined) {
      yChar.position = new BABYLON.Vector3(-0.05 * this.worldAxisSize, 0.9 * this.worldAxisSize, 0);
      yChar.parent = this.worldAxis;
    }
      
    var axisZ = BABYLON.MeshBuilder.CreateLines("axisZ", { points: [
        new BABYLON.Vector3(0, 0, 0), new BABYLON.Vector3(0, 0, this.worldAxisSize), new BABYLON.Vector3(0, -0.05 * this.worldAxisSize, this.worldAxisSize * 0.95),
        new BABYLON.Vector3(0, 0, this.worldAxisSize), new BABYLON.Vector3(0, 0.05 * this.worldAxisSize, this.worldAxisSize * 0.95)
    ]}, this.scene);
    axisZ.color = new BABYLON.Color3(0, 0, 1);
    axisZ.parent = this.worldAxis;
  
    var zChar = this.makeTextPlane("Z", "blue", this.worldAxisSize / 10);
    if (zChar !== undefined) {
      zChar.position = new BABYLON.Vector3(0, 0.05 * this.worldAxisSize, 0.9 * this.worldAxisSize);
      zChar.rotation =  new BABYLON.Vector3(-Math.PI/2, 0, 0);
      zChar.parent = this.worldAxis;
    }
  
    this.worldAxis.position = new BABYLON.Vector3(0, 0, 0);    
  }

  toggleWorldAxis() {
    if (this.worldAxis != undefined) {
      if (this.worldAxis.isEnabled()) {
        this.worldAxis.setEnabled(false);
      } else {
        this.worldAxis.setEnabled(true);
      }
    }
  }
  
  
  createUI() {
    if (!this.scene) {
      return;
    }
    
    this.UILayer = GUI.AdvancedDynamicTexture.CreateFullscreenUI("UI", true, this.scene);
    
    this.statusLabel.color = "white";
    this.statusLabel.textHorizontalAlignment = GUI.Control.HORIZONTAL_ALIGNMENT_LEFT;
    this.statusLabel.horizontalAlignment = GUI.Control.HORIZONTAL_ALIGNMENT_CENTER;
    this.statusLabel.resizeToFit = true;
    this.statusLabel.outlineColor = "green";
    this.statusLabel.outlineWidth = 2.0;
    this.UILayer.addControl(this.statusLabel);
  
    var toolbar = new GUI.StackPanel();
    toolbar.paddingTop = "10px";
    toolbar.paddingLeft = "10px";
    toolbar.width = "100%";
    toolbar.height = "50px";
    toolbar.fontSize = "14px";
    toolbar.horizontalAlignment = GUI.Control.HORIZONTAL_ALIGNMENT_LEFT;
    toolbar.verticalAlignment = GUI.Control.VERTICAL_ALIGNMENT_TOP;
    toolbar.isVertical = false;
    this.UILayer.addControl(toolbar);
  
    // Create a utility layer with specific settings to ensure gizmo visibility
    this.utilLayer = new BABYLON.UtilityLayerRenderer(this.scene);
    this.utilLayer.utilityLayerScene.autoClearDepthAndStencil = false; // Helps with depth sorting
    this.utilLayer.shouldRender = true; // Ensure the layer renders
    this.utilLayer.onlyCheckPointerDownEvents = false; // Respond to all pointer events
  
    const gizmoManager = new BABYLON.GizmoManager(this.scene, 5, this.utilLayer);
    gizmoManager.usePointerToAttachGizmos = false;
    gizmoManager.positionGizmoEnabled = true;
    gizmoManager.rotationGizmoEnabled = true;
    
    this.createWorldAxis();
  
    this.createButton(toolbar, "jointAxisButton", "Joint Axis", this.scene, () => {
      this.toggleAxisOnRobot(true, this.scene, this.utilLayer!);
    });
  
    this.createButton(toolbar, "linkAxisButton", "Link Axis", this.scene, () => {
      this.toggleAxisOnRobot(false, this.scene, this.utilLayer!);
    });
  
    this.createButton(toolbar, "jointRotationButton", "Joint Rotation", this.scene, () => {  
      this.toggleAxisRotationOnRobot(true, this.UILayer, this.scene, this.utilLayer!);
    });
  
    this.createButton(toolbar, "linkRotationButton", "Link Rotation", this.scene, () => {  
      this.toggleAxisRotationOnRobot(false, this.UILayer, this.scene, this.utilLayer!);
    });

    this.createButton(toolbar, "worldAxis", "World Axis", this.scene, () => {  
      this.toggleWorldAxis();
    });

    this.createButton(toolbar, "collision", "Collision", this.scene, () => {  
      this.toggleCollision();
    });

    this.createButton(toolbar, "visuals", "Visuals", this.scene, () => {  
      this.toggleVisuals();
    });

    this.createButton(toolbar, "reset", "Reset", this.scene, () => {  
      if (this.currentURDF !== undefined) {
        this.applyURDF(this.currentURDF);
      }
    });

    let that = this;
    this.scene.onPointerDown = function castRay() {
      if (that.scene && that.camera) {
        if (that.selectedVisual) {
          that.selectedVisual.geometry?.meshes?.forEach((m: BABYLON.AbstractMesh) => {
            m.showBoundingBox = false;
          });
          that.selectedVisual = undefined;
        }

        that.clearStatus();

        var ray = that.scene.createPickingRay(that.scene.pointerX, that.scene.pointerY, BABYLON.Matrix.Identity(), that.camera, false);	

        var hit = that.scene.pickWithRay(ray);
        if (hit?.pickedMesh && that.currentRobot) {
          let foundJoint: Joint | undefined;
          
          // Convert Map entries to array and iterate through them safely
          Array.from(that.currentRobot.joints.entries()).forEach(([name, j]) => {
            // Skip fixed joints since they can't be exercised
            if (j.type === JointType.Fixed) return;
            
            if (j.transform) {
              // Check if the picked mesh is a child of this joint's transform
              if (hit?.pickedMesh?.parent === j.transform) {
                foundJoint = j;
                console.log(`Found joint (direct parent): ${j.name}`);
                return; // Exit the forEach early
              }
              
              // If we have a child link, check if the mesh belongs to any visual in that link
              if (j.child && !foundJoint) {
                j.child.visuals.forEach(visual => {
                  if (visual.geometry?.meshes) {
                    visual.geometry.meshes.forEach(mesh => {
                      if (mesh === hit?.pickedMesh) {
                        foundJoint = j;
                        console.log(`Found joint (child link visual): ${j.name}`);
                        return; // Exit inner forEach
                      }
                    });
                  }
                });
              }
            }
          });
          
          // Process the found joint if any
          if (foundJoint) {
            console.log(`Creating gizmo for joint: ${foundJoint.name}`);
            
            // If it's different from the currently selected joint, update the gizmo
            if (foundJoint !== that.hoveredJoint) {
              that.clearJointExerciseGizmos();
              that.hoveredJoint = foundJoint;
              
              // Add the gizmo to the joint using our custom layer
              that.addExerciseGizmoToJoint(foundJoint, that.scene!, that.utilLayer!);
              
              // Update status label with joint info
              if (foundJoint.transform) {
                that.updateJointStatusLabel(foundJoint);
              }
            }
          } else {
            that.hoveredJoint = undefined;
            that.clearJointExerciseGizmos();
            that.clearStatus();
          }

          // find the visual that has this mesh
          // This is messy for lots of meshes.
          // Maybe highlight the mesh tree?
          /*
          let found = false;
          that.currentRobot?.links.forEach((link: Link, name: string) => {
            link.visuals.forEach((v: Visual) => {
              v.geometry?.meshes?.forEach((m: BABYLON.AbstractMesh) => {
                if (hit?.pickedMesh && m === hit.pickedMesh) {
                  that.selectedVisual = v;
                  that.selectedVisual.geometry?.meshes?.forEach((m: BABYLON.AbstractMesh) => {
                    m.showBoundingBox = true;
                  });
                }
              });
            });
          });
          */
          
        }
      }
    }
  }
  
  public async applyURDF(urdfText: string, vscode: any | undefined = undefined) {
    this.clearAxisGizmos();
    this.clearRotationGizmos();
    this.clearJointExerciseGizmos();
    this.clearStatus();
    this.resetCamera();

    if (this.currentRobot) {
      var tempR = this.currentRobot;
      this.currentRobot = undefined;
      tempR.dispose();
    }

    if (vscode !== undefined) {
      vscode.postMessage({
        command: "trace",
        text: `loading urdf`,
      });
    }
  

    try {
      if (this.scene) {
        this.currentURDF = urdfText;
        this.currentRobot = await urdf.deserializeUrdfToRobot(urdfText);
        this.currentRobot.create(this.scene);
      }
    } catch (err: any) {

      if (vscode === undefined) {
        console.error(`Error loading urdf: ${err.message}`);
      } else {
          vscode.postMessage({
          command: "error",
          text: err.message,
        });
      }

      return;
    } 

    if (vscode !== undefined) {
        vscode.postMessage({
        command: "trace",
        text: `loaded urdf`,
      });
    }
  }

  public async createScene(canvas: HTMLCanvasElement) {
    let e: any = new BABYLON.Engine(canvas, true); // Generate the BABYLON 3D engine
    this.engine = e;

    this.scene = new BABYLON.Scene(e);
    if (BABYLON.SceneLoader) {
      //Add this loader into the register plugin
      BABYLON.SceneLoader.RegisterPlugin(new ColladaFileLoader.DAEFileLoader());
    }

    // This creates a basic Babylon Scene object (non-mesh)
      // Create a default ground and skybox.
    const environment = this.scene.createDefaultEnvironment({
      createGround: true,
      createSkybox: false,
      enableGroundMirror: true,
      groundMirrorSizeRatio: 0.15
    });

    this.scene.useRightHandedSystem = true;
    this.scene.clearColor = BABYLON.Color4.FromColor3(BABYLON.Color3.Black());

    
    // This creates and positions a free camera (non-mesh)
    this.camera = new BABYLON.ArcRotateCamera("camera1", - Math.PI / 3, 5 * Math.PI / 12, 1, new BABYLON.Vector3(0, 0, 0), this.scene);
    this.camera.wheelDeltaPercentage = 0.01;
    this.camera.minZ = 0.1;

    const light = new BABYLON.HemisphericLight("light", new BABYLON.Vector3(0, 1, 0), this.scene);
    const light2 = new BABYLON.HemisphericLight("light2", new BABYLON.Vector3(1, 0, 0), this.scene);
    const light3 = new BABYLON.HemisphericLight("light3", new BABYLON.Vector3(0, 0, 1), this.scene);

    // This attaches the camera to the canvas
    this.camera.attachControl(canvas, true);

    var groundMaterial = new Materials.GridMaterial("groundMaterial", this.scene);
    groundMaterial.majorUnitFrequency = 5;
    groundMaterial.minorUnitVisibility = 0.5;
    groundMaterial.gridRatio = 1;
    groundMaterial.opacity = 0.8;
    groundMaterial.useMaxLine = true;
    groundMaterial.lineColor = BABYLON.Color3.Green();
    groundMaterial.mainColor = BABYLON.Color3.Green();

    this.ground = BABYLON.MeshBuilder.CreateGround("ground", {width: 50, height: 50}, this.scene);
    this.ground.material = groundMaterial;
    this.ground.isPickable = false;
  }
}


