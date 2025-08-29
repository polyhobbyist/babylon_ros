import * as BABYLON from 'babylonjs';
import * as Materials from 'babylonjs-materials';
import * as urdf from './urdf';
import {Robot} from './Robot';
import {Joint, JointType} from './Joint';
import {Link} from './Link';
import {Visual} from './Visual';
import { JointRotationGizmo } from './JointRotationGizmo';
import { JointPositionGizmo } from './JointPositionGizmo';

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
  private gizmoLayer : BABYLON.UtilityLayerRenderer | undefined = undefined;
  private planeRotationGizmo: JointRotationGizmo | undefined = undefined;
  private planePositionGizmo: JointPositionGizmo | undefined = undefined;
      

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
    this.planePositionGizmo?.dispose();
    this.planePositionGizmo = undefined;
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
      case JointType.Continuous:
      case JointType.Revolute:
        if (Math.abs(joint.axis.y) > 0.5) {
          console.log(`Joint ${joint.name} is primarily rotating around y-axis`);
          // Create a rotation gizmo for the XZ plane (rotating around Y axis)
          this.planeRotationGizmo = new JointRotationGizmo(
            joint,
            BABYLON.Color3.Green(),
            layer
          );
        } else if (Math.abs(joint.axis.z) > 0.5) {
          console.log(`Joint ${joint.name} is primarily rotating around z-axis`);
          // Create a rotation gizmo for the XY plane (rotating around Z axis)
          this.planeRotationGizmo = new JointRotationGizmo(
            joint,
            BABYLON.Color3.Blue(),
            layer
          );
        } else {
          console.log(`Joint ${joint.name} rotating around x-axis`);
          this.planeRotationGizmo = new JointRotationGizmo(
            joint,
            BABYLON.Color3.Red(),
            layer
          );
        }
        
        // Configure the rotation gizmo
        this.planeRotationGizmo.scaleRatio = 0.75; // Much larger for better visibility
        this.planeRotationGizmo.attachedNode = joint.transform;
        this.planeRotationGizmo.enableLimits = joint.type !== JointType.Continuous;
        
        this.planeRotationGizmo.dragBehavior.onDragObservable.add(() => {
          if (joint.transform) {
            this.updateJointStatusLabel(joint);
          }
        });
        
        break;
        
      case JointType.Prismatic:
        // For planar joints, create a position gizmo limited to two axes
        this.planePositionGizmo = undefined;
        if (Math.abs(joint.axis.y) > 0.5) {
          this.planePositionGizmo = new JointPositionGizmo(
            joint,
            BABYLON.Color3.Blue(),
            layer);
        } else if (Math.abs(joint.axis.z) > 0.5) {
          this.planePositionGizmo = new JointPositionGizmo(
            joint,
            BABYLON.Color3.Red(),
            layer);
        } else {
          this.planePositionGizmo = new JointPositionGizmo(
            joint,
            BABYLON.Color3.Green(),
            layer);
        }

        if (this.planePositionGizmo)
        {
          this.planePositionGizmo.scaleRatio = .75;
          this.planePositionGizmo.attachedNode = joint.transform;
        }
        break;

        case JointType.Planar:
          console.log(`Joint ${joint.name} is using a planar joint, which is not yet supported for exercise gizmos. If you would like to see this, please open an issue on the GitHub repository.`);
          break;

        case JointType.Floating:
          console.log(`Joint ${joint.name} is using a floating joint, which is not yet supported for exercise gizmos. If you would like to see this, please open an issue on the GitHub repository.`);
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
      if (!isNaN(joint.lowerLimit) && !isNaN(joint.upperLimit) && joint.lowerLimit !== joint.upperLimit && joint.type !== JointType.Continuous) {
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
    //this.utilLayer.utilityLayerScene.autoClearDepthAndStencil = false; // Helps with depth sorting
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

  /**
   * Takes a screenshot of the scene without UI elements and returns it as a base64 encoded PNG string
   * @param width Optional width for the screenshot. If not provided, uses current canvas width
   * @param height Optional height for the screenshot. If not provided, uses current canvas height
   * @returns Promise<string> Base64 encoded PNG data string
   */
  public async takeScreenshot(width?: number, height?: number): Promise<string> {
    if (!this.scene || !this.engine) {
      throw new Error("Scene or engine not initialized");
    }

    // Store current UI visibility states
    const uiWasVisible = (this.UILayer?.getChildren().length ?? 0) > 0;
    const utilLayerWasVisible = this.utilLayer?.shouldRender ?? false;
    const gizmoLayerWasVisible = this.gizmoLayer?.shouldRender ?? false;

    // Store UI controls for restoration
    const uiControls: GUI.Control[] = [];

    try {
      // Hide all UI elements temporarily by storing references and removing them
      if (this.UILayer) {
        // Store all UI controls and remove them temporarily
        const children = this.UILayer.getChildren();
        for (let i = children.length - 1; i >= 0; i--) {
          const control = children[i];
          uiControls.push(control);
          this.UILayer.removeControl(control);
        }
      }
      
      if (this.utilLayer) {
        this.utilLayer.shouldRender = false;
      }
      if (this.gizmoLayer) {
        this.gizmoLayer.shouldRender = false;
      }

      // Get canvas dimensions
      const canvas = this.engine.getRenderingCanvas();
      if (!canvas) {
        throw new Error("No rendering canvas found");
      }

      const targetWidth = width || canvas.width;
      const targetHeight = height || canvas.height;

      // Create a render target texture for capturing the scene
      const renderTarget = new BABYLON.RenderTargetTexture(
        "screenshot",
        { width: targetWidth, height: targetHeight },
        this.scene,
        false, // generateMipMaps
        true,  // doNotChangeAspectRatio
        BABYLON.Constants.TEXTURETYPE_UNSIGNED_INT,
        false, // isCube
        BABYLON.Texture.NEAREST_SAMPLINGMODE,
        true,  // generateDepthBuffer
        false, // generateStencilBuffer
        false, // isMulti
        BABYLON.Constants.TEXTUREFORMAT_RGBA
      );

      // Add all meshes to render list (excluding UI elements)
      if (this.scene.meshes) {
        renderTarget.renderList = this.scene.meshes.filter(mesh => {
          // Exclude UI-related meshes and gizmos
          return !mesh.name.includes("gizmo") && 
                 !mesh.name.includes("GUI") && 
                 !mesh.name.includes("ui") &&
                 mesh.isVisible;
        });
      }

      // Render the scene to the texture
      renderTarget.render();

      // Read the pixels from the render target
      const buffer = await renderTarget.readPixels();
      
      if (!buffer) {
        throw new Error("Failed to read pixels from render target");
      }

      // Create a canvas to convert the pixel data to PNG
      const tempCanvas = document.createElement('canvas');
      tempCanvas.width = targetWidth;
      tempCanvas.height = targetHeight;
      const ctx = tempCanvas.getContext('2d');
      
      if (!ctx) {
        throw new Error("Failed to get 2D context from canvas");
      }

      // Create ImageData from the buffer
      const imageData = ctx.createImageData(targetWidth, targetHeight);
      
      // Convert buffer to typed array for easier handling
      let pixelData: Uint8Array;
      if (buffer instanceof Uint8Array) {
        pixelData = buffer;
      } else if (buffer instanceof Float32Array) {
        // Convert Float32Array to Uint8Array
        pixelData = new Uint8Array(buffer.length);
        for (let i = 0; i < buffer.length; i++) {
          pixelData[i] = Math.floor(buffer[i] * 255);
        }
      } else {
        // Handle other ArrayBufferView types
        const view = new Uint8Array(buffer.buffer, buffer.byteOffset, buffer.byteLength);
        pixelData = view;
      }
      
      // Convert from RGBA to ImageData format with Y-flip
      for (let i = 0; i < pixelData.length; i += 4) {
        // Babylon.js uses RGBA, but we need to flip Y coordinate
        const pixelIndex = i / 4;
        const y = Math.floor(pixelIndex / targetWidth);
        const x = pixelIndex % targetWidth;
        const flippedY = targetHeight - 1 - y;
        const dstIndex = (flippedY * targetWidth + x) * 4;
        
        imageData.data[dstIndex] = pixelData[i];         // R
        imageData.data[dstIndex + 1] = pixelData[i + 1]; // G
        imageData.data[dstIndex + 2] = pixelData[i + 2]; // B
        imageData.data[dstIndex + 3] = pixelData[i + 3]; // A
      }

      // Put the image data on the canvas
      ctx.putImageData(imageData, 0, 0);

      // Convert to base64 PNG
      const base64Data = tempCanvas.toDataURL('image/png');
      
      // Clean up
      renderTarget.dispose();
      
      // Return just the base64 data part (without the data:image/png;base64, prefix)
      return base64Data.split(',')[1];

    } finally {
      // Restore UI visibility states
      if (this.UILayer && uiWasVisible) {
        // Re-add all UI controls
        for (const control of uiControls) {
          this.UILayer.addControl(control);
        }
      }
      if (this.utilLayer) {
        this.utilLayer.shouldRender = utilLayerWasVisible;
      }
      if (this.gizmoLayer) {
        this.gizmoLayer.shouldRender = gizmoLayerWasVisible;
      }
    }
  }

  /**
   * Takes a screenshot of the scene without UI elements and returns it as a data URL
   * @param width Optional width for the screenshot. If not provided, uses current canvas width
   * @param height Optional height for the screenshot. If not provided, uses current canvas height
   * @returns Promise<string> Data URL string (data:image/png;base64,...)
   */
  public async takeScreenshotDataURL(width?: number, height?: number): Promise<string> {
    const base64Data = await this.takeScreenshot(width, height);
    return `data:image/png;base64,${base64Data}`;
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


