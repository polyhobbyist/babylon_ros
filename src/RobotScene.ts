import * as BABYLON from 'babylonjs';
import * as Materials from 'babylonjs-materials';
import * as urdf from './urdf';
import {Robot} from './Robot';
import {Joint, JointType} from './Joint';
import {Link} from './Link';
import {Visual} from './Visual';

import * as GUI from 'babylonjs-gui';
import * as GUI3D from 'babylonjs-gui';
import * as ColladaFileLoader from '@polyhobbyist/babylon-collada-loader';

export class RobotScene {
  public engine : BABYLON.Engine | undefined = undefined;
  public scene : BABYLON.Scene | undefined = undefined;
  public uiScene : BABYLON.Scene | undefined = undefined;
  public currentRobot : Robot | undefined = undefined;
  public UI3DManager : GUI3D.GUI3DManager | undefined = undefined;
  public acitonMenu : GUI3D.NearMenu | undefined = undefined;
  
  public ground : BABYLON.GroundMesh | undefined = undefined;
  public camera : BABYLON.ArcRotateCamera | undefined = undefined;
  public uiCamera : BABYLON.FreeCamera | undefined = undefined;
  private status3DText : GUI3D.TextBlock | undefined = undefined;
  private statusHologram : GUI3D.HolographicButton | undefined = undefined;
  public readyToRender : Boolean = false;

  private jointAxisList : BABYLON.PositionGizmo[] = [];
  private linkAxisList : BABYLON.PositionGizmo[] = [];
  private jointRotationGizmos : BABYLON.RotationGizmo[] = [];
  private linkRotationGizmos : BABYLON.RotationGizmo[] = [];
  private jointExerciseGizmos : BABYLON.Gizmo[] = [];
  private worldAxis : BABYLON.TransformNode | undefined = undefined;
  private worldAxisSize = 8.0;
  private selectedVisual : Visual | undefined = undefined;
  private hoveredJoint : Joint | undefined = undefined;
  private utilLayer : BABYLON.UtilityLayerRenderer | undefined = undefined;
      
  clearStatus() {
    if (this.status3DText) {
      this.status3DText.text = "";
    }
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
          if (this.status3DText) {
            this.status3DText.text = transform.name + 
            "\nX: " + transform.position.x.toFixed(6) + 
            "\nY: " + transform.position.y.toFixed(6) + 
            "\nZ: " + transform.position.z.toFixed(6);
            
            this.updateStatusPosition(transform);
          }
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
          let statusText = transform.name + 
          "\nR:" + transform.rotation.x.toFixed(6) + 
          "\nP:" + transform.rotation.y.toFixed(6) + 
          "\nY:" + transform.rotation.z.toFixed(6);
          
          if (this.status3DText) {
            this.status3DText.text = statusText;
            this.updateStatusPosition(transform);
          }
        }
      };
  
      rotationGizmo.xGizmo.dragBehavior.onDragObservable.add(drag);
      rotationGizmo.yGizmo.dragBehavior.onDragObservable.add(drag);
      rotationGizmo.zGizmo.dragBehavior.onDragObservable.add(drag);
    }
  }
  
  toggleAxisRotationOnRobot(jointOrLink : Boolean, scene : BABYLON.Scene | undefined, layer: BABYLON.UtilityLayerRenderer) {
    if (!this.currentRobot || !scene) {
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
    this.jointExerciseGizmos.forEach((g) => {
      g.dispose();
    });
    this.jointExerciseGizmos = [];
  }

  updateStatusPosition(transform: BABYLON.TransformNode) {
    if (this.statusHologram && transform) {
      // Position the status hologram near the transform
      const worldPos = transform.getAbsolutePosition();
      this.statusHologram.position = new BABYLON.Vector3(
        worldPos.x, 
        worldPos.y + 0.3, // Position slightly above the object
        worldPos.z
      );
    }
  }

  addExerciseGizmoToJoint(joint: Joint, scene: BABYLON.Scene, layer: BABYLON.UtilityLayerRenderer) {
    if (!joint.transform) {
      console.log(`No transform for joint: ${joint.name}`);
      return;
    }

    console.log(`Creating gizmo for joint: ${joint.name}, type: ${joint.type}`);

    let gizmo: BABYLON.Gizmo | undefined;

    // Only create gizmos for non-fixed joints
    if (joint.type === JointType.Fixed) {
      return;
    }
    
    switch (joint.type) {
      case JointType.Revolute:
      case JointType.Continuous:
        // For revolute and continuous joints, create a rotation gizmo that aligns with the joint axis
        const rotationGizmo = new BABYLON.RotationGizmo(layer);
        rotationGizmo.scaleRatio = .5; // Much larger for better visibility
        rotationGizmo.attachedNode = joint.transform;

        // Disable all axes except the one that aligns with the joint's rotation axis
        // This doesn't work - bug?
        //rotationGizmo.xGizmo.isEnabled = Math.abs(joint.axis.x) > 0.5;
        //rotatonGizmo.yGizmo.isEnabled = Math.abs(joint.axis.y) > 0.5;
        //rotationGizmo.zGizmo.isEnabled = Math.abs(joint.axis.z) > 0.5;
        let activeGizmo : BABYLON.IPlaneRotationGizmo | undefined;
        let axisKey = "";

        if (Math.abs(joint.axis.x) < 0.5) {
          rotationGizmo.xGizmo.isEnabled = false;
        } else {
          activeGizmo = rotationGizmo.xGizmo;
          console.log(`Joint ${joint.name} is primarily rotating around x-axis`);
          axisKey = "x";
        }

        if (Math.abs(joint.axis.y) < 0.5) {
          rotationGizmo.yGizmo.isEnabled = false;
        } else {
          activeGizmo = rotationGizmo.yGizmo;
          console.log(`Joint ${joint.name} is primarily rotating around y-axis`);
          axisKey = "y";
        }

        if (Math.abs(joint.axis.z) < 0.5) {
          rotationGizmo.zGizmo.isEnabled = false;
        } else {
          activeGizmo = rotationGizmo.zGizmo;
          console.log(`Joint ${joint.name} is primarily rotating around z-axis`);
          axisKey = "z";
        }

        // Make sure at least one axis is enabled if the joint axis values are too small
        if (!rotationGizmo.xGizmo.isEnabled && !rotationGizmo.yGizmo.isEnabled && !rotationGizmo.zGizmo.isEnabled) {
          console.log(`Joint ${joint.name} has no dominant axis, defaulting to x-axis`);
          rotationGizmo.xGizmo.isEnabled = true;
        }

        if (activeGizmo && activeGizmo.dragBehavior) {
          // Store initial rotation to track relative changes
          let startRotation = Number(joint.transform.rotation[axisKey as keyof BABYLON.Vector3]);
          let currentRotation = Number(startRotation);
          let lastRotation = Number(startRotation);

          activeGizmo.dragBehavior.onDragObservable.add(() => {
            if (!joint.transform) return;

            // Calculate how much rotation changed in this drag event
            const newRotation = joint.transform.rotation[axisKey as keyof BABYLON.Vector3];
            const delta = Number(newRotation) - Number(lastRotation);
            lastRotation = Number(newRotation);

            // Update the accumulated rotation
            currentRotation += delta;

            // For revolute joints, apply limits
            if (joint.type === JointType.Revolute) {
              const relativeToStart = currentRotation - startRotation;
              let correction : number = 0;
              
              // Clamp the rotation within the limits
              const clampedRotation = Math.max(joint.lowerLimit, Math.min(joint.upperLimit, relativeToStart));
              correction = clampedRotation - relativeToStart;

              const updatedRotation = new BABYLON.Vector3(
                axisKey === "x" ? joint.transform.rotation.x + correction : joint.transform.rotation.x,
                axisKey === "y" ? joint.transform.rotation.y + correction : joint.transform.rotation.y,
                axisKey === "z" ? joint.transform.rotation.z + correction : joint.transform.rotation.z
              );
              joint.transform.rotation = updatedRotation;
              currentRotation += correction;
            }

            this.updateJointStatusLabel(joint);
          });
        }
        
        gizmo = rotationGizmo;
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
        
        gizmo = positionGizmo;
        break;
        
      case JointType.Planar:
      case JointType.Floating:
        // For planar and floating joints, simplified implementation
        const floatingGizmo = new BABYLON.PositionGizmo(layer);
        floatingGizmo.scaleRatio = .5;
        floatingGizmo.attachedNode = joint.transform;
        
        gizmo = floatingGizmo;
        break;
    }
    
    if (gizmo) {
      console.log(`Added gizmo to joint: ${joint.name}`);
      this.jointExerciseGizmos.push(gizmo);
    } else {
      console.log(`Failed to create gizmo for joint: ${joint.name}`);
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

    let statusText = joint.name + 
      "\nType: " + joint.type +
      limitsText +
      rotationText +
      positionText;
    
    if (this.status3DText) {
      this.status3DText.text = statusText;
      this.updateStatusPosition(joint.transform);
    }
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
  
  create3DButton(name: string, text: string, onClick: () => void): GUI3D.TouchHolographicButton {
    const button = new GUI3D.TouchHolographicButton(name);
    button.onPointerUpObservable.add(onClick);
    
    const textBlock = new GUI3D.TextBlock();
    textBlock.text = text;
    textBlock.color = "white";
    textBlock.fontSize = 14;
    
    button.content = textBlock;
    
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
    if (!this.engine || !this.scene) {
      return;
    }

    this.uiScene = new BABYLON.Scene(this.engine);
    this.uiScene.autoClearDepthAndStencil = false; // Helps with depth sorting
    this.uiScene.autoClear = false; // Don't clear the scene, we want to overlay on top of it
    this.uiScene.clearColor = new BABYLON.Color4(0, 0, 0, 0);
    this.uiScene.useRightHandedSystem = false;

    this.uiCamera = new BABYLON.FreeCamera("overlayCamera", new BABYLON.Vector3(0, 0, 0), this.uiScene);
    this.uiCamera.fov = this.camera?.fov || 0.8; // Match field of view    
    
    // Create 3D UI manager
    this.UI3DManager = new GUI3D.GUI3DManager(this.uiScene);
    
    // Create status display for 3D UI
    this.statusHologram = new GUI3D.HolographicButton("statusHologram");
    this.statusHologram.isVisible = false;
    
    // Create status text for 3D UI
    this.status3DText = new GUI3D.TextBlock();
    this.status3DText.text = "";
    this.status3DText.color = "white";
    this.status3DText.fontSize = 14;
    
    this.statusHologram.content = this.status3DText;
    
    // Position status display
    this.statusHologram.position = new BABYLON.Vector3(0, 4, 0);
    
    // Create utility layer for gizmos
    this.utilLayer = new BABYLON.UtilityLayerRenderer(this.uiScene);
    this.utilLayer.utilityLayerScene.autoClearDepthAndStencil = false; // Helps with depth sorting
    this.utilLayer.shouldRender = true; // Ensure the layer renders
    this.utilLayer.onlyCheckPointerDownEvents = false; // Respond to all pointer events
  
    const gizmoManager = new BABYLON.GizmoManager(this.uiScene, 5, this.utilLayer);
    gizmoManager.usePointerToAttachGizmos = false;
    gizmoManager.positionGizmoEnabled = true;
    gizmoManager.rotationGizmoEnabled = true;
    
    this.acitonMenu = new GUI3D.NearMenu("near");
    let follower = this.acitonMenu.defaultBehavior.followBehavior;
    if (follower) {
      //follower.defaultDistance = 5;
      //follower.minimumDistance = 5;
      //follower.maximumDistance = 5;
    }
    //this.UI3DManager.addControl(this.statusHologram);
    this.UI3DManager.addControl(this.acitonMenu);

    // Create buttons for circular menu
    const buttonData = [
      { name: "jointAxisButton", text: "Joint Axis", action: () => this.toggleAxisOnRobot(true, this.uiScene, this.utilLayer!) },
      { name: "linkAxisButton", text: "Link Axis", action: () => this.toggleAxisOnRobot(false, this.uiScene, this.utilLayer!) },
      { name: "jointRotationButton", text: "Joint Rotation", action: () => this.toggleAxisRotationOnRobot(true, this.uiScene, this.utilLayer!) },
      { name: "linkRotationButton", text: "Link Rotation", action: () => this.toggleAxisRotationOnRobot(false, this.uiScene, this.utilLayer!) },
      { name: "worldAxisButton", text: "World Axis", action: () => this.toggleWorldAxis() },
      { name: "collisionButton", text: "Collision", action: () => this.toggleCollision() },
      { name: "visualsButton", text: "Visuals", action: () => this.toggleVisuals() },
      { name: "boundingBoxButton", text: "Bounding Box", action: () => this.toggleBoundingBoxes() }
    ];

    buttonData.forEach((bd) => {
      // Create 3D button
      const button = this.create3DButton(bd.name, bd.text, bd.action);
      this.acitonMenu?.addButton(button);
    });

    // this is how you make it vertical
    this.acitonMenu.rows = buttonData.length / 2;
    
    this.createWorldAxis();

    this.scene.onBeforeRenderObservable.add(() => {
      if (this.camera) {
        this.uiCamera?.position.copyFrom(this.camera.position);
        this.uiCamera?.rotation.copyFrom(this.camera.rotation);
      }
    });    
    
    // Set up pointer interaction for selecting joints
    let that = this;
    this.uiScene.onPointerDown = function castRay() {
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


