/*
 * Copyright (c) 2025 Lou Amadio and Ranch Hand Robotics, LLC
 * All rights reserved.
 */

import * as BABYLON from 'babylonjs';

import { Joint } from './Joint';

export class JointPositionGizmo extends BABYLON.Gizmo {
    /**
     * Drag behavior responsible for the gizmos dragging interactions
     */
    public dragBehavior: BABYLON.PointerDragBehavior;
    protected _pointerObserver: BABYLON.Nullable<BABYLON.Observer<BABYLON.PointerInfo>> = null;
    /**
     * Drag distance in babylon units that the gizmo will snap to when dragged (Default: 0)
     */
    public snapDistance = 0;
    /**
     * Event that fires each time the gizmo snaps to a new location.
     * * snapDistance is the change in distance
     */
    public onSnapObservable = new BABYLON.Observable<{ snapDistance: number }>();

    protected _isEnabled: boolean = true;

    protected _gizmoMesh: BABYLON.Mesh;
    protected _coloredMaterial: BABYLON.StandardMaterial;
    protected _hoverMaterial: BABYLON.StandardMaterial;
    protected _disableMaterial: BABYLON.StandardMaterial;
    protected _dragging: boolean = false;


    public associatedJoint: Joint | undefined = undefined;

    /** Default material used to render when gizmo is not disabled or hovered */
    public get coloredMaterial() {
        return this._coloredMaterial;
    }

    /** Material used to render when gizmo is hovered with mouse*/
    public get hoverMaterial() {
        return this._hoverMaterial;
    }

    /** Material used to render when gizmo is disabled. typically grey.*/
    public get disableMaterial() {
        return this._disableMaterial;
    }    /**
     * @internal
     */
    public static _CreateArrow(scene: BABYLON.Scene, material: BABYLON.StandardMaterial, thickness: number = 1, isCollider = false): BABYLON.TransformNode {
        const arrow = new BABYLON.TransformNode("arrow", scene);
        const cylinder = BABYLON.CreateCylinder(
            "cylinder",
            {
                diameterTop: 0,
                height: 0.075,
                diameterBottom: 0.0375 * (1 + (thickness - 1) / 4),
                tessellation: 96,
            },
            scene
        );
        const line = BABYLON.CreateCylinder(
            "cylinder",
            {
                diameterTop: 0.005 * thickness,
                height: 0.275,
                diameterBottom: 0.005 * thickness,
                tessellation: 96,
            },
            scene
        );
        
        // Position arrow pointing in its drag axis
        cylinder.parent = arrow;
        cylinder.material = material;
        cylinder.rotation.x = Math.PI / 2;
        cylinder.position.z += 0.3;

        line.parent = arrow;
        line.material = material;
        line.position.z += 0.275 / 2;
        line.rotation.x = Math.PI / 2;

        if (isCollider) {
            line.visibility = 0;
            cylinder.visibility = 0;
        }
        return arrow;
    }
    
    /**
     * @internal
     */
    public static _CreateArrowInstance(scene: BABYLON.Scene, arrow: BABYLON.TransformNode): BABYLON.TransformNode {
        const instance = new BABYLON.TransformNode("arrow", scene);
        for (const mesh of arrow.getChildMeshes()) {
            const childInstance = (mesh as BABYLON.Mesh).createInstance(mesh.name);
            childInstance.parent = instance;
        }
        return instance;
    }

    /**
     * Creates an AxisDragGizmo
     * @param color The color of the gizmo
     * @param gizmoLayer The utility layer the gizmo will be added to
     * @param parent
     * @param thickness display gizmo axis thickness
     * @param hoverColor The color of the gizmo when hovering over and dragging
     * @param disableColor The Color of the gizmo when its disabled
     */
    constructor(
        associatedJoint: Joint | undefined,
        color: BABYLON.Color3 = BABYLON.Color3.Gray(),
        gizmoLayer: BABYLON.UtilityLayerRenderer = BABYLON.UtilityLayerRenderer.DefaultUtilityLayer,
        thickness: number = 1,
        hoverColor: BABYLON.Color3 = BABYLON.Color3.Yellow(),
        disableColor: BABYLON.Color3 = BABYLON.Color3.Gray()

    ) {
        super(gizmoLayer);
        this.associatedJoint = associatedJoint;

        // Create Material
        this._coloredMaterial = new BABYLON.StandardMaterial("", gizmoLayer.utilityLayerScene);
        this._coloredMaterial.diffuseColor = color;
        this._coloredMaterial.specularColor = color.subtract(new BABYLON.Color3(0.1, 0.1, 0.1));

        this._hoverMaterial = new BABYLON.StandardMaterial("", gizmoLayer.utilityLayerScene);
        this._hoverMaterial.diffuseColor = hoverColor;

        this._disableMaterial = new BABYLON.StandardMaterial("", gizmoLayer.utilityLayerScene);
        this._disableMaterial.diffuseColor = disableColor;
        this._disableMaterial.alpha = 0.4;

        // Build Mesh + Collider
        const arrow = JointPositionGizmo._CreateArrow(gizmoLayer.utilityLayerScene, this._coloredMaterial, thickness);
        const collider = JointPositionGizmo._CreateArrow(gizmoLayer.utilityLayerScene, this._coloredMaterial, thickness * 8, true);        // Add to Root Node
        this._gizmoMesh = new BABYLON.Mesh("", gizmoLayer.utilityLayerScene);
        this._gizmoMesh.addChild(arrow as BABYLON.Mesh);
        this._gizmoMesh.addChild(collider as BABYLON.Mesh);

        this._gizmoMesh.lookAt(this._rootMesh.position.add(this.associatedJoint?.axis || BABYLON.Vector3.Zero()));
        //this._gizmoMesh.scaling.scaleInPlace(1 / 3);
        this._gizmoMesh.parent = this._rootMesh;

        // Add drag behavior to handle events when the gizmo is dragged
        this.dragBehavior = new BABYLON.PointerDragBehavior({ dragAxis: this.associatedJoint?.axis || BABYLON.Vector3.Zero() });
        this.dragBehavior.moveAttached = false;
        this.dragBehavior.updateDragPlane = false;
        this._rootMesh.addBehavior(this.dragBehavior);        
          this.dragBehavior.onDragObservable.add((event) => {
            if (this.attachedNode) {
                // Keep world translation and use it to update world transform
                // if the node has parent, the local transform properties (position, rotation, scale)
                // will be recomputed in _matrixChanged function
                
                let matrixChanged: boolean = false;
                // Check if limits are enabled and enforce them
                if (this.associatedJoint && 
                    (this.associatedJoint.lowerLimit !== 0 || this.associatedJoint.upperLimit !== 0)) {
                    // Get the joint's origin as the reference point
                    const jointOrigin = this.associatedJoint.origin.clone();
                    
                    // Get current position of the attached node
                    const currentPos = BABYLON.TmpVectors.Vector3[3];
                    this.attachedNode.getWorldMatrix().getTranslationToRef(currentPos);
                    
                    // Calculate current displacement vector from joint origin to current position
                    const currentDisplacementVector = jointOrigin.subtract(currentPos);
                    
                    // Project the current displacement onto the joint axis to get scalar position along axis
                    const jointAxis = this.associatedJoint.axis.normalize();
                    const currentPosOnAxis = BABYLON.Vector3.Dot(currentDisplacementVector, jointAxis);
                      // Project the movement delta onto the joint axis
                    const deltaOnAxis = BABYLON.Vector3.Dot(event.delta, jointAxis);
                    
                    // Store the original delta for reference
                    const originalDelta = event.delta.clone();
                    
                    // Calculate new position along axis after applying delta
                    const newPosOnAxis = currentPosOnAxis + deltaOnAxis;
                    
                    console.log(`Current position along axis: ${currentPosOnAxis.toFixed(3)}`);
                    console.log(`Delta along axis: ${deltaOnAxis.toFixed(3)}`);
                    console.log(`New position along axis: ${newPosOnAxis.toFixed(3)}`);
                    console.log(`Limits: [${this.associatedJoint.lowerLimit.toFixed(3)}, ${this.associatedJoint.upperLimit.toFixed(3)}]`);
                    
                    if (newPosOnAxis < this.associatedJoint.lowerLimit) {
                        // If moving away from the lower limit (positive delta), allow the movement
                        if (deltaOnAxis > 0 && currentPosOnAxis <= this.associatedJoint.lowerLimit) {
                            console.log(`Allowing movement away from lower limit`);
                            // We allow the movement to proceed
                        } else {
                            // We're trying to move further beyond the lower limit
                            console.log(`Movement aborted: would exceed lower limit (${this.associatedJoint.lowerLimit.toFixed(3)})`);
                            event.delta.scaleInPlace(0); // Cancel the movement completely
                            return; // Exit early
                        }
                    } else if (newPosOnAxis > this.associatedJoint.upperLimit) {
                        // If moving away from the lower limit (positive delta), allow the movement
                        if (deltaOnAxis < 0 && currentPosOnAxis >= this.associatedJoint.upperLimit) {
                            console.log(`Allowing movement away from upper limit`);
                        } else {
                            console.log(`Movement aborted: would exceed upper limit (${this.associatedJoint.upperLimit.toFixed(3)})`);
                            event.delta.scaleInPlace(0); // Cancel the movement completely
                            return; // Exit early
                        }
                    } else {
                        // If we get here, the movement is within limits, so we allow it to proceed normally
                        console.log(`Movement within limits: ${newPosOnAxis.toFixed(3)} is between [${this.associatedJoint.lowerLimit.toFixed(3)}, ${this.associatedJoint.upperLimit.toFixed(3)}]`);
                    }
                }
                
                this.attachedNode.getWorldMatrix().getTranslationToRef(BABYLON.TmpVectors.Vector3[2]);
                BABYLON.TmpVectors.Vector3[2].addInPlace(event.delta);
                if (this.dragBehavior.validateDrag(BABYLON.TmpVectors.Vector3[2])) {
                    if ((this.attachedNode as any).position) {
                        // Required for nodes like lights
                        (this.attachedNode as any).position.addInPlaceFromFloats(event.delta.x, event.delta.y, event.delta.z);
                    }

                    // use _worldMatrix to not force a matrix update when calling GetWorldMatrix especially with Cameras
                    this.attachedNode.getWorldMatrix().addTranslationFromFloats(event.delta.x, event.delta.y, event.delta.z);
                    this.attachedNode.updateCache();
                    matrixChanged = true;
                }
                if (matrixChanged) {
                    this._matrixChanged();
                }
            }
        });
        this.dragBehavior.onDragStartObservable.add(() => {
            this._dragging = true;
        });
        this.dragBehavior.onDragEndObservable.add(() => {
            this._dragging = false;
        });

        const light = gizmoLayer._getSharedGizmoLight();
        light.includedOnlyMeshes = light.includedOnlyMeshes.concat(this._rootMesh.getChildMeshes(false));

        const cache: BABYLON.GizmoAxisCache = {
            gizmoMeshes: arrow.getChildMeshes(),
            colliderMeshes: collider.getChildMeshes(),
            material: this._coloredMaterial,
            hoverMaterial: this._hoverMaterial,
            disableMaterial: this._disableMaterial,
            active: false,
            dragBehavior: this.dragBehavior,
        };

        this._pointerObserver = gizmoLayer.utilityLayerScene.onPointerObservable.add((pointerInfo) => {
            if (this._customMeshSet) {
                return;
            }
            this._isHovered = !!(cache.colliderMeshes.indexOf(<BABYLON.Mesh>pointerInfo?.pickInfo?.pickedMesh) != -1);
            const material = this.dragBehavior.enabled ? (this._isHovered || this._dragging ? this._hoverMaterial : this._coloredMaterial) : this._disableMaterial;
            this._setGizmoMeshMaterial(cache.gizmoMeshes, material);
        });

        this.dragBehavior.onEnabledObservable.add((newState) => {
            this._setGizmoMeshMaterial(cache.gizmoMeshes, newState ? cache.material : cache.disableMaterial);
        });
    }

    protected override _attachedNodeChanged(value: BABYLON.Nullable<BABYLON.Node>) {
        if (this.dragBehavior) {
            this.dragBehavior.enabled = value ? true : false;
        }
    }    /**
     * If the gizmo is enabled
     */
    public set isEnabled(value: boolean) {
        this._isEnabled = value;
        if (!value) {
            this.attachedMesh = null;
            this.attachedNode = null;
        }
    }

    public get isEnabled(): boolean {
        return this._isEnabled;
    }

    /**
     * Disposes of the gizmo
     */
    public override dispose() {
        this.onSnapObservable.clear();
        this.gizmoLayer.utilityLayerScene.onPointerObservable.remove(this._pointerObserver);
        this.dragBehavior.detach();
        if (this._gizmoMesh) {
            this._gizmoMesh.dispose();
        }
        const mats = [this._coloredMaterial, this._hoverMaterial, this._disableMaterial];
        for (const matl of mats) {
            if (matl) {
                matl.dispose();
            }
        }
        super.dispose();
    }
}