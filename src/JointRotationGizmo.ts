import * as BABYLON from 'babylonjs';

// Import BabylonJS types directly
import { Observable } from "babylonjs/Misc/observable";
import { Vector3, Matrix, Quaternion, TmpVectors, Vector2 } from "babylonjs/Maths/math.vector";
import { Color3 } from "babylonjs/Maths/math.color";
import { Mesh } from "babylonjs/Meshes/mesh";
import { AbstractMesh } from "babylonjs/Meshes/abstractMesh";
import { TransformNode } from "babylonjs/Meshes/transformNode";
import { Node } from "babylonjs/node";
import { PointerDragBehavior } from "babylonjs/Behaviors/Meshes/pointerDragBehavior";
import { Gizmo, GizmoAxisCache, IGizmo } from "babylonjs/Gizmos/gizmo";
import { UtilityLayerRenderer } from "babylonjs/Rendering/utilityLayerRenderer";
import { StandardMaterial } from "babylonjs/Materials/standardMaterial";
import { ShaderMaterial } from "babylonjs/Materials/shaderMaterial";
import { Effect } from "babylonjs/Materials/effect";
import { CreatePlane } from "babylonjs/Meshes/Builders/planeBuilder";
import { CreateTorus } from "babylonjs/Meshes/Builders/torusBuilder";
import { Epsilon } from "babylonjs/Maths/math.constants";
import { Logger } from "babylonjs/Misc/logger";
import { PointerInfo } from "babylonjs/Events/pointerEvents";
import { Observer } from "babylonjs/Misc/observable";
import { Nullable } from "babylonjs/types";

/**
 * Interface for joint rotation gizmo
 */
export interface IJointRotationGizmo extends IGizmo {
    /** Drag behavior responsible for the gizmos dragging interactions */
    dragBehavior: PointerDragBehavior;
    /** Drag distance in babylon units that the gizmo will snap to when dragged */
    snapDistance: number;
    /** Sensitivity factor for dragging */
    sensitivity: number;
    /** Lower rotation limit in radians */
    lowerLimit: number;
    /** Upper rotation limit in radians */
    upperLimit: number;
    /** Whether to enforce rotation limits */
    enableLimits: boolean;
    /**
     * Event that fires each time the gizmo snaps to a new location.
     * * snapDistance is the change in distance
     */
    onSnapObservable: Observable<{ snapDistance: number }>;
    /** Accumulated relative angle value for rotation on the axis. */
    angle: number;
    /** If the gizmo is enabled */
    isEnabled: boolean;

    /** Default material used to render when gizmo is not disabled or hovered */
    coloredMaterial: StandardMaterial;
    /** Material used to render when gizmo is hovered with mouse */
    hoverMaterial: StandardMaterial;
    /** Color used to render the drag angle sector when gizmo is rotated with mouse */
    rotationColor: Color3;
    /** Material used to render when gizmo is disabled. typically grey. */
    disableMaterial: StandardMaterial;
}

/**
 * Joint rotation gizmo
 */
export class JointRotationGizmo extends Gizmo implements IJointRotationGizmo {
    /**
     * Drag behavior responsible for the gizmos dragging interactions
     */
    public dragBehavior: PointerDragBehavior;
    protected _pointerObserver: Nullable<Observer<PointerInfo>> = null;

    /**
     * Rotation distance in radians that the gizmo will snap to (Default: 0)
     */
    public snapDistance = 0;
    /**
     * Event that fires each time the gizmo snaps to a new location.
     * * snapDistance is the change in distance
     */
    public onSnapObservable = new Observable<{ snapDistance: number }>();

    /**
     * The maximum angle between the camera and the rotation allowed for interaction
     * If a rotation plane appears 'flat', a lower value allows interaction.
     */
    public static MaxDragAngle: number = (Math.PI * 9) / 20;

    /**
     * Accumulated relative angle value for rotation on the axis. Reset to 0 when a dragStart occurs
     */
    public angle: number = 0;

    /**
     * Custom sensitivity value for the drag strength
     */
    public sensitivity = 1;
    
    /**
     * Lower rotation limit in radians
     */
    public lowerLimit: number = -Math.PI;
    
    /**
     * Upper rotation limit in radians
     */
    public upperLimit: number = Math.PI;
    
    /**
     * Whether to enforce rotation limits
     */
    public enableLimits: boolean = true;

    protected _isEnabled: boolean = true;
    protected _parent: Nullable<any> = null; // Using any instead of RotationGizmo as we don't need full type
    protected _coloredMaterial: StandardMaterial;
    protected _hoverMaterial: StandardMaterial;
    protected _disableMaterial: StandardMaterial;
    protected _gizmoMesh: Mesh;
    protected _rotationDisplayPlane: Mesh;
    protected _dragging: boolean = false;
    protected _angles = new Vector3();

    // Shader for rotation visualization
    protected static _RotationGizmoVertexShader = `
        precision highp float;
        attribute vec3 position;
        attribute vec2 uv;
        uniform mat4 worldViewProjection;
        varying vec3 vPosition;
        varying vec2 vUV;

        void main(void) {
            gl_Position = worldViewProjection * vec4(position, 1.0);
            vUV = uv;
        }`;

    // Updated fragment shader to support joint limits visualization
    protected static _RotationGizmoFragmentShader = `
        precision highp float;
        varying vec2 vUV;
        varying vec3 vPosition;
        uniform vec3 angles;
        uniform vec3 rotationColor;
        uniform vec2 limits;
        uniform bool enableLimits;

        #define twopi 6.283185307

        void main(void) {
            vec2 uv = vUV - vec2(0.5);
            float angle = atan(uv.y, uv.x) + 3.141592;
            float delta = gl_FrontFacing ? angles.y : -angles.y;
            float begin = angles.x - delta * angles.z;
            float len = sqrt(dot(uv,uv));
            float opacity = 1. - step(0.5, len);
            
            // Default behavior for full rotation
            float start = (begin < (begin + delta)) ? begin : (begin + delta);
            float end = (begin > (begin + delta)) ? begin : (begin + delta);
            
            // Apply joint limits if enabled
            if (enableLimits) {
                // Only show the arc between lower and upper limits
                start = limits.x;  // lowerLimit
                end = limits.y;    // upperLimit
            }

            float base = abs(floor(start / twopi)) * twopi;
            start += base;
            end += base;

            float intensity = 0.;
            for (int i = 0; i < 5; i++) {
                intensity += max(step(start, angle) - step(end, angle), 0.);
                angle += twopi;
            }

            gl_FragColor = vec4(rotationColor, min(intensity * 0.25, 0.8)) * opacity;
        }
    `;

    protected _rotationShaderMaterial: ShaderMaterial;

    /**
     * Creates a JointRotationGizmo
     * @param planeNormal The normal of the plane which the gizmo will be able to rotate on
     * @param color The color of the gizmo
     * @param gizmoLayer The utility layer the gizmo will be added to
     * @param tessellation Amount of tessellation to be used when creating rotation circles
     * @param parent Parent gizmo
     * @param useEulerRotation Use and update Euler angle instead of quaternion
     * @param thickness display gizmo axis thickness
     * @param hoverColor The color of the gizmo when hovering over and dragging
     * @param disableColor The Color of the gizmo when its disabled
     * @param lowerLimit Lower joint rotation limit in radians
     * @param upperLimit Upper joint rotation limit in radians
     */
    constructor(
        planeNormal: Vector3,
        color: Color3 = Color3.Gray(),
        gizmoLayer: UtilityLayerRenderer = UtilityLayerRenderer.DefaultUtilityLayer,
        tessellation = 32,
        parent: Nullable<any> = null,
        // eslint-disable-next-line @typescript-eslint/no-unused-vars
        useEulerRotation = false,
        thickness: number = 1,
        hoverColor: Color3 = Color3.Yellow(),
        disableColor: Color3 = Color3.Gray(),
        lowerLimit: number = -Math.PI,
        upperLimit: number = Math.PI
    ) {
        super(gizmoLayer);
        this._parent = parent;
        
        // Set rotation limits
        this.lowerLimit = lowerLimit;
        this.upperLimit = upperLimit;

        // Create Material
        this._coloredMaterial = new StandardMaterial("", gizmoLayer.utilityLayerScene);
        this._coloredMaterial.diffuseColor = color;
        this._coloredMaterial.specularColor = color.subtract(new Color3(0.1, 0.1, 0.1));

        this._hoverMaterial = new StandardMaterial("", gizmoLayer.utilityLayerScene);
        this._hoverMaterial.diffuseColor = hoverColor;
        this._hoverMaterial.specularColor = hoverColor;

        this._disableMaterial = new StandardMaterial("", gizmoLayer.utilityLayerScene);
        this._disableMaterial.diffuseColor = disableColor;
        this._disableMaterial.alpha = 0.4;

        // Build mesh on root node
        this._gizmoMesh = new Mesh("", gizmoLayer.utilityLayerScene);
        const { rotationMesh, collider } = this._createGizmoMesh(this._gizmoMesh, thickness, tessellation);

        // Setup Rotation Circle
        this._rotationDisplayPlane = CreatePlane(
            "rotationDisplay",
            {
                size: 0.6,
                updatable: false,
            },
            this.gizmoLayer.utilityLayerScene
        );
        this._rotationDisplayPlane.rotation.z = Math.PI * 0.5;
        this._rotationDisplayPlane.parent = this._gizmoMesh;
        this._rotationDisplayPlane.setEnabled(false);

        // Setup shader for visualization
        if (!Effect.ShadersStore["rotationGizmoVertexShader"]) {
            Effect.ShadersStore["rotationGizmoVertexShader"] = JointRotationGizmo._RotationGizmoVertexShader;
        }
        if (!Effect.ShadersStore["rotationGizmoFragmentShader"]) {
            Effect.ShadersStore["rotationGizmoFragmentShader"] = JointRotationGizmo._RotationGizmoFragmentShader;
        }
        
        this._rotationShaderMaterial = new ShaderMaterial(
            "shader",
            this.gizmoLayer.utilityLayerScene,
            {
                vertex: "rotationGizmo",
                fragment: "rotationGizmo",
            },
            {
                attributes: ["position", "uv"],
                uniforms: ["worldViewProjection", "angles", "rotationColor", "limits", "enableLimits"],
            }
        );
        
        this._rotationShaderMaterial.backFaceCulling = false;
        this.rotationColor = hoverColor;
        
        // Set initial limit values in the shader
        this._rotationShaderMaterial.setVector2("limits", new BABYLON.Vector2(this.lowerLimit, this.upperLimit));
        this._rotationShaderMaterial.setDefine("enableLimits", this.enableLimits);

        this._rotationDisplayPlane.material = this._rotationShaderMaterial;
        this._rotationDisplayPlane.visibility = 0.999;

        this._gizmoMesh.lookAt(this._rootMesh.position.add(planeNormal));
        this._rootMesh.addChild(this._gizmoMesh, Gizmo.PreserveScaling);
        this._gizmoMesh.scaling.scaleInPlace(1 / 3);
        
        // Add drag behavior to handle events when the gizmo is dragged
        this.dragBehavior = new PointerDragBehavior({ dragPlaneNormal: planeNormal });
        this.dragBehavior.moveAttached = false;
        this.dragBehavior.maxDragAngle = JointRotationGizmo.MaxDragAngle;
        this.dragBehavior._useAlternatePickedPointAboveMaxDragAngle = true;
        this._rootMesh.addBehavior(this.dragBehavior);

        // Closures for drag logic
        const lastDragPosition = new Vector3();

        const rotationMatrix = new Matrix();
        const planeNormalTowardsCamera = new Vector3();
        let localPlaneNormalTowardsCamera = new Vector3();

        this.dragBehavior.onDragStartObservable.add((e) => {
            if (this.attachedNode) {
                lastDragPosition.copyFrom(e.dragPlanePoint);
                this._rotationDisplayPlane.setEnabled(true);

                this._rotationDisplayPlane.getWorldMatrix().invertToRef(rotationMatrix);
                Vector3.TransformCoordinatesToRef(e.dragPlanePoint, rotationMatrix, lastDragPosition);

                this._angles.x = Math.atan2(lastDragPosition.y, lastDragPosition.x) + Math.PI;
                this._angles.y = 0;
                this._angles.z = this.updateGizmoRotationToMatchAttachedMesh ? 1 : 0;
                this._dragging = true;
                lastDragPosition.copyFrom(e.dragPlanePoint);
                this._rotationShaderMaterial.setVector3("angles", this._angles);
                this.angle = 0;
            }
        });

        this.dragBehavior.onDragEndObservable.add(() => {
            this._dragging = false;
            this._rotationDisplayPlane.setEnabled(false);
        });

        const tmpSnapEvent = { snapDistance: 0 };
        let currentSnapDragDistance = 0;
        const tmpMatrix = new Matrix();
        const amountToRotate = new Quaternion();
        
        this.dragBehavior.onDragObservable.add((event) => {
            if (this.attachedNode) {
                // Calc angle over full 360 degree
                const nodeScale = new Vector3(1, 1, 1);
                const nodeQuaternion = new Quaternion(0, 0, 0, 1);
                const nodeTranslation = new Vector3(0, 0, 0);
                const attachedNodeTransform = this._attachedNode as TransformNode;
                
                // Check there is an active pivot for the TransformNode attached
                if (attachedNodeTransform && attachedNodeTransform.isUsingPivotMatrix && attachedNodeTransform.isUsingPivotMatrix() && attachedNodeTransform.position) {
                    // When a TransformNode has an active pivot, its position might differ from world matrix translation
                    attachedNodeTransform.getWorldMatrix().setTranslation(attachedNodeTransform.position);
                }

                this.attachedNode.getWorldMatrix().decompose(nodeScale, nodeQuaternion, nodeTranslation);
                // Check for uniform scaling
                const uniformScaling = Math.abs(Math.abs(nodeScale.x) - Math.abs(nodeScale.y)) <= Epsilon && 
                                      Math.abs(Math.abs(nodeScale.x) - Math.abs(nodeScale.z)) <= Epsilon;
                
                if (!uniformScaling && this.updateGizmoRotationToMatchAttachedMesh) {
                    Logger.Warn(
                        "Unable to use a rotation gizmo matching mesh rotation with non uniform scaling. Use uniform scaling or set updateGizmoRotationToMatchAttachedMesh to false."
                    );
                    return;
                }
                
                nodeQuaternion.normalize();

                const nodeTranslationForOperation = this.updateGizmoPositionToMatchAttachedMesh ? nodeTranslation : this._rootMesh.absolutePosition;
                const newVector = event.dragPlanePoint.subtract(nodeTranslationForOperation).normalize();
                const originalVector = lastDragPosition.subtract(nodeTranslationForOperation).normalize();
                const cross = Vector3.Cross(newVector, originalVector);
                const dot = Vector3.Dot(newVector, originalVector);
                let angle = Math.atan2(cross.length(), dot) * this.sensitivity;
                
                planeNormalTowardsCamera.copyFrom(planeNormal);
                localPlaneNormalTowardsCamera.copyFrom(planeNormal);
                
                if (this.updateGizmoRotationToMatchAttachedMesh) {
                    nodeQuaternion.toRotationMatrix(rotationMatrix);
                    localPlaneNormalTowardsCamera = Vector3.TransformCoordinates(planeNormalTowardsCamera, rotationMatrix);
                }
                
                // Flip up vector depending on which side the camera is on
                let cameraFlipped = false;
                if (gizmoLayer.utilityLayerScene.activeCamera) {
                    const camVec = gizmoLayer.utilityLayerScene.activeCamera.position.subtract(nodeTranslationForOperation).normalize();
                    if (Vector3.Dot(camVec, localPlaneNormalTowardsCamera) > 0) {
                        planeNormalTowardsCamera.scaleInPlace(-1);
                        localPlaneNormalTowardsCamera.scaleInPlace(-1);
                        cameraFlipped = true;
                    }
                }
                
                const halfCircleSide = Vector3.Dot(localPlaneNormalTowardsCamera, cross) > 0.0;
                if (halfCircleSide) {
                    angle = -angle;
                }

                TmpVectors.Vector3[0].set(angle, 0, 0);
                if (!this.dragBehavior.validateDrag(TmpVectors.Vector3[0])) {
                    angle = 0;
                }

                // Snapping logic
                let snapped = false;
                if (this.snapDistance != 0) {
                    currentSnapDragDistance += angle;
                    if (Math.abs(currentSnapDragDistance) > this.snapDistance) {
                        let dragSteps = Math.floor(Math.abs(currentSnapDragDistance) / this.snapDistance);
                        if (currentSnapDragDistance < 0) {
                            dragSteps *= -1;
                        }
                        currentSnapDragDistance = currentSnapDragDistance % this.snapDistance;
                        angle = this.snapDistance * dragSteps;
                        snapped = true;
                    } else {
                        angle = 0;
                    }
                }
                
                // Calculate the new accumulated angle
                const newAngle = this.angle + (cameraFlipped ? -angle : angle);
                
                // Apply joint limits if enabled
                if (this.enableLimits) {
                    if (newAngle < this.lowerLimit) {
                        // Adjust angle to stay at lower limit
                        angle = this.lowerLimit - this.angle;
                    } else if (newAngle > this.upperLimit) {
                        // Adjust angle to stay at upper limit
                        angle = this.upperLimit - this.angle;
                    }
                }

                // Convert angle and axis to quaternion
                const quaternionCoefficient = Math.sin(angle / 2);
                amountToRotate.set(
                    planeNormalTowardsCamera.x * quaternionCoefficient,
                    planeNormalTowardsCamera.y * quaternionCoefficient,
                    planeNormalTowardsCamera.z * quaternionCoefficient,
                    Math.cos(angle / 2)
                );

                // If the meshes local scale is inverted, adjust rotation
                if (tmpMatrix.determinant() > 0) {
                    const tmpVector = new Vector3();
                    amountToRotate.toEulerAnglesToRef(tmpVector);
                    Quaternion.RotationYawPitchRollToRef(tmpVector.y, -tmpVector.x, -tmpVector.z, amountToRotate);
                }

                if (this.updateGizmoRotationToMatchAttachedMesh) {
                    // Rotate selected mesh quaternion over fixed axis
                    nodeQuaternion.multiplyToRef(amountToRotate, nodeQuaternion);
                    nodeQuaternion.normalize();
                    // recompose matrix
                    Matrix.ComposeToRef(nodeScale, nodeQuaternion, nodeTranslation, this.attachedNode.getWorldMatrix());
                } else {
                    // Rotate selected mesh quaternion over rotated axis
                    amountToRotate.toRotationMatrix(TmpVectors.Matrix[0]);
                    const translation = this.attachedNode.getWorldMatrix().getTranslation();
                    this.attachedNode.getWorldMatrix().multiplyToRef(TmpVectors.Matrix[0], this.attachedNode.getWorldMatrix());
                    this.attachedNode.getWorldMatrix().setTranslation(translation);
                }

                lastDragPosition.copyFrom(event.dragPlanePoint);
                if (snapped) {
                    tmpSnapEvent.snapDistance = angle;
                    this.onSnapObservable.notifyObservers(tmpSnapEvent);
                }
                
                this._angles.y += angle;
                this.angle += cameraFlipped ? -angle : angle;
                
                // Update shader with current angles and limits
                this._rotationShaderMaterial.setVector3("angles", this._angles);
                this._rotationShaderMaterial.setVector2("limits", new BABYLON.Vector2(this.lowerLimit, this.upperLimit));
                this._rotationShaderMaterial.setDefine("enableLimits", this.enableLimits);
                
                this._matrixChanged();
            }
        });

        const light = gizmoLayer._getSharedGizmoLight();
        light.includedOnlyMeshes = light.includedOnlyMeshes.concat(this._rootMesh.getChildMeshes(false));

        const cache: GizmoAxisCache = {
            colliderMeshes: [collider],
            gizmoMeshes: [rotationMesh],
            material: this._coloredMaterial,
            hoverMaterial: this._hoverMaterial,
            disableMaterial: this._disableMaterial,
            active: false,
            dragBehavior: this.dragBehavior,
        };
        
        if (this._parent && this._parent.addToAxisCache) {
            this._parent.addToAxisCache(this._gizmoMesh, cache);
        }

        this._pointerObserver = gizmoLayer.utilityLayerScene.onPointerObservable.add((pointerInfo) => {
            if (this._customMeshSet) {
                return;
            }
            // updating the maxangle for drag behavior
            this.dragBehavior.maxDragAngle = JointRotationGizmo.MaxDragAngle;
            this._isHovered = !!(cache.colliderMeshes.indexOf(<Mesh>pointerInfo?.pickInfo?.pickedMesh) != -1);
            
            if (!this._parent) {
                const material = cache.dragBehavior.enabled ? 
                    (this._isHovered || this._dragging ? this._hoverMaterial : this._coloredMaterial) : 
                    this._disableMaterial;
                this._setGizmoMeshMaterial(cache.gizmoMeshes, material);
            }
        });

        this.dragBehavior.onEnabledObservable.add((newState) => {
            this._setGizmoMeshMaterial(cache.gizmoMeshes, newState ? this._coloredMaterial : this._disableMaterial);
        });
    }

    /**
     * @internal
     * Create Geometry for Gizmo
     * @param parentMesh
     * @param thickness
     * @param tessellation
     * @returns
     */
    protected _createGizmoMesh(parentMesh: AbstractMesh, thickness: number, tessellation: number) {
        const collider = CreateTorus(
            "ignore",
            {
                diameter: 0.6,
                thickness: 0.03 * thickness,
                tessellation,
            },
            this.gizmoLayer.utilityLayerScene
        );
        collider.visibility = 0;
        const rotationMesh = CreateTorus(
            "",
            {
                diameter: 0.6,
                thickness: 0.005 * thickness,
                tessellation,
            },
            this.gizmoLayer.utilityLayerScene
        );
        rotationMesh.material = this._coloredMaterial;

        // Position arrow pointing in its drag axis
        rotationMesh.rotation.x = Math.PI / 2;
        collider.rotation.x = Math.PI / 2;

        parentMesh.addChild(rotationMesh, Gizmo.PreserveScaling);
        parentMesh.addChild(collider, Gizmo.PreserveScaling);
        return { rotationMesh, collider };
    }

    protected override _attachedNodeChanged(value: Nullable<Node>) {
        if (this.dragBehavior) {
            this.dragBehavior.enabled = value ? true : false;
        }
    }

    /**
     * If the gizmo is enabled
     */
    public set isEnabled(value: boolean) {
        this._isEnabled = value;
        if (!value) {
            this.attachedMesh = null;
        } else {
            if (this._parent) {
                this.attachedMesh = this._parent.attachedMesh;
            }
        }
    }

    public get isEnabled(): boolean {
        return this._isEnabled;
    }

    /**
     * Set the color used for the gizmo's rotation visualization
     */
    public set rotationColor(color: Color3) {
        this._rotationShaderMaterial.setColor3("rotationColor", color);
    }

    /** Default material used to render when gizmo is not disabled or hovered */
    public get coloredMaterial() {
        return this._coloredMaterial;
    }

    /** Material used to render when gizmo is hovered with mouse */
    public get hoverMaterial() {
        return this._hoverMaterial;
    }

    /** Material used to render when gizmo is disabled. typically grey. */
    public get disableMaterial() {
        return this._disableMaterial;
    }

    /**
     * Sets the gizmo's joint limits and updates the visualization
     * @param lowerLimit Lower rotation limit in radians
     * @param upperLimit Upper rotation limit in radians
     */
    public setLimits(lowerLimit: number, upperLimit: number): void {
        this.lowerLimit = lowerLimit;
        this.upperLimit = upperLimit;
        
        // Update shader with new limits
        if (this._rotationShaderMaterial) {
            this._rotationShaderMaterial.setVector2("limits", new BABYLON.Vector2(this.lowerLimit, this.upperLimit));
        }
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
        if (this._rotationDisplayPlane) {
            this._rotationDisplayPlane.dispose();
        }
        if (this._rotationShaderMaterial) {
            this._rotationShaderMaterial.dispose();
        }
        const materials = [this._coloredMaterial, this._hoverMaterial, this._disableMaterial];
        for (const matl of materials) {
            if (matl) {
                matl.dispose();
            }
        }
        super.dispose();
    }
}