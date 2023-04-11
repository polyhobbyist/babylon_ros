import * as BABYLON from 'babylonjs';
import { Material } from './Material';
import { IGeometry } from "./IGeometry";

export class Box implements IGeometry {
    public width : number = 0;
    public height : number = 0;
    public depth : number = 0;


    public mesh: BABYLON.Mesh | undefined = undefined;
    public transform : BABYLON.TransformNode | undefined;

    constructor(x : number, y: number, z: number) {

        // BabylonJS maps w/h/d differently than ROS
        // d: z
        // h: y
        // w: x

        this.width = x;
        this.height = y;
        this.depth = z;
    }
    
    public create(scene: BABYLON.Scene, mat: Material) : void {
        this.transform = new BABYLON.TransformNode("mesh_box", scene);

        this.mesh = BABYLON.MeshBuilder.CreateBox("box", 
            {
                width: this.width,
                height: this.height,
                depth: this.depth,
            }, scene);

        this.mesh.material = mat.material as BABYLON.Material;
        this.mesh.parent = this.transform;
    }
    
    public dispose() : void {
        this.mesh?.dispose();
        this.transform?.dispose();
    }

}