import * as BABYLON from 'babylonjs';
import { Material } from './Material';
import { IGeometry } from "./IGeometry";

export class Cylinder implements IGeometry {
    public length : number = 0;
    public radius : number = 0;


    public mesh: BABYLON.AbstractMesh | undefined = undefined;
    public transform : BABYLON.TransformNode | undefined;

    constructor(l : number, r: number) {
        this.length = l;
        this.radius = r;
    }
    
    public create(scene: BABYLON.Scene, mat: Material) : void {
        this.transform = new BABYLON.TransformNode("mesh_cylinder", scene);
      // Babylon.JS cylinder has the flat sides on the XZ plane, where ROS is on the XY plane
      this.transform.rotation.x =  -Math.PI/2;

        this.mesh = BABYLON.MeshBuilder.CreateCylinder("cylinder", 
            {
                diameter: this.radius * 2.0,
                height: this.length
            }, scene);

        this.mesh.material = mat.material as BABYLON.Material;
        this.mesh.parent = this.transform;
    }
    public dispose() : void {
        this.mesh?.dispose();
        this.transform?.dispose();
    }
}