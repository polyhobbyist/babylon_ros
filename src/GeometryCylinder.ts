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
    
    public create(scene: BABYLON.Scene) : void {
        this.transform = new BABYLON.TransformNode("mesh_cylinder", scene);

        this.mesh = BABYLON.MeshBuilder.CreateCylinder("cylinder", 
            {
                diameter: this.radius * 2.0,
                height: this.length
            }, scene);

        this.mesh.parent = this.transform;
        this.mesh.addRotation(Math.PI / 2.0, 0, 0);
     }
    public dispose() : void {
        this.mesh?.dispose();
        this.transform?.dispose();
    }
}