import * as BABYLON from 'babylonjs';
import { Material } from './Material';
import { IGeometry } from "./IGeometry";

export class Sphere implements IGeometry {
    public length : number = 0;
    public radius : number = 0;


    public mesh: BABYLON.AbstractMesh | undefined = undefined;
    public transform : BABYLON.TransformNode | undefined;

    constructor(r: number) {
        this.radius = r;
    }
    
    public create(scene: BABYLON.Scene, mat : Material | undefined) : void {
        this.transform = new BABYLON.TransformNode("mesh_sphere", scene);

        this.mesh = BABYLON.MeshBuilder.CreateSphere("sphere", 
            {
                diameter: this.radius * 2.0,
            }, scene);

        this.mesh.parent = this.transform;
        if (mat != undefined && mat.material != undefined) {
            this.mesh.material = mat.material;
        }
    }

    public dispose() : void {
        this.mesh?.dispose();
        this.transform?.dispose();
    }
}