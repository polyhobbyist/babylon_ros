import * as BABYLON from 'babylonjs';
import { Material } from './Material';
import { IGeometry } from "./IGeometry";

export class Box implements IGeometry {
    public width : number = 0;
    public height : number = 0;
    public depth : number = 0;


    public mesh: BABYLON.Mesh | undefined = undefined;

    constructor(w : number, h: number, d: number) {
        this.width = w;
        this.height = h;
        this.depth = d;
    }
    
    public create(scene: BABYLON.Scene, mat: Material) : void {
        this.mesh = BABYLON.MeshBuilder.CreateBox("box", 
            {
                width: this.width,
                height: this.height,
                depth: this.depth,
            }, scene);

        this.mesh.material = mat.material as BABYLON.Material;
    }
}