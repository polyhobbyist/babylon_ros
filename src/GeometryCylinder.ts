import * as BABYLON from 'babylonjs';
import { IGeometry } from "./IGeometry";

export class Cylinder implements IGeometry {
    public length : number = 0;
    public radius : number = 0;

    public mesh: BABYLON.Mesh | undefined = undefined;

    constructor(l : number, r: number) {
        this.length = l;
        this.radius = r;
    }
    
    public create(scene: BABYLON.Scene) : void {
        this.mesh = BABYLON.MeshBuilder.CreateCylinder("cylinder", 
            {
                diameter: this.radius * 2.0,
                height: this.length
            }, scene);

    }
}