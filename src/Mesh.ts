import * as BABYLON from 'babylonjs';
import 'babylonjs-loaders';
import { IGeometry } from "./IGeometry";
import { Material } from "./Material";

export class Mesh implements IGeometry {
    public uri: string = "";
    public scale: number = 1.0;

    public mesh: BABYLON.Mesh | undefined = undefined;

    constructor(uri: string, scale: number) {
        this.uri = uri;
    }
    
    public create(scene: BABYLON.Scene, mat: Material) : void {

        BABYLON.SceneLoader.ImportMesh("", "./", "xyz_cube.stl", scene, function (this:any, mesh) {
            // Get a pointer to the mesh
            this.mesh = mesh;
            this.mesh.material = mat.material as BABYLON.Material;
        });       
    }
}