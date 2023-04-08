import * as BABYLON from 'babylonjs';
import 'babylonjs-loaders';
import { IGeometry } from "./IGeometry";
import { Material } from "./Material";
import path from 'path';
import { readFileSync } from 'fs';

export class Mesh implements IGeometry {
    public uri: string = "";
    public scale: number = 1.0;

    public mesh: BABYLON.AbstractMesh | undefined = undefined;

    constructor(uri: string, scale: number) {
        this.uri = uri;
    }
    
    public create(scene: BABYLON.Scene, mat: Material) : void {
        // TODO: Not sure why BabylonJS is so brain dead with this?
        if (this.uri.startsWith("file://"))
        {
            // Handle relative paths
            var filePath = this.uri.substring(7);
            if (!filePath.startsWith("/")) {
                filePath = path.join(__dirname, filePath);
            }
            var fileExtension = filePath.substring(filePath.lastIndexOf('.'));
            var meshdata = readFileSync(filePath).toString('base64');

            // Force the file to be read as base64 encoded data blob
            BABYLON.SceneLoader.ImportMesh("", "", "data:;base64," + meshdata, scene, (mesh) => {
                // Get a pointer to the mesh
                if (mesh) {
                    this.mesh = mesh[0];
                    this.mesh.material = mat.material as BABYLON.Material;
                }
            }, null, null, fileExtension);
        } else {
            // TODO: This requires XMLHttpRequest
            BABYLON.SceneLoader.ImportMesh("", this.uri, "", scene, (mesh) => {
                // Get a pointer to the mesh
                if (mesh) {
                    this.mesh = mesh[0];
                    this.mesh.material = mat.material as BABYLON.Material;
                }
            });
        }
    }
}