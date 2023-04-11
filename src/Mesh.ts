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
    public transform : BABYLON.TransformNode | undefined;

    constructor(uri: string, scale: number) {
        this.uri = uri;
    }
    
    public create(scene: BABYLON.Scene, mat: Material) : void {
        this.transform = new BABYLON.TransformNode("mesh_mesh", scene);
        // TODO: Apply transform to match ROS coordinate system if needed

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
            let filename = this.uri.substring(this.uri.lastIndexOf('/') + 1);
            if (filename) {
                let base = this.uri.substring(0, this.uri.lastIndexOf('/') + 1);
                BABYLON.SceneLoader.ImportMesh(null, base, filename, scene, (mesh) => {
                    // Get a pointer to the mesh
                    if (mesh) {
                        this.mesh = mesh[0];
                        this.mesh.material = mat.material as BABYLON.Material;
                    }
                });
            }
        }

        if (this.mesh) {
            this.mesh.parent = this.transform;
        }
    }

    public dispose(): void {
        this.mesh?.dispose();
        this.transform?.dispose();
    }
}