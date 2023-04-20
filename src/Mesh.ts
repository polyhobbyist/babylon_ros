import * as BABYLON from 'babylonjs';
import 'babylonjs-loaders';
import { IGeometry } from "./IGeometry";
import { Material } from "./Material";
import path from 'path';
import { readFileSync } from 'fs';

export class Mesh implements IGeometry {
    public uri: string = "";
    public scale: BABYLON.Vector3 = new BABYLON.Vector3(1.0, 1.0, 1.0);

    public mesh: BABYLON.AbstractMesh | undefined = undefined;
    public transform : BABYLON.TransformNode | undefined = undefined;

    constructor(uri: string, scale: BABYLON.Vector3) {
        this.uri = uri;
        this.scale = scale;
    }
    
    private meshCallback(mesh : BABYLON.AbstractMesh[]) {
        // Get a pointer to the mesh
        if (mesh.length > 0) {
            this.mesh = mesh[0];
            if (this.mesh != null &&
                this.transform != undefined) {
                this.mesh.parent = this.transform;
                this.mesh.scaling = this.scale;
            }
        }
    }


    public create(scene: BABYLON.Scene) : void {
        this.transform = new BABYLON.TransformNode("mesh_mesh", scene);

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
            BABYLON.SceneLoader.ImportMesh(null, "", "data:;base64," + meshdata, scene, (mesh) => {this.meshCallback(mesh)}, null, null, fileExtension);
        } else {
            let filename = this.uri.substring(this.uri.lastIndexOf('/') + 1);
            if (filename) {
                let base = this.uri.substring(0, this.uri.lastIndexOf('/') + 1);
                BABYLON.SceneLoader.ImportMesh(null, base, filename, scene, (mesh) => {this.meshCallback(mesh)});
            }
        }
    }

    public dispose(): void {
        this.mesh?.dispose();
        this.transform?.dispose();
    }
}