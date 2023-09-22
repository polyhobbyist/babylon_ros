import * as BABYLON from 'babylonjs';
import 'babylonjs-loaders';
import { IGeometry } from "./IGeometry";
import { Material } from "./Material";
import path from 'path';
import { readFileSync } from 'fs';

export class Mesh implements IGeometry {
    public uri: string = "";
    public scale: BABYLON.Vector3 = new BABYLON.Vector3(1.0, 1.0, 1.0);

    public meshes: BABYLON.AbstractMesh[] | undefined = undefined;
    public transform : BABYLON.TransformNode | undefined = undefined;
    public material : Material | undefined = undefined;

    constructor(uri: string, scale: BABYLON.Vector3) {
        this.uri = uri;
        this.scale = scale;
    }
    
    private meshCallback(scene: BABYLON.Scene, meshes : BABYLON.AbstractMesh[], particleSystems : BABYLON.IParticleSystem[] | undefined, skeletons : BABYLON.Skeleton[] | undefined) {
        // Get a pointer to the mesh
        if (meshes.length > 0 && this.transform != undefined) {
            this.meshes = meshes;

            // find the top level bone in skeletons
            if (skeletons != undefined && skeletons.length > 0) {
                let rootBone = skeletons[0].bones.find(b => b.getParent() == undefined);
                if (rootBone != undefined) {
                    let t = rootBone.getTransformNode();
                    if (t != undefined) {
                        t.parent = this.transform;
                        //t.addRotation(0, 0, Math.PI).addRotation(Math.PI/2, 0, 0);
                    }
                }
            } else {

                this.meshes.forEach(m => {
                    if (this.transform != undefined) {
                        m.addRotation(0, 0, Math.PI).addRotation(Math.PI/2, 0, 0);
                        m.parent = this.transform;
                        m.scaling = this.scale;
                        if (this.material != undefined && this.material.material != undefined) {
                            m.material = this.material.material;
                        }
                    }
                });
            }
        }
    }


    public create(scene: BABYLON.Scene, mat : Material | undefined) : void {
        this.transform = new BABYLON.TransformNode("mesh_mesh", scene);
        this.material = mat;

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
            BABYLON.SceneLoader.ImportMesh(null, "", "data:;base64," + meshdata, scene, (mesh, ps, sk) => {this.meshCallback(scene, mesh, ps, sk)}, null, null, fileExtension);
        } else {
            let filename = this.uri.substring(this.uri.lastIndexOf('/') + 1);
            if (filename) {
                let base = this.uri.substring(0, this.uri.lastIndexOf('/') + 1);
                BABYLON.SceneLoader.ImportMesh(null, base, filename, scene, (mesh, ps, sk) => {this.meshCallback(scene, mesh, ps, sk)});
            }
        }
    }

    public dispose(): void {
        if (this.meshes != undefined) {
            this.meshes.forEach(m => {
                m.dispose();
            });
        }
        this.transform?.dispose();
    }
}