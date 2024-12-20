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
    public skeletons : BABYLON.Skeleton[] | undefined = undefined;

    constructor(uri: string, scale: BABYLON.Vector3) {
        this.uri = uri;
        this.scale = scale;
    }
    
    private meshCallback(scene: BABYLON.Scene, meshes : BABYLON.AbstractMesh[], particleSystems : BABYLON.IParticleSystem[] | undefined, skeletons : BABYLON.Skeleton[] | undefined) {
        // Get a pointer to the mesh
        if (meshes.length > 0 && this.transform != undefined) {
            this.meshes = meshes;
            this.meshes[0].parent = this.transform;

            // find the top level bone in skeletons
            if (skeletons != undefined && skeletons.length > 0) {
                this.skeletons = skeletons;

                let rootBone = skeletons[0].bones.find(b => b.getParent() == undefined);
                if (rootBone != undefined) {
                    rootBone.returnToRest();
                }
            } else {

                this.meshes.forEach(m => {
                    if (this.transform != undefined) {
                        m.addRotation(0, 0, Math.PI).addRotation(Math.PI/2, 0, 0);
                        // Invert the left handed mesh to conform to the right handed coodinate system
                        m.scaling = new BABYLON.Vector3(-1, 1, 1);
                        m.parent = this.transform;
                        
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
        this.transform.scaling = this.scale;

        this.material = mat ?? this.material;

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
        if (this.skeletons != undefined) {
            this.skeletons.forEach(s => {
                s.bones.forEach(b => {
                    b.getChildMeshes().forEach(m => {
                        m.dispose();
                    });
                });
                
                s.dispose();
            });
        }

        if (this.meshes != undefined) {
            this.meshes.forEach(m => {
                m.dispose();
            });
        }
        this.transform?.dispose();
    }
}