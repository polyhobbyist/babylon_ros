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
    private ext: string = "";
    private name: string = "mesh";

    constructor(uri: string, scale: BABYLON.Vector3) {
        this.uri = uri;
        this.scale = scale;
    }
    
    private meshCallback(scene: BABYLON.Scene, meshes : BABYLON.AbstractMesh[], particleSystems : BABYLON.IParticleSystem[] | undefined, skeletons : BABYLON.Skeleton[] | undefined, animationGroups: BABYLON.AnimationGroup[], transformNodes: BABYLON.TransformNode[], geometries: BABYLON.Geometry[], lights: BABYLON.Light[], spriteManagers: BABYLON.ISpriteManager[]) {
        // Get a pointer to the mesh
        if (meshes.length > 0 && this.transform != undefined) {
            this.meshes = meshes;
            if (this.transform != undefined) {
                const transformNode = transformNodes.find(tn => !tn.parent);
                if (transformNode) {
                    transformNode.parent = this.transform;
                }

                if (this.ext.toLowerCase().indexOf('.stl') !== -1) {
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
    }


    public create(scene: BABYLON.Scene, mat : Material | undefined) : void {
        this.material = mat ?? this.material;

        if (this.uri.startsWith("file://"))
        {
            // Handle relative paths
            var filePath = this.uri.substring(7); 
            if (!filePath.startsWith("/")) {
                filePath = path.join(__dirname, filePath);
            }
            this.ext = filePath.substring(filePath.lastIndexOf('.'));
            this.name = path.basename(filePath, this.ext);
            var meshdata = readFileSync(filePath).toString('base64');

            this.transform = new BABYLON.TransformNode(`mesh_${this.name}`, scene);

            // Force the file to be read as base64 encoded data blob
            BABYLON.SceneLoader.ImportMesh(null, "", "data:;base64," + meshdata, scene, (mesh, ps, sk, ag, tn, g, l, sm) => {this.meshCallback(scene, mesh, ps, sk, ag, tn, g, l,sm)}, null, null, this.ext);
        } else {
            let filename = this.uri.substring(this.uri.lastIndexOf('/') + 1);
            if (filename) {
                let base = this.uri.substring(0, this.uri.lastIndexOf('/') + 1);
                this.ext = this.uri.substring(this.uri.lastIndexOf('.'));
                this.name = filename.substring(0, filename.lastIndexOf('.'));
                this.transform = new BABYLON.TransformNode(`mesh_${this.name}`, scene);
                BABYLON.SceneLoader.ImportMesh(null, base, filename, scene, (mesh, ps, sk, ag, tn, g, l, sm) => {this.meshCallback(scene, mesh, ps, sk, ag, tn, g, l, sm)});
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