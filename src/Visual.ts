import * as BABYLON from 'babylonjs';
import { Material } from './Material';

import {IGeometry} from './IGeometry';

export class Visual {
    public name : string = "";

    public geometry : IGeometry | undefined = undefined;

    public material : Material | undefined = undefined;

    public origin : BABYLON.Vector3 = new BABYLON.Vector3(0, 0, 0);
    public rpy : BABYLON.Vector3 = new BABYLON.Vector3(0, 0, 0);
    public transform : BABYLON.TransformNode | undefined;

    public create(scene: BABYLON.Scene, materialMap : Map<string, Material>) : void {

        this.transform = new BABYLON.TransformNode("visual_" + this.name, scene);
        this.transform.position = this.origin;
        // Babylon.JS coordinate system to ROS transform
        this.transform.rotation = new BABYLON.Vector3(this.rpy.x+Math.PI/2, this.rpy.y, this.rpy.z);

        let mat = this.material;
        if (this.material != undefined) {
            if (this.material.isReference()) {
                mat = materialMap.get(this.material.name);
            } else {
                this.material.create(scene);
            }
        }

        if (this.geometry != undefined) {
            this.geometry.create(scene);

            if (mat?.material != undefined && this.geometry?.mesh != undefined) {
                this.geometry.mesh.material = mat.material;
            }

            if (this.transform  != undefined && this.geometry.transform != undefined) {
                this.geometry.transform.parent = this.transform;
            }


        }
    }

    public dispose() : void {
        this.geometry?.dispose();

        // References will be disposed by the robot
        if (this.material?.isReference()) {
            this.material?.dispose();
        }

        this.transform?.dispose();
    }
}