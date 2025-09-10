/*
 * Copyright (c) 2025 Lou Amadio and Ranch Hand Robotics, LLC
 * All rights reserved.
 */

import * as BABYLON from 'babylonjs';
import { Material } from './Material';
import { IGeometry } from "./IGeometry";

export class Box implements IGeometry {
    public width : number = 0;
    public height : number = 0;
    public depth : number = 0;


    public meshes: BABYLON.Mesh[] = [];
    public transform : BABYLON.TransformNode | undefined;

    constructor(x : number, y: number, z: number) {

        // BabylonJS maps w/h/d differently than ROS
        // d: z
        // h: y
        // w: x

        this.width = x;
        this.height = y;
        this.depth = z;
    }
    
    public create(scene: BABYLON.Scene, mat : Material | undefined) : void {
        this.transform = new BABYLON.TransformNode("mesh_box", scene);

        this.meshes.push(BABYLON.MeshBuilder.CreateBox("box", 
            {
                width: this.width,
                height: this.height,
                depth: this.depth,
            }, scene));

        this.meshes[0].parent = this.transform;
        if (mat != undefined && mat.material != undefined) {
            this.meshes[0].material = mat.material;
        }
    }
    
    public dispose() : void {
        if (this.meshes != undefined) {
            this.meshes.forEach(m => {
                m.dispose();
            });
        }
        this.transform?.dispose();
    }

}