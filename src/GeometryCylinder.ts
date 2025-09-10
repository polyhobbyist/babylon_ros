/*
 * Copyright (c) 2025 Lou Amadio and Ranch Hand Robotics, LLC
 * All rights reserved.
 */

import * as BABYLON from 'babylonjs';
import { Material } from './Material';
import { IGeometry } from "./IGeometry";

export class Cylinder implements IGeometry {
    public length : number = 0;
    public radius : number = 0;


    public meshes: BABYLON.AbstractMesh[] = [];
    public transform : BABYLON.TransformNode | undefined;

    constructor(l : number, r: number) {
        this.length = l;
        this.radius = r;
    }
    
    public create(scene: BABYLON.Scene, mat : Material | undefined) : void {
        this.transform = new BABYLON.TransformNode("mesh_cylinder", scene);

        this.meshes.push(BABYLON.MeshBuilder.CreateCylinder("cylinder", 
            {
                diameter: this.radius * 2.0,
                height: this.length
            }, scene));

        this.meshes[0].parent = this.transform;
        this.meshes[0].addRotation(Math.PI / 2.0, 0, 0);
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