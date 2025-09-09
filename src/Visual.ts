/*
 * Copyright (c) 2025 Lou Amadio and Ranch Hand Robotics, LLC
 * All rights reserved.
 */

import * as BABYLON from 'babylonjs';
import { Material } from './Material';
import * as Util from './util';
import {IGeometry} from './IGeometry';

export class Visual {
    public name : string = "";

    public geometry : IGeometry | undefined = undefined;

    public material : Material | undefined = undefined;

    public origin : BABYLON.Vector3 = new BABYLON.Vector3(0, 0, 0);
    public rpy : BABYLON.Vector3 = new BABYLON.Vector3(0, 0, 0);
    public transform : BABYLON.TransformNode | undefined;

    public create(scene: BABYLON.Scene, materialMap : Map<string, Material>) : void {

        this.transform = new BABYLON.TransformNode(this.name, scene);
        this.transform.position = this.origin;
        Util.applyRotationToTransform(this.transform, this.rpy);

        let mat = this.material;
        if (this.material != undefined) {
            if (this.material.isReference()) {
                mat = materialMap.get(this.material.name);
            } else {
                this.material.create(scene);
            }
        }

        if (this.geometry != undefined) {
            this.geometry.create(scene, mat);

            if (this.transform  != undefined && this.geometry.transform != undefined) {
                this.geometry.transform.parent = this.transform;
            }
        }
    }

    public setEnabled(enabled: boolean) : void {
        if (this.transform != undefined) {
            this.transform.setEnabled(enabled);
        }
    }

    public isEnabled() : boolean {
        if (this.transform != undefined) {
            return this.transform.isEnabled();
        }
        return false;
    }

    public dispose() : void {
        this.geometry?.dispose();

        // References will be disposed by the robot
        if (this.material?.isReference() == false) {
            this.material?.dispose();
        }

        this.transform?.dispose();
    }
}