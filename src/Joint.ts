/*
 * Copyright (c) 2025 Lou Amadio and Ranch Hand Robotics, LLC
 * All rights reserved.
 */

import * as BABYLON from 'babylonjs';
import {Link} from './Link';
import { Material } from './Material';
import * as Util from './util';

export enum JointType {
    Fixed = "fixed",
    Revolute = "revolute",
    Continuous = "continuous",
    Prismatic = "prismatic",
    Floating = "floating",
    Planar = "planar"
};


export class Joint {

    public name : string = "";
    public type : JointType = JointType.Fixed;

    public origin : BABYLON.Vector3 = new BABYLON.Vector3(0, 0, 0);
    public rpy : BABYLON.Vector3 = new BABYLON.Vector3(0, 0, 0);
    public axis : BABYLON.Vector3 = new BABYLON.Vector3(1, 0, 0);
    public transform : BABYLON.TransformNode | undefined;

    public parentName : string = "";
    public childName : string = "";

    public parent : Link | undefined = undefined;
    public child : Link | undefined = undefined;
    

    public lowerLimit : number = 0;
    public upperLimit : number = 0;

    public create(scene: BABYLON.Scene, materialMap : Map<string, Material>) : void {

        this.transform = new BABYLON.TransformNode(this.name, scene);
        this.transform.position = this.origin;
        Util.applyRotationToTransform(this.transform, this.rpy);
    }

    public dispose() : void {
        this.transform?.dispose();
    }
    
}