import * as BABYLON from 'babylonjs';
import {Link} from './Link';

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
    public rpy : BABYLON.Quaternion = new BABYLON.Quaternion(0, 0, 0, 1);
    public axis : BABYLON.Vector3 = new BABYLON.Vector3(1, 0, 0);

    public parentName : string = "";
    public childName : string = "";

    public parent : Link | undefined = undefined;
    public child : Link | undefined = undefined;
    

    public lowerLimit : number = 0;
    public upperLimit : number = 0;


    
}