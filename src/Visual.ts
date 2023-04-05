import * as BABYLON from 'babylonjs';
import {IGeometry} from './IGeometry';

export class Visual {
    public name : string = "";

    public geometry : IGeometry | undefined = undefined;

    public origin : BABYLON.Vector3 = new BABYLON.Vector3(0, 0, 0);
    public rpy : BABYLON.Quaternion = new BABYLON.Quaternion(0, 0, 0, 1);
}