import * as BABYLON from 'babylonjs';
import { Material } from './Material';
import * as Util from './util';
import {IGeometry} from './IGeometry';

export class Inertial {
  public name : string = "";
  public mass : number = 0;
  public origin : BABYLON.Vector3 = new BABYLON.Vector3(0, 0, 0);
  public ixx : number = 0;
  public ixy : number = 0;
  public ixz : number = 0;
  public iyy : number = 0;
  public iyz : number = 0;
  public izz : number = 0;

  public transform : BABYLON.TransformNode | undefined;
  
  public create(scene: BABYLON.Scene, materialMap : Map<string, Material>) : void {
    this.transform = new BABYLON.TransformNode(this.name, scene);
    this.transform.position = this.origin;
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
    this.transform?.dispose();
  }
}

