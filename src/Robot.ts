import * as BABYLON from 'babylonjs';
import { Material } from './Material';

import {Link} from './Link';
import {Joint} from './Joint';

export class Robot {
    public name : string = "";

    public transform : BABYLON.TransformNode | undefined;

    public links : Map<string, Link> = new Map<string, Link>();
    public joints : Map<string, Joint> = new Map<string, Joint>();
    public materials : Map<string, Material> = new Map<string, Material>();

    constructor() {
        this.materials.set("default", new Material());
    }
    
    create(scene: BABYLON.Scene) {
      this.transform = new BABYLON.TransformNode(this.name, scene);

      // Babylon.JS coordinate system to ROS transform
      this.transform.rotation =  new BABYLON.Vector3(-Math.PI/2, 0, 0);

      for (let [name, mat] of this.materials) {
        mat.create(scene);
      }

      for (let [name, link] of this.links) {
        link.create(scene, this.materials);
      }

      // first, check for base_footprint, then base_link
      let base = this.links.get("base_footprint");
      if (base == undefined) {
        base = this.links.get("base_link");
      }

      if (base == undefined) {
        throw new Error("No base_link or base_footprint defined in this robot");
      } else if (base.transform) {
        base.transform.parent = this.transform;
      }

      for (let [name, joint] of this.joints) {
        joint.create(scene, this.materials);
      }

      // Fixup transform tree
      for (let [name, joint] of this.joints) {
        // A Joint connects two links - each has a transform
        // We want this joint to be conncted to the "parent" link between the two links
        if (joint.transform != undefined) {
          if (joint.parent != undefined&& 
              joint.parent.transform != undefined) {
              joint.transform.parent = joint.parent.transform;
          } else {
            // TODO: Is this a bug?
          }

          // We also want the child link to point to this joints' transform.
          if (joint.child  != undefined && 
              joint.child.transform != undefined) {
              joint.child.transform.parent = joint.transform;
          } else {
            // TODO: Is this a bug?
          }
        }
      }
    }

    public dispose() : void {
      this.transform?.dispose();

      for (let [name, mat] of this.materials) {
        mat.material?.dispose();
      }

      for (let [name, link] of this.links) {
        link.dispose();
      }

      for (let [name, joint] of this.joints) {
        joint.dispose();
      }
    }
}
