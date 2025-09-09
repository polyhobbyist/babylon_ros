/*
 * Copyright (c) 2025 Lou Amadio and Ranch Hand Robotics, LLC
 * All rights reserved.
 */

import * as BABYLON from 'babylonjs';
import { Material, CollisionMaterial } from './Material';

import {Link} from './Link';
import {Joint} from './Joint';

export class Robot {
    public name : string = "";

    public transform : BABYLON.TransformNode | undefined;

    public links : Map<string, Link> = new Map<string, Link>();
    public joints : Map<string, Joint> = new Map<string, Joint>();
    public materials : Map<string, Material> = new Map<string, Material>();

    constructor() {
      let mat = new Material();
      mat.name = "default";
      mat.color = new BABYLON.Color4(0.5, 0.5, 0.5, 1);
      this.materials.set("default", mat);
      this.materials.set("collision", new CollisionMaterial());
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

      for (let [name, joint] of this.joints) {
        joint.create(scene, this.materials);
      }

      // NOTES:
      // 1. The base_link is the root of the transform tree for mobile robots.
      // 2. base_footprint is the root of the transform tree for turtlebot and walking robots.
      // 3. All link transforms with no parent will be parented to this.transform.

/*
      // Commenting out this code for now. It turns out that some companies make up their own root node. Once we configure the visual tree, any unparented transform will be parented to this.transform.

      // for issue https://github.com/ms-iot/vscode-ros/issues/939,
      // Base_footprint is an orphan tree, so applying our root transform to convert to babylon coordinate system won't work.
      // We need to apply the transform to the base_link instead.

      let base = this.links.get("base_link");

      if (base == null || base == undefined) {
        base = this.links.get("base_footprint");
      }

      // unitree uses world as the base link
      if (base == null || base == undefined) {
        base = this.links.get("world");
      }

      if (base == undefined) {
        throw new Error("No base_link or base_footprint defined in this robot");
      } 
*/


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

      for (let [name, link] of this.links) {
        if (link.transform != undefined && 
            link.transform.parent == undefined) {
          link.transform.parent = this.transform;
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
