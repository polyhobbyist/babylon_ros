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

      for (let [name, mat] of this.materials) {
        mat.create(scene);
      }

      for (let [name, link] of this.links) {
        link.create(scene, this.materials);
      }

      let base_link = this.links.get("base_link");
      if (base_link == undefined) {
        throw new Error("No base_link defined in this robot");
      }

      for (let [name, joint] of this.joints) {

        if (joint.parent && 
            joint.parent.visuals.length > 0 && 
            joint.transform && 
            joint.parent.visuals[0].transform) {
          joint.transform.parent = joint.parent.visuals[0].transform;

          // BUGBUG - spec says the multiple visuals can exist, but which is the transform parent?
          if (joint.child && joint.child.visuals) {
            for (let v of joint.child.visuals) {
              if (v.transform ) {
                v.transform.parent = joint.transform;
              }
            }
          }
        }
      }
    }
}
