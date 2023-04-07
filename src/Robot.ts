import * as BABYLON from 'babylonjs';
import { Material } from './Material';

import {Link} from './Link';
import {Joint} from './Joint';

export class Robot {
    public name : string = "";

    public links : Map<string, Link> = new Map<string, Link>();
    public joints : Map<string, Joint> = new Map<string, Joint>();
    public materials : Map<string, Material> = new Map<string, Material>();

    constructor() {
        this.materials.set("default", new Material());
    }
    
    create(scene: BABYLON.Scene)
    {
        for (let [name, mat] of this.materials) {
            mat.create(scene);
        }

        for (let [name, link]  of this.links) {
            for (let visual of link.visual) {
                visual.create(scene, this.materials);
            }
        }
    }
}
