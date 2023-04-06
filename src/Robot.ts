import * as BABYLON from 'babylonjs';
import { Material } from './Material';

import {Link} from './Link';
import {Joint} from './Joint';

export class Robot {
    public name : string = "";

    public links : Array<Link> = [];
    public materials : Map<string, Material> = new Map<string, Material>();

    constructor() {
        this.materials.set("default", new Material());
    }
    
    create(scene: BABYLON.Scene)
    {
        for (let [name, mat] of this.materials) {
            mat.create(scene);
        }

        for (let link of this.links) {
            for (let visual of link.visual) {
                visual.create(scene, this.materials);
            }
        }
    }
}
