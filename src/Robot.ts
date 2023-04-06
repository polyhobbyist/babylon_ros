import * as BABYLON from 'babylonjs';
import * as Materials from 'babylonjs-materials';

import {Link} from './Link';
import {Joint} from './Joint';

export class Robot {
    public name : string = "";

    public links : Array<Link> = [];
    public materials : Array<BABYLON.Material> = [];

    constructor() {
    }
    
    create(scene: BABYLON.Scene)
    {
        for (let link of this.links) {
            if (link.visual.geometry) {
                link.visual.geometry.create(scene);
            }
        }
    }
}
