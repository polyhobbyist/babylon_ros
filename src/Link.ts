import * as BABYLON from 'babylonjs';

import { IGeometry } from "./IGeometry";
import { Material } from './Material';
import { Visual } from "./Visual";

export class Link {
    public name : string = "";

    public material : Material | undefined = undefined;
    public transform : BABYLON.TransformNode | undefined;

    public visuals : Array<Visual> = new Array<Visual>();

    // public collisions : Visual | undefined = undefined;

    public create(scene: BABYLON.Scene, materialMap : Map<string, Material>) {
        this.transform = new BABYLON.TransformNode("link_" + this.name, scene);

        for (let visual of this.visuals) {
            visual.create(scene, materialMap);
            if (visual.transform) {
                visual.transform.parent = this.transform;
            }
        }
    }

    public dispose() : void {
        this.material?.dispose();
        this.transform?.dispose();
        for (let visual of this.visuals) {
            visual.dispose();
        }
    }

}