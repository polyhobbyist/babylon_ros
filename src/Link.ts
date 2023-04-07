import * as BABYLON from 'babylonjs';

import { IGeometry } from "./IGeometry";
import { Material } from './Material';
import { Visual } from "./Visual";

export class Link {
    public name : string = "";

    public material : Material | undefined = undefined;

    public visuals : Array<Visual> = new Array<Visual>();

    // public collisions : Visual | undefined = undefined;

    public create(scene: BABYLON.Scene, materialMap : Map<string, Material>) {
        for (let visual of this.visuals) {
            visual.create(scene, materialMap);
        }
    }

}