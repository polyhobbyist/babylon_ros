import * as BABYLON from 'babylonjs';

import { IGeometry } from "./IGeometry";
import { Material } from './Material';
import { Visual } from "./Visual";
import { Inertial } from './Inertial';

export class Link {
    public name : string = "";

    public material : Material | undefined = undefined;
    public transform : BABYLON.TransformNode | undefined;

    public visuals : Array<Visual> = new Array<Visual>();

    public collisions : Array<Visual> = new Array<Visual>();

    public inertial : Inertial | undefined = undefined;

    public create(scene: BABYLON.Scene, materialMap : Map<string, Material>) {
        this.transform = new BABYLON.TransformNode(this.name, scene);

        if (this.visuals.length > 0) {
            for (let visual of this.visuals) {
                visual.create(scene, materialMap);
                if (visual.transform) {
                    visual.transform.parent = this.transform;
                }
            }
        }

        if (this.collisions.length > 0) {
            for (let collision of this.collisions) {
                collision.material = materialMap.get("collision");
                collision.create(scene, materialMap);
                collision.setEnabled(false);
                if (collision.transform) {
                    collision.transform.parent = this.transform;
                }
            }
        }
    }

    public dispose() : void {
        if (this.visuals.length > 0) {
            for (let visual of this.visuals) {
                visual.dispose();
            }
        }
        if (this.collisions.length > 0) {
            for (let collision of this.collisions) {
                collision.dispose();
            }
        }
        this.material?.dispose();
        this.transform?.dispose();
    }

}