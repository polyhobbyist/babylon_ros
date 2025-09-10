/*
 * Copyright (c) 2025 Lou Amadio and Ranch Hand Robotics, LLC
 * All rights reserved.
 */

import * as BABYLON from 'babylonjs';

import { IGeometry } from "./IGeometry";
import { Material } from './Material';
import { Visual } from "./Visual";

export class Link {
    public name : string = "";

    public material : Material | undefined = undefined;
    public transform : BABYLON.TransformNode | undefined;

    public visuals : Array<Visual> = new Array<Visual>();

    public collisions : Array<Visual> = new Array<Visual>();

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
                if (collision.transform) {
                    collision.transform.parent = this.transform;
                }
                collision.setEnabled(false);
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