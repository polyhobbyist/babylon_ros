import * as BABYLON from 'babylonjs';
import { Material } from './Material';

import {IGeometry} from './IGeometry';

export class Visual {
    public name : string = "";

    public geometry : IGeometry | undefined = undefined;

    public material : Material | undefined = undefined;

    public origin : BABYLON.Vector3 = new BABYLON.Vector3(0, 0, 0);
    public rpy : BABYLON.Vector3 = new BABYLON.Vector3(0, 0, 0);
    public transform : BABYLON.TransformNode | undefined;

    public create(scene: BABYLON.Scene, materialMap : Map<string, Material>) : void {

        this.transform = new BABYLON.TransformNode("visual_" + this.name, scene);
        this.transform.locallyTranslate(this.origin);
        this.transform.rotation = this.rpy;

        let mat = this.material;
        if (this.material) {
            if (this.material.isReference()) {
                mat = materialMap.get(this.material.name);
            } else {
                this.material.create(scene);
            }
        }

        if (mat == undefined) {
            mat = materialMap.get("default");
        }

        if (this.geometry && mat) {
            this.geometry.create(scene, mat);
            if (this.transform && this.geometry.mesh) {
                this.geometry.mesh.parent = this.transform;
            }
        }
    }
}