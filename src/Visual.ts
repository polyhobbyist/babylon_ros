import * as BABYLON from 'babylonjs';
import { Material } from './Material';

import {IGeometry} from './IGeometry';

export class Visual {
    public name : string = "";

    public geometry : IGeometry | undefined = undefined;

    public material : Material | undefined = undefined;

    public origin : BABYLON.Vector3 = new BABYLON.Vector3(0, 0, 0);
    public rpy : BABYLON.Quaternion = new BABYLON.Quaternion(0, 0, 0, 1);
    public create(scene: BABYLON.Scene, materialMap : Map<string, Material>) : void {

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
            let pivotMatrix = BABYLON.Matrix.Compose(new BABYLON.Vector3(1.0, 1.0, 1.0), this.rpy, this.origin);
            this.geometry.create(scene, mat);
            this.geometry.mesh?.setPivotMatrix(pivotMatrix);
        }
    }
}