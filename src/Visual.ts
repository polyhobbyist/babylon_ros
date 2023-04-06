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
        if (mat?.isReference()) {
            mat = materialMap.get(mat.name);
        }

        if (mat == undefined) {
            mat = materialMap.get("default");
        }

        if (this.geometry && mat) {
            this.geometry.create(scene, mat);
        }
    }
}