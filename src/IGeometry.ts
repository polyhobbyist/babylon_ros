import * as BABYLON from 'babylonjs';
import { Material } from './Material';

export interface IGeometry {
    mesh : BABYLON.AbstractMesh | undefined;

    create(scene: BABYLON.Scene, mat: Material) : void;
}