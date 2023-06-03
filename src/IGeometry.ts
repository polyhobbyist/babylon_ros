import * as BABYLON from 'babylonjs';
import { Material } from './Material';

export interface IGeometry {
    mesh : BABYLON.AbstractMesh | undefined;
    transform : BABYLON.TransformNode | undefined;

    create(scene: BABYLON.Scene, mat : Material | undefined) : void;
    dispose() : void;
}