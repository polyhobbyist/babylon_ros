import * as BABYLON from 'babylonjs';

export interface IGeometry {
    mesh : BABYLON.Mesh | undefined;

    create(scene: BABYLON.Scene) : void;
}