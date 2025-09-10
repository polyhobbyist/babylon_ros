/*
 * Copyright (c) 2025 Lou Amadio and Ranch Hand Robotics, LLC
 * All rights reserved.
 */

import * as BABYLON from 'babylonjs';
import { Material } from './Material';

export interface IGeometry {
    meshes : BABYLON.AbstractMesh[] | undefined;
    transform : BABYLON.TransformNode | undefined;

    create(scene: BABYLON.Scene, mat : Material | undefined) : void;
    dispose() : void;
    setLoadCompleteCallback?(callback: () => void) : void;
}