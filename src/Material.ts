/*
 * Copyright (c) 2025 Lou Amadio and Ranch Hand Robotics, LLC
 * All rights reserved.
 */

import * as BABYLON from 'babylonjs';
//import {FireProceduralTexture} from 'babylonjs-procedural-textures';

export class Material {
    public name : string = "default";
    public filename : string = "";
    public color : BABYLON.Color4 | undefined = undefined;
    public material : BABYLON.Material | undefined = undefined

    constructor() {
    }

    public isReference() : boolean {
        return this.filename === "" && this.color == undefined;
    }
    
    public create(scene: BABYLON.Scene) : void {
        if (this.filename) {
            let c = new BABYLON.StandardMaterial(this.name, scene);
            c.diffuseTexture = new BABYLON.Texture(this.filename);
            c.diffuseTexture.hasAlpha = true;
        } else {
            let m = new BABYLON.StandardMaterial(this.name, scene);

            if (this.color == undefined) {
                this.color = new BABYLON.Color4(1, 1, 1, 1);
            }
            m.diffuseColor = new BABYLON.Color3(this.color.r, this.color.g, this.color.b);
            m.alpha = this.color.a;
            this.material = m;
        }
        if (this.material) {
            this.material.backFaceCulling = false;
        }
    }

    public dispose() {
        this.material?.dispose();
    }
}

export class CollisionMaterial extends Material {
    constructor() {
        super();
        this.name = "collision";
    }
    public create(scene: BABYLON.Scene) : void {
        let m = new BABYLON.StandardMaterial('collision_material', scene);
        m.diffuseColor = new BABYLON.Color3(1, 0, 0);
        m.alpha = 0.25;
        m.backFaceCulling = false;

        //var fireTexture = new FireProceduralTexture("fire", 256, scene);
        //m.emissiveTexture = fireTexture;
        //m.opacityTexture = fireTexture;

        this.material = m;
    }
    
}
