import * as BABYLON from 'babylonjs';
import * as Materials from 'babylonjs-materials';

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
            c.backFaceCulling = false;
        } else if (this.color != undefined) {
            let m = new BABYLON.StandardMaterial(this.name, scene);
            m.diffuseColor = new BABYLON.Color3(this.color.r, this.color.g, this.color.b);
            m.alpha = this.color.a;
            this.material = m;
        } else {
            this.material = new Materials.SimpleMaterial(this.name, scene);
        }
    }

    public dispose() {
        this.material?.dispose();
    }
}