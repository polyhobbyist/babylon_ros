import { IGeometry } from "./IGeometry";

export class Cylinder implements IGeometry {
    public length : number = 0;
    public radius : number = 0;

    constructor(l : number, r: number) {
        this.length = l;
        this.radius = r;
    }    
}