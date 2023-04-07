import { IGeometry } from "./IGeometry";
import { Material } from './Material';
import { Visual } from "./Visual";

export class Link {
    public name : string = "";

    public material : Material | undefined = undefined;

    public visual : Array<Visual> = new Array<Visual>();

    // public collision : Visual | undefined = undefined;

}