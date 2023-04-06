import { IGeometry } from "./IGeometry";
import { Visual } from "./Visual";

export class Link {
    public name : string = "";

    public visual : Array<Visual> = new Array<Visual>();

    // public collision : Visual | undefined = undefined;

}