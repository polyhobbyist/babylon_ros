import {Link} from './Link';
import {Material} from './Material';
import {Joint} from './Joint';

export class Robot {
    public name : string = "";

    public links : Array<Link> = [];
    public materials : Array<Material> = [];

    constructor() {
    }    
}
