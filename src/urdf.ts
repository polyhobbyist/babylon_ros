import * as BABYLON from 'babylonjs';
import * as Materials from 'babylonjs-materials';
import {parseString} from 'xml2js';
import { Robot } from './Robot';
import { Link } from './Link';
import { Visual } from './Visual';
import { Material } from './Material';
import { Cylinder } from './GeometryCylinder';
import {parseVector, parseRPY, parseColor } from './util';

export async function parseUrdf(urdf: string) : Promise<any> {
    return await new Promise((resolve, reject) => parseString(urdf, (err, jsonData) => {
        if (err) {
        reject(err);
        }
        resolve(jsonData);
    }));
}

export function deserializeMaterial(materialNode: any) : Material {
    let m = new Material;
    m.name = materialNode.$?.name;
    console.debug(JSON.stringify(materialNode));
    if (materialNode.color?.length == 1 && materialNode.color[0].$?.rgba) {
        let color = parseColor(materialNode.color[0].$.rgba);
        m.color = color
        return m;
    } else if (materialNode.color?.length > 1) {
        throw new Error("Material ${materialNode.$?.name} has multiple color values; should only have 1.");
    }

    if (materialNode.texture?.length == 1 && materialNode.texture[0].$?.filename) {
        m.filename = materialNode.texture[0].$.filename;
    } else if (materialNode.texture?.length > 1) {
        throw new Error("Material ${materialNode.$?.name} has multiple texture values; should only have 1.");
    } 
    
    return m;
}

export async function deserializeVisual(visualObject: any) : Promise<Visual> {
    let visual = new Visual();

    if (visualObject.origin) {
        visual.origin = parseVector(visualObject?.origin?.$?.xyz);
        visual.rpy = parseRPY(visualObject?.origin?.$?.rpy);
    }

    if (visualObject.geometry[0]?.cylinder[0]) {
        visual.geometry = new Cylinder(visualObject.geometry[0].cylinder[0].$?.length || 0, visualObject.geometry[0].cylinder[0].$?.radius || 0);
    }

    return visual;
}

export async function deserializeLink(linkObject: any) : Promise<Link> {
    let link = new Link();
    link.name = linkObject.$.name;


    for (let visual of linkObject.visual) {
        link.visual.push(await deserializeVisual(visual));
    }
    return link;
}


export async function deserializeUrdfToRobot(urdfString: string) : Promise<Robot> {
    let urdf = await parseUrdf(urdfString);

    console.debug(JSON.stringify(urdf, undefined, 2));

    let robot = new Robot();

    robot.name = urdf.robot.$.name;

    if (urdf.robot?.material instanceof Array) {
        for (let material of urdf.robot.material) {
            let m = await deserializeMaterial(material);
            robot.materials.set(m.name, m);
        }
    }

    for (let link of urdf.robot.link) {
        robot.links.push(await deserializeLink(link));
    }

    return robot;
}


