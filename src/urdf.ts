import {parseString} from 'xml2js';
import { Robot } from './Robot';
import { Link } from './Link';
import { Cylinder } from './GeometryCylinder';

export async function parseUrdf(urdf: string) {
    return await new Promise((resolve, reject) => parseString(urdf, (err, jsonData) => {
      if (err) {
        reject(err);
      }
      resolve(jsonData);
    }));
}

export async function deserializeLink(linkObject: any) : Promise<Link> {
    let link = new Link();
    link.name = linkObject.$.name;

    // TODO: Origin
    // TODO: RPY
    if (linkObject?.visual[0]?.geometry[0]?.cylinder[0]) {
        link.geometry = new Cylinder(linkObject.visual[0].geometry[0].cylinder[0].$?.length || 0, linkObject.visual[0].geometry[0].cylinder[0].$?.radius || 0);
    }

    return link;
}


export async function deserializeUrdfToRobot(urdfString: string) : Promise<Robot> {
    let urdf = await parseUrdf(urdfString) as any;

    console.debug(JSON.stringify(urdf, undefined, 2));

    let robot = new Robot();

    robot.name = urdf.robot.$.name;

    for (let link of urdf.robot.link) {
        robot.links.push(await deserializeLink(link));
    }

    return robot;
}


