import * as BABYLON from 'babylonjs';
import * as Materials from 'babylonjs-materials';
import {parseString} from 'xml2js';
import { Robot } from './Robot';
import { Link } from './Link';
import { Joint, JointType } from './Joint';
import { Visual } from './Visual';
import { Material } from './Material';
import { Cylinder } from './GeometryCylinder';
import { Box } from './GeometryBox';
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
        throw new Error(`Material ${materialNode.$?.name} has multiple color values; should only have 1.`);
    }

    if (materialNode.texture?.length == 1 && materialNode.texture[0].$?.filename) {
        m.filename = materialNode.texture[0].$.filename;
    } else if (materialNode.texture?.length > 1) {
        throw new Error(`Material ${materialNode.$?.name} has multiple texture values; should only have 1.`);
    } 
    
    return m;
}

export async function deserializeVisual(visualObject: any) : Promise<Visual> {
    let visual = new Visual();

    if (visualObject.origin && visualObject.origin.length == 1) {
      if (visualObject.origin[0].$.xyz) {
        visual.origin = parseVector(visualObject?.origin[0].$.xyz);
      }
      if (visualObject.origin[0].$.rpy) {
        visual.rpy = parseRPY(visualObject.origin[0].$.rpy);
      }
    }

    if (visualObject.material?.length == 1) {
      visual.material = deserializeMaterial(visualObject.material[0]);
    } else if (visualObject.material?.length > 1) {
        throw new Error("Visual has multiple materials; must only have 1.");
    } 

    if (visualObject.geometry[0]?.cylinder && visualObject.geometry[0]?.cylinder.length == 1) {
      visual.geometry = new Cylinder(visualObject.geometry[0].cylinder[0].$?.length || 0, visualObject.geometry[0].cylinder[0].$?.radius || 0);
    } else if  (visualObject.geometry[0]?.box && visualObject.geometry[0]?.box.length == 1) {
      let size = parseVector(visualObject.geometry[0].box[0].$.size);
      visual.geometry = new Box(size.x, size.y, size.z);
    }



    return visual;
}

export async function deserializeLink(linkObject: any) : Promise<Link> {
    let link = new Link();
    link.name = linkObject.$.name;

    if (linkObject.material?.length == 1) {
        throw new Error(`Link ${link.name} has a material; Did you mean to put it on visual?`);
    } 

    for (let visual of linkObject.visual) {
      let v = await deserializeVisual(visual);
      v.name = link.name;
      link.visuals.push(v);
    }
    return link;
}

export async function deserializeJoint(jointObject: any) : Promise<Joint> {
    let joint = new Joint();
    joint.name = jointObject.$.name;
    joint.type = jointObject.$?.type as JointType;

    if (jointObject.limit?.length == 1) {
        joint.lowerLimit = parseFloat(jointObject.limit[0].$?.lower);
        joint.upperLimit = parseFloat(jointObject.limit[0].$?.upper);
    }

    if (jointObject.parent?.length == 1) {
        joint.parentName = jointObject.parent[0].$.link;
    } else {
        throw new Error(`Joint ${jointObject.$?.name} has multiple parents, and requires only a single.`);
    }

    if (jointObject.child?.length == 1) {
        joint.childName = jointObject.child[0].$.link;
    } else {
        throw new Error(`Joint ${jointObject.$?.name} has multiple children, and requires only a single.`);
    }

    if (jointObject.origin) {
      if (jointObject?.origin?.$?.xyz) {
        joint.origin = parseVector(jointObject?.origin?.$?.xyz);
      }
      if (jointObject?.origin?.$?.rpy) {
        joint.rpy = parseRPY(jointObject?.origin?.$?.rpy);
      }
    }

    return joint;
}

export async function deserializeUrdfToRobot(urdfString: string) : Promise<Robot> {
    let urdf = await parseUrdf(urdfString);

    console.debug(JSON.stringify(urdf, undefined, 2));

    let robot = new Robot();

    robot.name = urdf.robot.$.name;

    if (urdf.robot.material instanceof Array) {
      for (let material of urdf.robot.material) {
        let m = await deserializeMaterial(material);
        robot.materials.set(m.name, m);
      }
    }

    if (urdf.robot.link instanceof Array) {
      for (let link of urdf.robot.link) {
        let l = await deserializeLink(link);
        robot.links.set(l.name, l);
      }
    }

    if (urdf.robot.joint instanceof Array) {
      for (let joint of urdf.robot.joint) {
        let j = await deserializeJoint(joint);
        robot.joints.set(j.name, j);

        let p = robot.links.get(j.parentName);
        if (p) {
          j.parent = p;
        } else {
          throw new Error(`Joint ${j.name} has refered to a link ${j.parentName} which could not be found.`);
        }

        let c = robot.links.get(j.childName);
        if (c) {
          j.child = c;
        } else {
          throw new Error(`Joint ${j.name} has refered to a link ${j.childName} which could not be found.`);
        }
      }
    }

    return robot;
}


