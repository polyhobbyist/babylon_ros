import * as BABYLON from 'babylonjs';
import {Robot} from './Robot';
import {Visual} from './Visual';
import {Link} from './Link';
import { LightInformationBlock } from 'babylonjs/Materials/index';

export class Rendering {
    public updateTransformTree(robot : Robot) : void {

    // update tfClient callbacks to inject joint transform data
    for (let [name, link]  of robot.links) {
        let frameID = link.name;
        if (frameID[0] === '/') {
          frameID = frameID.substring(1);
        }

        if (robot.joints.has(frameID)) {
          let tfMatrix = new BABYLON.Matrix();
          
          let tempFrameID = frameID;
          while (robot.joints.has(tempFrameID)) {
            let origin = robot.joints.get(tempFrameID)?.origin;
            let rpy = robot.joints.get(tempFrameID)?.rpy;
            initialPose.applyTransform(tf);
            tempFrameID = joints.get(tempFrameID).parent;
          }
          let tf = new ROSLIB.Transform({translation : initialPose.position, rotation : initialPose.orientation});
          entry[1](tf);
        } else {
          let tf = new ROSLIB.Transform();
          entry[1](tf);
        }
      }

    }
}