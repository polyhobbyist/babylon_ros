/*
 * Copyright (c) 2025 Lou Amadio and Ranch Hand Robotics, LLC
 * All rights reserved.
 */

import * as BABYLON from 'babylonjs';

export function parseVector(vec: string) : BABYLON.Vector3 {
  vec = vec.trim();
  var xyz = vec.split(' ');
  if (xyz.length != 3) {
    throw new Error("Vector ${vec} does not have 3 values")
  }

  return new BABYLON.Vector3(parseFloat(xyz[0]), parseFloat(xyz[1]), parseFloat(xyz[2]));
}

export function parseRPY(rpy: string) : BABYLON.Vector3 {
  let v = parseVector(rpy);

  // ROS is Roll, Pitch, Yaw - BABYLON Vector is Pitch, Yaw, Roll
  return new BABYLON.Vector3(v.x, v.y, v.z);
}

export function parseColor(color: string) : BABYLON.Color4 {
  color = color.trim();
  var rgba = color.split(' ');
  if (rgba.length != 4) {
    throw new Error("Color ${rgba} does not have 4 values")
  }

  return new BABYLON.Color4(parseFloat(rgba[0]), parseFloat(rgba[1]), parseFloat(rgba[2]), parseFloat(rgba[3]));
}

export function applyRotationToTransform(transformNode : BABYLON.TransformNode, vec: BABYLON.Vector3) {

  transformNode.addRotation(0, 0, vec.z).addRotation(0, vec.y, 0).addRotation(vec.x, 0, 0);
}
