/*
 * Copyright (c) 2025 Lou Amadio and Ranch Hand Robotics, LLC
 * All rights reserved.
 */

import * as fs from 'node:fs/promises';
import * as path from 'path';
import * as BABYLON from 'babylonjs';
import { Robot } from '../src/Robot';
import {deserializeUrdfToRobot} from '../src/urdf'

export async function loadRobot(file : string) : Promise<Robot> {
  const basicUrdfFilename = path.join(__dirname, file);
  const basicUrdf = await fs.readFile(basicUrdfFilename);
  return await deserializeUrdfToRobot(basicUrdf.toString());
}
