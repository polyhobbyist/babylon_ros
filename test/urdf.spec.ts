import * as BABYLON from 'babylonjs';
import * as fs from 'node:fs/promises';
import * as path from 'path';
import {deserializeUrdfToRobot} from '../src/urdf'
import { Cylinder } from '../src/GeometryCylinder';

describe("Testing URDF Loading", () => {
    test('Test Basic Loading', async () => {
        const basicUrdfFilename = path.join(__dirname, '/testdata/basic.urdf');
        const basicUrdf = await fs.readFile(basicUrdfFilename);
        var robot = await deserializeUrdfToRobot(basicUrdf.toString());

        expect(robot.name).toBe('myfirst');
        expect(robot.links.length).toBe(1);
        expect(robot.links[0].name).toBe('base_link');
        expect(robot.links[0].visual.geometry).toBeInstanceOf(Cylinder);
    });
});
