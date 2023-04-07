import * as BABYLON from 'babylonjs';
import * as fs from 'node:fs/promises';
import * as path from 'path';
import {parseString} from 'xml2js';
import {deserializeUrdfToRobot, deserializeMaterial, parseUrdf} from '../src/urdf'
import { Cylinder } from '../src/GeometryCylinder';

let engine = undefined;
let scene : BABYLON.Scene | undefined = undefined;

beforeAll(() => {
    // Needed for testing material loading
    engine = new BABYLON.NullEngine();
    scene = new BABYLON.Scene(engine);
});

afterAll(() => {
    scene = undefined
    engine = undefined;
});

describe("Testing Rendering Loading", () => {
    test('Test simple create', async () => {
        const basicUrdfFilename = path.join(__dirname, '/testdata/basic_with_material.urdf');
        const basicUrdf = await fs.readFile(basicUrdfFilename);
        var robot = await deserializeUrdfToRobot(basicUrdf.toString());

        expect(scene).toBeDefined();
        if (scene) {
            robot.create(scene);
        }

        let bl = robot.links.get("base_link");
        expect(bl).toBeDefined();
        expect(bl?.visual[0].material?.name).toBe("Cyan");
    });
});
