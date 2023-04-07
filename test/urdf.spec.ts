import * as BABYLON from 'babylonjs';
import * as fs from 'node:fs/promises';
import * as path from 'path';
import {parseString} from 'xml2js';
import {deserializeUrdfToRobot, deserializeMaterial, parseUrdf} from '../src/urdf'
import { Cylinder } from '../src/GeometryCylinder';
import { Robot } from '../src/Robot';

let engine = undefined;
let scene : BABYLON.Scene | undefined = undefined;

async function loadRobot(file : string) : Promise<Robot> {
    const basicUrdfFilename = path.join(__dirname, file);
    const basicUrdf = await fs.readFile(basicUrdfFilename);
    return await deserializeUrdfToRobot(basicUrdf.toString());
}

beforeAll(() => {
    // Needed for testing material loading
    engine = new BABYLON.NullEngine();
    scene = new BABYLON.Scene(engine);
});

afterAll(() => {
    scene = undefined
    engine = undefined;
});

describe("Testing URDF Loading", () => {
    test('Test Basic Loading', async () => {
        var robot = await loadRobot('/testdata/basic.urdf');

        let bl = robot.links.get("base_link");

        expect(robot.name).toBe('myfirst');
        expect(robot.links.size).toBe(1);
        expect(bl).toBeDefined();
        if (bl) {
            expect(bl.visuals[0].geometry).toBeInstanceOf(Cylinder);
        }
    });

    test('Test Basic with Joint', async () => {
        var robot = await loadRobot('/testdata/basic_with_joint.urdf');

        let bl = robot.links.get("base_link");

        expect(robot.name).toBe('origins');
        expect(robot.links.size).toBe(2);
        expect(bl).toBeDefined();
        if (bl) {
            expect(bl.visuals[0].geometry).toBeInstanceOf(Cylinder);
        }
    });

    test('Test Material Loading', async () => {
        var robot = await loadRobot('/testdata/basic_with_material.urdf');

        expect(robot.name).toBe('myfirst');

        expect(robot.materials.size).toBe(2);
        expect(robot.materials.get("black")).toBeDefined();
        
    });

    test('Test Material Color', async () => {
        const materialColor = /*xml*/ `
        <?xml version="1.0"?>
        <material name="Cyan">
            <color rgba="0 1.0 1.0 1.0"/>
            </material>
        `
        var m = await parseUrdf(materialColor);
        var mat = deserializeMaterial(m.material);
    
        expect(mat.name).toBe('Cyan');
        expect(mat.filename).toBe("");
        expect(mat.color).toBeDefined();
        expect(mat.color?.a).toBeCloseTo(1.0);
        expect(mat.color?.r).toBeCloseTo(0.0);
        expect(mat.color?.g).toBeCloseTo(1.0);
        expect(mat.color?.b).toBeCloseTo(1.0);
    });

    test('Test Material Filename', async () => {
        const materialColor = /*xml*/ `
        <?xml version="1.0"?>
        <material name="foofile">
            <texture filename="foo"/>
            </material>
        `
        var m = await parseUrdf(materialColor);
        var mat = deserializeMaterial(m.material);
    
        expect(mat.name).toBe('foofile');
        expect(mat.filename).toBe("foo");
        expect(mat.color).toBeUndefined();
    });

    test('Test Loading STL Mesh', async () => {
        const basicUrdfFilename = path.join(__dirname, '/testdata/basic_with_stl_mesh.urdf');
        const basicUrdf = await fs.readFile(basicUrdfFilename);
        var robot = await deserializeUrdfToRobot(basicUrdf.toString());

        if (scene) {
            robot.create(scene);
        }

        let bl = robot.links.get("base_link");
        expect(bl).toBeDefined();
        expect(bl?.visuals[0].geometry?.mesh?.name).toBe("XYZ Cube");
    });
});
