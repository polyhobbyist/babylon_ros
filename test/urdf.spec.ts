import * as BABYLON from 'babylonjs';
import * as fs from 'node:fs/promises';
import * as path from 'path';
import {parseString} from 'xml2js';
import {deserializeUrdfToRobot, deserializeMaterial, parseUrdf} from '../src/urdf'
import { Cylinder } from '../src/GeometryCylinder';

let engine = undefined;
let scene = undefined;
beforeAll(() => {
    engine = new BABYLON.NullEngine();
    scene = new BABYLON.Scene(engine);
  });
  
  afterAll(() => {
    scene = undefined
    engine = undefined;
  });

describe("Testing URDF Loading", () => {
    test('Test Basic Loading', async () => {
        const basicUrdfFilename = path.join(__dirname, '/testdata/basic.urdf');
        const basicUrdf = await fs.readFile(basicUrdfFilename);
        var robot = await deserializeUrdfToRobot(basicUrdf.toString());

        expect(robot.name).toBe('myfirst');
        expect(robot.links.length).toBe(1);
        expect(robot.links[0].name).toBe('base_link');
        expect(robot.links[0].visual[0].geometry).toBeInstanceOf(Cylinder);
    });

    test('Test Material Loading', async () => {
        const basicUrdfFilename = path.join(__dirname, '/testdata/basic_with_material.urdf');
        const basicUrdf = await fs.readFile(basicUrdfFilename);
        var robot = await deserializeUrdfToRobot(basicUrdf.toString());

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
});
