/*
 * Copyright (c) 2025 Lou Amadio and Ranch Hand Robotics, LLC
 * All rights reserved.
 */

import * as BABYLON from 'babylonjs';
import * as fs from 'node:fs/promises';
import * as path from 'path';
import {parseString} from 'xml2js';
import {deserializeUrdfToRobot, deserializeMaterial, parseUrdf} from '../src/urdf'
import { Cylinder } from '../src/GeometryCylinder';
import { Robot } from '../src/Robot';
import {loadRobot} from './testutil';

let engine : any = undefined;
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
        // TODO Figure out how to do async mesh testing
        // expect(bl?.visuals[0].geometry?.mesh).toBeDefined();
    });

    test('Test Two Base_links', async () => {
        const urdf = /*xml*/ `
        <?xml version="1.0"?>
        <robot name="origins">
            <link name="base_link"/>
            <link name="base_link"/>
        </robot>
        `
        expect(async ()=> {
            var r = await deserializeUrdfToRobot(urdf);
        }).rejects.toThrow("Robot already has base_link please use another name for the second link.");
    });

    test('Test link with no name', async () => {
        const urdf = /*xml*/ `
        <?xml version="1.0"?>
        <robot name="origins">
            <link type="fixed" />
        </robot>
        `
        expect(async ()=> {
            var r = await deserializeUrdfToRobot(urdf);
        }).rejects.toThrow("Links must have a name.");
    });

    test('Test joint with no name', async () => {
        const urdf = /*xml*/ `
        <?xml version="1.0"?>
        <robot name="origins">
            <joint type="fixed" />
        </robot>
        `
        expect(async ()=> {
            var r = await deserializeUrdfToRobot(urdf);
        }).rejects.toThrow("Links must have a name.");
    });

    test('Test joint with no type', async () => {
        const urdf = /*xml*/ `
        <?xml version="1.0"?>
        <robot name="origins">
            <joint name="fixed" />
        </robot>
        `
        expect(async ()=> {
            var r = await deserializeUrdfToRobot(urdf);
        }).rejects.toThrow("Link fixed must have a type.");
    });

    test('Test revolute joint with effort', async () => {
        var robot = await loadRobot('/testdata/basic_with_joint_with_effort.urdf');

        let j = robot.joints.get("base_to_right_leg");
        expect(j).toBeDefined();
        expect(j?.lowerLimit).toBe(0);
        expect(j?.upperLimit).toBe(10);
    });

});
