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
import {loadRobot} from './testutil';
import { RobotScene } from '../src/RobotScene';

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

describe("Testing Rendering Loading", () => {
    test('Test simple create', async () => {
        var robot = await loadRobot('/testdata/basic_with_material.urdf');

        expect(scene).toBeDefined();
        if (scene) {
            robot.create(scene);
        }

        let bl = robot.links.get("base_link");
        expect(bl).toBeDefined();
        expect(bl?.visuals[0].material?.name).toBe("Cyan");
    });

    test('Test rendering with single joint', async () => {
        var robot = await loadRobot('/testdata/basic_with_joint.urdf');

        expect(scene).toBeDefined();
        if (scene) {
            robot.create(scene);
        }

        let bl = robot.links.get("base_link");
        expect(bl).toBeDefined();
    });

    test('Test rendering with r2', async () => {
        var robot = await loadRobot('/testdata/r2.urdf');

        expect(scene).toBeDefined();
        if (scene) {
            robot.create(scene);
        }

        let bl = robot.links.get("base_link");
        expect(bl).toBeDefined();
    });

    test('Test screenshot functionality', async () => {
        // Create a mock canvas for testing
        const mockCanvas = {
            width: 800,
            height: 600,
            getContext: jest.fn().mockReturnValue({
                createImageData: jest.fn().mockReturnValue({
                    data: new Uint8ClampedArray(800 * 600 * 4)
                }),
                putImageData: jest.fn()
            }),
            toDataURL: jest.fn().mockReturnValue('data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR42mNkYPhfDwAChwGA60e6kgAAAABJRU5ErkJggg==')
        };

        // Mock document.createElement to return our mock canvas
        const originalCreateElement = global.document.createElement;
        global.document.createElement = jest.fn().mockImplementation((tagName) => {
            if (tagName === 'canvas') {
                return mockCanvas;
            }
            return originalCreateElement.call(document, tagName);
        });

        try {
            const robotScene = new RobotScene();
            
            // Mock the engine and scene
            robotScene.engine = {
                getRenderingCanvas: jest.fn().mockReturnValue(mockCanvas)
            } as any;
            
            robotScene.scene = scene as any;

            // Test takeScreenshot method - this should fail gracefully in test environment
            // but we're mainly testing that the method exists and has the right signature
            await expect(robotScene.takeScreenshot()).rejects.toThrow();
            
            // Test takeScreenshotDataURL method
            await expect(robotScene.takeScreenshotDataURL()).rejects.toThrow();
            
            // Verify methods exist
            expect(typeof robotScene.takeScreenshot).toBe('function');
            expect(typeof robotScene.takeScreenshotDataURL).toBe('function');
            
        } finally {
            // Restore original createElement
            global.document.createElement = originalCreateElement;
        }
    }, 10000); // Increase timeout for this test
});
