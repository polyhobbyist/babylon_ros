/*
 * Copyright (c) 2025 Lou Amadio and Ranch Hand Robotics, LLC
 * All rights reserved.
 */

import * as BABYLON from 'babylonjs';
import { parseVector, parseRPY, parseColor } from '../src/util'
describe("Testing Vector", () => {
  test('Parse Vector3 with zeros', () => {
    let v = parseVector("0 0 0");

    expect(v.x).toBe(0);
    expect(v.y).toBe(0);
    expect(v.z).toBe(0);
  })
})

describe("Testing Color", () => {
  test('Parse Color with zeros', () => {
    let c = parseColor("0 0 0 0");

    expect(c.r).toBe(0);
    expect(c.g).toBe(0);
    expect(c.b).toBe(0);
    expect(c.a).toBe(0);
  })
})
