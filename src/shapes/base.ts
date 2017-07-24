/* Copyright (c) 2017 Ben Mather
 * Forked from Chipmunk JS, copyright (c) 2013 Seph Gentle
 * Ported from Chipmunk, copyright (c) 2010 Scott Lembcke
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

import { assert, closestPointOnSegment } from "../util";
import {
    vadd, vcross,
    vdist, vdot,
    Vect, vlength2, vlerp,
    vmult, vneg,
    vnormalize, vperp,
    vrotate, vsub, vzero,
} from "../vect";

import { BB } from "../bb";
import { Body } from "../body";
import { Space } from "../space";

let shapeIDCounter = 0;

const CP_NO_GROUP = 0;
const CP_ALL_LAYERS = ~0;

function resetShapeIdCounter() {
    shapeIDCounter = 0;
}

/// The cpShape struct defines the shape of a rigid body.
//
/// Opaque collision shape struct. Do not create directly - instead use
/// PolyShape, CircleShape and SegmentShape.
export abstract class Shape {
    type: string;

    body: Body;

    bbL: number;
    bbB: number;
    bbR: number;
    bbT: number;

    hashid: number;
    sensor: boolean;

    e: number;
    u: number;

    surface_v: Vect;

    collision_type: number;
    collisionCode: number;

    // TODO TODO TODO should be a map
    collisionTable: any;

    // TODO
    group: number;

    // TODO
    layers: number;

    space: Space;

    constructor(body: Body) {
        /// The rigid body this collision shape is attached to.
        this.body = body;

        /// The current bounding box of the shape.
        this.bbL = this.bbB = this.bbR = this.bbT = 0;

        this.hashid = shapeIDCounter++;

        /// Sensor flag.
        /// Sensor shapes call collision callbacks but don't produce collisions.
        this.sensor = false;

        /// Coefficient of restitution. (elasticity)
        this.e = 0;
        /// Coefficient of friction.
        this.u = 0;
        /// Surface velocity used when solving for friction.
        this.surface_v = vzero;

        /// Collision type of this shape used when picking collision handlers.
        this.collision_type = 0;
        /// Group of this shape. Shapes in the same group don't collide.
        this.group = 0;
        // Layer bitmask for this shape. Shapes only collide if the bitwise and of their layers is non-zero.
        this.layers = CP_ALL_LAYERS;

        this.space = null;
    }

    setElasticity(e: number): void {
        this.e = e;
    }

    setFriction(u: number): void {
        this.body.activate(); this.u = u;
    }

    setLayers(layers: number): void {
        this.body.activate(); this.layers = layers;
    }

    setSensor(sensor: boolean): void {
        this.body.activate(); this.sensor = sensor;
    }

    setCollisionType(collision_type: number): void {
        this.body.activate(); this.collision_type = collision_type;
    }

    getBody() {
        return this.body;
    }

    active() {
        // return shape->prev || (shape->body && shape->body->shapeList == shape);
        return this.body && this.body.shapeList.indexOf(this) !== -1;
    }

    setBody(body: Body): void {
        assert(!this.active(), "You cannot change the body on an active shape. You must remove the shape from the space before changing the body.");
        this.body = body;
    }

    cacheBB(): void {
        return this.update(this.body.p, this.body.rot);
    }

    update(pos: Vect, rot: Vect) {
        assert(!isNaN(rot.x), "Rotation is NaN");
        assert(!isNaN(pos.x), "Position is NaN");
        this.cacheData(pos, rot);
    }

    pointQuery(p: Vect) {
        const info = this.nearestPointQuery(p);
        if (info.d < 0) return info;
    }

    getBB(): BB {
        return new BB(this.bbL, this.bbB, this.bbR, this.bbT);
    }

    protected abstract cacheData(pos: Vect, rot: Vect): void;

    abstract nearestPointQuery(poing: Vect): NearestPointQueryInfo;

    abstract segmentQuery(a: Vect, b: Vect): SegmentQueryInfo;
}

/// Extended point query info struct. Returned from calling pointQuery on a shape.
export class PointQueryExtendedInfo {
    shape: Shape;
    d: number;
    n: Vect;

    constructor(shape: Shape) {
        /// Shape that was hit, NULL if no collision occurred.
        this.shape = shape;
        /// Depth of the point inside the shape.
        this.d = Infinity;
        /// Direction of minimum norm to the shape's surface.
        this.n = vzero;
    }
}

export class NearestPointQueryInfo {
    shape: Shape;
    p: Vect;
    d: number;

    constructor(shape: Shape, p: Vect, d: number) {
        /// The nearest shape, NULL if no shape was within range.
        this.shape = shape;
        /// The closest point on the shape's surface. (in world space coordinates)
        this.p = p;
        /// The distance to the point. The distance is negative if the point is inside the shape.
        this.d = d;
    }
}

export class SegmentQueryInfo {
    shape: Shape;
    t: number;
    n: Vect;

    constructor(shape: Shape, t: number, n: Vect) {
        /// The shape that was hit, NULL if no collision occured.
        this.shape = shape;
        /// The normalized distance along the query segment in the range [0, 1].
        this.t = t;
        /// The normal of the surface hit.
        this.n = n;
    }

    /// Get the hit point for a segment query.
    hitPoint(start: Vect, end: Vect): Vect {
        return vlerp(start, end, this.t);
    }

    /// Get the hit distance for a segment query.
    hitDist(start: Vect, end: Vect): number {
        return vdist(start, end) * this.t;
    }
}
