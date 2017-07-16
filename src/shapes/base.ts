/* Copyright (c) 2007 Scott Lembcke
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

import { assert, closestPointOnSegment } from '../util'
import {
    Vect, vzero,
    vadd, vsub,
    vmult, vcross, vdot,
    vdist, vlength2,
    vlerp, vnormalize,
    vneg, vperp, vrotate,
} from '../vect';

import { Body } from '../body';
import { BB } from '../bb';
import { Space } from '../space';
/// Segment query info struct.
/* These are created using literals where needed.
typedef struct cpSegmentQueryInfo {
	/// The shape that was hit, null if no collision occured.
	cpShape *shape;
	/// The normalized distance along the query segment in the range [0, 1].
	cpFloat t;
	/// The normal of the surface hit.
	cpVect n;
} cpSegmentQueryInfo;
*/



let shapeIDCounter = 0;

const CP_NO_GROUP = 0;
const CP_ALL_LAYERS = ~0;

function resetShapeIdCounter() {
    shapeIDCounter = 0;
};

/// The cpShape struct defines the shape of a rigid body.
//
/// Opaque collision shape struct. Do not create directly - instead use
/// PolyShape, CircleShape and SegmentShape.
export abstract class Shape {
    type: string;

    body: Body;

    bb_l: number;
    bb_b: number;
    bb_r: number;
    bb_t: number;

    hashid: number;
    sensor: boolean;

    e: number;
    u: number;

    surface_v: Vect;

    // TODO
    collision_type: number;

    // TODO
    collisionCode;
    collisionTable;

    // TODO
    group: number;

    // TODO
    layers;

    space: Space;

    constructor(body: Body) {
        /// The rigid body this collision shape is attached to.
        this.body = body;

        /// The current bounding box of the shape.
        this.bb_l = this.bb_b = this.bb_r = this.bb_t = 0;

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

    setElasticity(e) { this.e = e; }
    setFriction(u) { this.body.activate(); this.u = u; }
    setLayers(layers) { this.body.activate(); this.layers = layers; }
    setSensor(sensor) { this.body.activate(); this.sensor = sensor; }
    setCollisionType(collision_type) { this.body.activate(); this.collision_type = collision_type; }
    getBody() { return this.body; }

    active() {
        // return shape->prev || (shape->body && shape->body->shapeList == shape);
        return this.body && this.body.shapeList.indexOf(this) !== -1;
    }

    setBody(body) {
        assert(!this.active(), "You cannot change the body on an active shape. You must remove the shape from the space before changing the body.");
        this.body = body;
    }

    cacheBB() {
        return this.update(this.body.p, this.body.rot);
    }

    update(pos, rot) {
        assert(!isNaN(rot.x), 'Rotation is NaN');
        assert(!isNaN(pos.x), 'Position is NaN');
        this.cacheData(pos, rot);
    }

    pointQuery(p) {
        const info = this.nearestPointQuery(p);
        if (info.d < 0) return info;
    }

    getBB() {
        return new BB(this.bb_l, this.bb_b, this.bb_r, this.bb_t);
    }

    protected abstract cacheData(pos: Vect, rot: Vect): void;

    protected abstract nearestPointQuery({ x, y });
}

/* Not implemented - all these getters and setters. Just edit the object directly.
CP_DefineShapeStructGetter(cpBody*, body, Body);
void cpShapeSetBody(cpShape *shape, cpBody *body);

CP_DefineShapeStructGetter(cpBB, bb, BB);
CP_DefineShapeStructProperty(cpBool, sensor, Sensor, cpTrue);
CP_DefineShapeStructProperty(cpFloat, e, Elasticity, cpFalse);
CP_DefineShapeStructProperty(cpFloat, u, Friction, cpTrue);
CP_DefineShapeStructProperty(cpVect, surface_v, SurfaceVelocity, cpTrue);
CP_DefineShapeStructProperty(cpDataPointer, data, UserData, cpFalse);
CP_DefineShapeStructProperty(cpCollisionType, collision_type, CollisionType, cpTrue);
CP_DefineShapeStructProperty(cpGroup, group, Group, cpTrue);
CP_DefineShapeStructProperty(cpLayers, layers, Layers, cpTrue);
*/

/// Extended point query info struct. Returned from calling pointQuery on a shape.
export class PointQueryExtendedInfo {
    shape: Shape;
    d: number;
    n: Vect;

    constructor(shape) {
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

    constructor(shape, p, d) {
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

    constructor(shape, t, n) {
        /// The shape that was hit, NULL if no collision occured.
        this.shape = shape;
        /// The normalized distance along the query segment in the range [0, 1].
        this.t = t;
        /// The normal of the surface hit.
        this.n = n;
    }

    /// Get the hit point for a segment query.
    hitPoint(start, end) {
        return vlerp(start, end, this.t);
    }

    /// Get the hit distance for a segment query.
    hitDist(start, end) {
        return vdist(start, end) * this.t;
    }
}
