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

/// @defgroup cpBody cpBody
/// Chipmunk's rigid body type. Rigid bodies hold the physical properties of an object like
/// it's mass, and position and velocity of it's center of gravity. They don't have an shape on their own.
/// They are given a shape by creating collision shapes (cpShape) that point to the body.
/// @{

// DEBUG,
import { Arbiter } from "./arbiter";
import { Constraint } from "./constraints";
import { apply_impulse } from "./constraints/util";
import { Shape } from "./shapes";
import { Space } from "./space";
import { componentActivate, componentRoot } from "./space-components";
import {
    assert, assertSoft,
    clamp, deleteObjFromList,
} from "./util";
import {
    vadd, vcross,
    Vect, vmult,
    vperp, vrotate,
    vsub, vunrotate, vzero,
} from "./vect";

function filterConstraints(node: Constraint, body: Body, filter: Constraint) {
    if (node === filter) {
        return node.next(body);
    } else if (node.a === body) {
        node.nextA = filterConstraints(node.nextA, body, filter);
    } else {
        node.nextB = filterConstraints(node.nextB, body, filter);
    }

    return node;
}

export class Body {
    /// Position of the rigid body's center of gravity.
    p: Vect;
    /// Velocity of the rigid body's center of gravity.
    vx: number;
    vy: number;
    /// Force acting on the rigid body's center of gravity.
    f: Vect;

    /// Rotation of the body around it's center of gravity in radians.
    /// Must agree with cpBody.rot! Use cpBodySetAngle() when changing the angle for this reason.
    a: number;
    /// Angular velocity of the body around it's center of gravity in radians/second.
    w: number;
    /// Torque applied to the body around it's center of gravity.
    t: number;

    /// Cached unit length vector representing the angle of the body.
    /// Used for fast rotations using cpvrotate().
    //cpVect rot;

    /// Maximum velocity allowed when updating the velocity.
    v_limit: number;
    /// Maximum rotational rate (in radians/second) allowed when updating the angular velocity.
    w_limit: number;

    // This stuff is all private.
    v_biasy: number;
    v_biasx: number;
    w_bias: number;

    space: Space;

    shapeList: Shape[];
    // These are both wacky linked lists.
    arbiterList: Arbiter;
    constraintList: Constraint;

    // This stuff is used to track information on the collision graph.
    // TODO
    nodeRoot: Body;
    nodeNext: Body;
    nodeIdleTime: number;

    // Mass and one-over-mass.
    mass: number;
    massInv: number;

    // Inertia and one over inertia.
    inertia: number;
    inertiaInv: number;

    // Set this.a and this.rot
    rot: Vect;

    constructor(m: number, i: number) {
        /// Mass of the body.
        /// Must agree with cpBody.m_inv! Use body.setMass() when changing the mass for this reason.
        //this.m;
        /// Mass inverse.
        //this.m_inv;

        /// Moment of inertia of the body.
        /// Must agree with cpBody.i_inv! Use body.setMoment() when changing the moment for this reason.
        //this.i;
        /// Moment of inertia inverse.
        //this.i_inv;

        /// Position of the rigid body's center of gravity.
        this.p = new Vect(0, 0);
        /// Velocity of the rigid body's center of gravity.
        this.vx = this.vy = 0;
        /// Force acting on the rigid body's center of gravity.
        this.f = new Vect(0, 0);

        /// Rotation of the body around it's center of gravity in radians.
        /// Must agree with cpBody.rot! Use cpBodySetAngle() when changing the angle for this reason.
        //this.a;
        /// Angular velocity of the body around it's center of gravity in radians/second.
        this.w = 0;
        /// Torque applied to the body around it's center of gravity.
        this.t = 0;

        /// Cached unit length vector representing the angle of the body.
        /// Used for fast rotations using cpvrotate().
        //cpVect rot;

        /// Maximum velocity allowed when updating the velocity.
        this.v_limit = Infinity;
        /// Maximum rotational rate (in radians/second) allowed when updating the angular velocity.
        this.w_limit = Infinity;

        // This stuff is all private.
        this.v_biasx = this.v_biasy = 0;
        this.w_bias = 0;

        this.space = null;

        this.shapeList = [];
        this.arbiterList = null; // These are both wacky linked lists.
        this.constraintList = null;

        // This stuff is used to track information on the collision graph.
        this.nodeRoot = null;
        this.nodeNext = null;
        this.nodeIdleTime = 0;

        // Set this.m and this.m_inv
        this.setMass(m);

        // Set this.i and this.i_inv
        this.setMoment(i);

        // Set this.a and this.rot
        this.rot = new Vect(0, 0);
        this.setAngle(0);
    }

    getPos(): Vect {
        return this.p;
    }
    getVel(): Vect {
        return new Vect(this.vx, this.vy);
    }
    getAngVel(): number {
        return this.w;
    }

    /// Returns true if the body is sleeping.
    isSleeping(): boolean {
        return this.nodeRoot !== null;
    }

    /// Returns true if the body is static.
    isStatic(): boolean {
        return this.nodeIdleTime === Infinity;
    }

    /// Returns true if the body has not been added to a space.
    isRogue(): boolean {
        return this.space === null;
    }

    // It would be nicer to use defineProperty for this, but its about 30x slower:
    // http://jsperf.com/defineproperty-vs-setter
    setMass(mass: number): void {
        assert(mass > 0, "Mass must be positive and non-zero.");

        //activate is defined in cpSpaceComponent
        this.activate();
        this.mass = mass;
        this.massInv = 1 / mass;
    }

    setMoment(moment: number): void {
        assert(moment > 0, "Moment of Inertia must be positive and non-zero.");

        this.activate();
        this.inertia = moment;
        this.inertiaInv = 1 / moment;
    }

    addShape(shape: Shape): void {
        this.shapeList.push(shape);
    }

    removeShape(shape: Shape): void {
        // This implementation has a linear time complexity with the number of shapes.
        // The original implementation used linked lists instead, which might be faster if
        // you're constantly editing the shape of a body. I expect most bodies will never
        // have their shape edited, so I'm just going to use the simplest possible implemention.
        deleteObjFromList(this.shapeList, shape);
    }

    removeConstraint(constraint: Constraint): void {
        // The constraint must be in the constraints list when this is called.
        this.constraintList = filterConstraints(this.constraintList, this, constraint);
    }

    setPos(pos: Vect): void {
        this.activate();
        this.sanityCheck();
        // If I allow the position to be set to vzero, vzero will get changed.
        if (pos === vzero) {
            pos = new Vect(0, 0);
        }
        this.p = pos;
    }

    setVel(vel: Vect): void {
        this.activate();
        this.vx = vel.x;
        this.vy = vel.y;
    }

    setAngVel(w: number): void {
        this.activate();
        this.w = w;
    }

    setAngleInternal(angle: number): void {
        assert(!isNaN(angle), "Internal Error: Attempting to set body's angle to NaN");
        this.a = angle; //fmod(a, (cpFloat)M_PI*2.0f);

        //this.rot = vforangle(angle);
        this.rot.x = Math.cos(angle);
        this.rot.y = Math.sin(angle);
    }

    setAngle(angle: number) {
        this.activate();
        this.sanityCheck();
        this.setAngleInternal(angle);
    }

    velocity_func(gravity: Vect, damping: number, dt: number): void {
        //this.v = vclamp(vadd(vmult(this.v, damping), vmult(vadd(gravity, vmult(this.f, this.m_inv)), dt)), this.v_limit);
        const vx = this.vx * damping + (gravity.x + this.f.x * this.massInv) * dt;
        const vy = this.vy * damping + (gravity.y + this.f.y * this.massInv) * dt;

        //var v = vclamp(new Vect(vx, vy), this.v_limit);
        //this.vx = v.x; this.vy = v.y;
        const v_limit = this.v_limit;
        const lensq = vx * vx + vy * vy;
        const scale = (lensq > v_limit * v_limit) ? v_limit / Math.sqrt(lensq) : 1;
        this.vx = vx * scale;
        this.vy = vy * scale;

        const w_limit = this.w_limit;
        this.w = clamp(this.w * damping + this.t * this.inertiaInv * dt, -w_limit, w_limit);

        this.sanityCheck();
    }

    position_func(dt: number): void {
        //this.p = vadd(this.p, vmult(vadd(this.v, this.v_bias), dt));

        //this.p = this.p + (this.v + this.v_bias) * dt;
        this.p.x += (this.vx + this.v_biasx) * dt;
        this.p.y += (this.vy + this.v_biasy) * dt;

        this.setAngleInternal(this.a + (this.w + this.w_bias) * dt);

        this.v_biasx = this.v_biasy = 0;
        this.w_bias = 0;

        this.sanityCheck();
    }

    resetForces(): void {
        this.activate();
        this.f = new Vect(0, 0);
        this.t = 0;
    }

    applyForce(force: Vect, r: Vect): void {
        this.activate();
        this.f = vadd(this.f, force);
        this.t += vcross(r, force);
    }

    applyImpulse(impulse: Vect, r: Vect) {
        this.activate();
        apply_impulse(this, impulse.x, impulse.y, r);
    }

    getVelAtPoint(r: Vect): Vect {
        return vadd(new Vect(this.vx, this.vy), vmult(vperp(r), this.w));
    }

    /// Get the velocity on a body (in world units) at a point on the body in world coordinates.
    getVelAtWorldPoint(point: Vect): Vect {
        return this.getVelAtPoint(vsub(point, this.p));
    }

    /// Get the velocity on a body (in world units) at a point on the body in local coordinates.
    getVelAtLocalPoint(point: Vect): Vect {
        return this.getVelAtPoint(vrotate(point, this.rot));
    }

    eachShape(func: (shape: Shape) => void) {
        for (let i = 0, len = this.shapeList.length; i < len; i++) {
            func(this.shapeList[i]);
        }
    }

    eachConstraint(func: (constraint: Constraint) => void) {
        let constraint = this.constraintList;
        while (constraint) {
            const next = constraint.next(this);
            func(constraint);
            constraint = next;
        }
    }

    eachArbiter(func: (arbiter: Arbiter) => void) {
        let arb = this.arbiterList;
        while (arb) {
            const next = arb.next(this);

            arb.swappedColl = (this === arb.body_b);
            func(arb);

            arb = next;
        }
    }

    /// Convert body relative/local coordinates to absolute/world coordinates.
    local2World(v: Vect): Vect {
        return vadd(this.p, vrotate(v, this.rot));
    }

    /// Convert body absolute/world coordinates to	relative/local coordinates.
    world2Local(v: Vect): Vect {
        return vunrotate(vsub(v, this.p), this.rot);
    }

    /// Get the kinetic energy of a body.
    kineticEnergy(): number {
        // Need to do some fudging to avoid NaNs
        const vsq = this.vx * this.vx + this.vy * this.vy;
        const wsq = this.w * this.w;
        return (vsq ? vsq * this.mass : 0) + (wsq ? wsq * this.inertia : 0);
    }

    sanityCheck(): void {
        assert(this.mass === this.mass && this.massInv === this.massInv, "Body's mass is invalid.");
        assert(this.inertia === this.inertia && this.inertiaInv === this.inertiaInv, "Body's moment is invalid.");

        v_assert_sane(this.p, "Body's position is invalid.");
        v_assert_sane(this.f, "Body's force is invalid.");
        assert(this.vx === this.vx && Math.abs(this.vx) !== Infinity, "Body's velocity is invalid.");
        assert(this.vy === this.vy && Math.abs(this.vy) !== Infinity, "Body's velocity is invalid.");

        assert(this.a === this.a && Math.abs(this.a) !== Infinity, "Body's angle is invalid.");
        assert(this.w === this.w && Math.abs(this.w) !== Infinity, "Body's angular velocity is invalid.");
        assert(this.t === this.t && Math.abs(this.t) !== Infinity, "Body's torque is invalid.");

        v_assert_sane(this.rot, "Body's rotation vector is invalid.");

        assert(this.v_limit === this.v_limit, "Body's velocity limit is invalid.");
        assert(this.w_limit === this.w_limit, "Body's angular velocity limit is invalid.");
    }

    activate(): void {
        if (!this.isRogue()) {
            this.nodeIdleTime = 0;
            componentActivate(componentRoot(this));
        }
    }

    activateStatic(filter: Shape): void {
        assert(this.isStatic(), "Body.activateStatic() called on a non-static body.");

        for (let arb = this.arbiterList; arb; arb = arb.next(this)) {
            if (!filter || filter == arb.a || filter == arb.b) {
                (arb.body_a == this ? arb.body_b : arb.body_a).activate();
            }
        }

        // TODO should also activate joints!
    }

    pushArbiter(arb: Arbiter): void {
        assertSoft((arb.body_a === this ? arb.thread_a_next : arb.thread_b_next) === null,
            "Internal Error: Dangling contact graph pointers detected. (A)");
        assertSoft((arb.body_a === this ? arb.thread_a_prev : arb.thread_b_prev) === null,
            "Internal Error: Dangling contact graph pointers detected. (B)");

        const next = this.arbiterList;
        assertSoft(next === null || (next.body_a === this ? next.thread_a_prev : next.thread_b_prev) === null,
            "Internal Error: Dangling contact graph pointers detected. (C)");

        if (arb.body_a === this) {
            arb.thread_a_next = next;
        } else {
            arb.thread_b_next = next;
        }

        if (next) {
            if (next.body_a === this) {
                next.thread_a_prev = arb;
            } else {
                next.thread_b_prev = arb;
            }
        }
        this.arbiterList = arb;
    }

    sleep(): void {
        this.sleepWithGroup(null);
    }

    sleepWithGroup(group: Body): void {
        assert(!this.isStatic() && !this.isRogue(), "Rogue and static bodies cannot be put to sleep.");

        const space = this.space;
        assert(space, "Cannot put a rogue body to sleep.");
        assert(!space.locked, "Bodies cannot be put to sleep during a query or a call to cpSpaceStep(). Put these calls into a post-step callback.");
        assert(group === null || group.isSleeping(), "Cannot use a non-sleeping body as a group identifier.");

        if (this.isSleeping()) {
            assert(componentRoot(this) === componentRoot(group), "The body is already sleeping and it's group cannot be reassigned.");
            return;
        }

        for (let i = 0; i < this.shapeList.length; i++) {
            this.shapeList[i].update(this.p, this.rot);
        }
        space.deactivateBody(this);

        if (group) {
            const root = componentRoot(group);

            this.nodeRoot = root;
            this.nodeNext = root.nodeNext;
            this.nodeIdleTime = 0;

            root.nodeNext = this;
        } else {
            this.nodeRoot = this;
            this.nodeNext = null;
            this.nodeIdleTime = 0;

            space.sleepingComponents.push(this);
        }

        deleteObjFromList(space.bodies, this);
    }
}

export function createStaticBody(): Body {
    const body = new Body(Infinity, Infinity);
    body.nodeIdleTime = Infinity;

    return body;
}

function v_assert_nan(v: Vect, message: string): void {
    assert(v.x == v.x && v.y == v.y, message);
}

function v_assert_infinite(v: Vect, message: string): void {
    assert(Math.abs(v.x) !== Infinity && Math.abs(v.y) !== Infinity, message);
}

function v_assert_sane(v: Vect, message: string): void {
     v_assert_nan(v, message); v_assert_infinite(v, message);
    }
