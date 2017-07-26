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
/// Chipmunk's rigid body type. Rigid bodies hold the physical properties of an
/// object like it's mass, and position and velocity of it's center of gravity.
/// They don't have an shape on their own.  They are given a shape by creating
/// collision shapes (cpShape) that point to the body.

import { Arbiter } from "./arbiter";
import { Constraint } from "./constraints";
import { applyImpulse } from "./constraints/util";
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
    p: Vect = new Vect(0, 0);
    /// Velocity of the rigid body's center of gravity.
    vx: number = 0;
    vy: number = 0;
    /// Force acting on the rigid body's center of gravity.
    f: Vect = new Vect(0, 0);

    /// Rotation of the body around it's center of gravity in radians.
    /// Must agree with cpBody.rot! Use cpBodySetAngle() when changing the angle
    /// for this reason.
    a: number = 0;
    /// Angular velocity of the body around it's center of gravity in radians
    /// per second.
    w: number = 0;
    /// Torque applied to the body around it's center of gravity.
    t: number = 0;

    /// Cached unit length vector representing the angle of the body.
    /// Used for fast rotations using cpvrotate().
    rot: Vect = new Vect(0, 0);

    /// Maximum velocity allowed when updating the velocity.
    vLimit: number = Infinity;
    /// Maximum rotational rate (in radians/second) allowed when updating the
    /// angular velocity.
    wLimit: number = Infinity;

    // This stuff is all private.
    vyBias: number = 0;
    vxBias: number = 0;
    wBias: number = 0;

    space: Space = null;

    shapeList: Shape[] = [];
    // These are both wacky linked lists.
    arbiterList: Arbiter = null;
    constraintList: Constraint = null;

    // This stuff is used to track information on the collision graph.
    // TODO
    nodeRoot: Body = null;
    nodeNext: Body = null;
    nodeIdleTime: number = 0;

    // Mass and one-over-mass.
    mass: number;
    massInv: number;

    // Inertia and one over inertia.
    inertia: number;
    inertiaInv: number;

    constructor(m: number, i: number) {
        // Set this.m and this.m_inv
        this.setMass(m);

        // Set this.i and this.i_inv
        this.setMoment(i);
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

    // It would be nicer to use defineProperty for this, but its about 30x
    // slower: http://jsperf.com/defineproperty-vs-setter
    setMass(mass: number): void {
        assert(mass > 0, "Mass must be positive and non-zero.");

        // activate is defined in cpSpaceComponent
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
        // This implementation has a linear time complexity with the number of
        // shapes. The original implementation used linked lists instead, which
        // might be faster if you're constantly editing the shape of a body
        // I expect most bodies will never have their shape edited, so I'm just
        // going to use the simplest possible implemention.
        deleteObjFromList(this.shapeList, shape);
    }

    removeConstraint(constraint: Constraint): void {
        // The constraint must be in the constraints list when this is called.
        this.constraintList = filterConstraints(
            this.constraintList, this, constraint,
        );
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
        assert(
            !isNaN(angle),
            "Internal Error: Attempting to set body's angle to NaN",
        );
        this.a = angle; // fmod(a, (cpFloat)M_PI*2.0f);

        // this.rot = vforangle(angle);
        this.rot = new Vect(Math.cos(angle), Math.sin(angle));
    }

    setAngle(angle: number) {
        this.activate();
        this.sanityCheck();
        this.setAngleInternal(angle);
    }

    velocity_func(gravity: Vect, damping: number, dt: number): void {
        // this.v = vclamp(vadd(
        //     vmult(this.v, damping),
        //     vmult(vadd(gravity, vmult(this.f, this.m_inv)), dt)
        // ), this.v_limit);
        const vx = (
            this.vx * damping +
            (gravity.x + this.f.x * this.massInv) * dt
        );
        const vy = (
            this.vy * damping +
            (gravity.y + this.f.y * this.massInv) * dt
        );

        // var v = vclamp(new Vect(vx, vy), this.v_limit);
        // this.vx = v.x; this.vy = v.y;
        const vLimit = this.vLimit;
        const lensq = vx * vx + vy * vy;
        const scale = ((lensq > vLimit * vLimit)
            ? vLimit / Math.sqrt(lensq)
            : 1
        );
        this.vx = vx * scale;
        this.vy = vy * scale;

        const wLimit = this.wLimit;
        this.w = clamp(
            this.w * damping + this.t * this.inertiaInv * dt,
            -wLimit, wLimit,
        );

        this.sanityCheck();
    }

    position_func(dt: number): void {
        // this.p = vadd(this.p, vmult(vadd(this.v, this.v_bias), dt));

        // this.p = this.p + (this.v + this.v_bias) * dt;
        this.p = vadd(
            this.p,
            new Vect(
                (this.vx + this.vxBias) * dt,
                (this.vy + this.vyBias) * dt,
            ),
        );

        this.setAngleInternal(this.a + (this.w + this.wBias) * dt);

        this.vxBias = this.vyBias = 0;
        this.wBias = 0;

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
        applyImpulse(this, impulse.x, impulse.y, r);
    }

    getVelAtPoint(r: Vect): Vect {
        return vadd(new Vect(this.vx, this.vy), vmult(vperp(r), this.w));
    }

    /// Get the velocity on a body (in world units) at a point on the body in
    // world coordinates.
    getVelAtWorldPoint(point: Vect): Vect {
        return this.getVelAtPoint(vsub(point, this.p));
    }

    /// Get the velocity on a body (in world units) at a point on the body in
    // local coordinates.
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

            arb.swappedColl = (this === arb.bodyB);
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
        assert(
            this.mass === this.mass && this.massInv === this.massInv,
            "Body's mass is invalid.",
        );
        assert(
            (
                this.inertia === this.inertia &&
                this.inertiaInv === this.inertiaInv
            ),
            "Body's moment is invalid.",
        );

        v_assert_sane(this.p, "Body's position is invalid.");
        v_assert_sane(this.f, "Body's force is invalid.");
        assert(
            this.vx === this.vx && Math.abs(this.vx) !== Infinity,
            "Body's velocity is invalid.",
        );
        assert(
            this.vy === this.vy && Math.abs(this.vy) !== Infinity,
            "Body's velocity is invalid.",
        );

        assert(
            this.a === this.a && Math.abs(this.a) !== Infinity,
            "Body's angle is invalid.",
        );
        assert(
            this.w === this.w && Math.abs(this.w) !== Infinity,
            "Body's angular velocity is invalid.",
        );
        assert(
            this.t === this.t && Math.abs(this.t) !== Infinity,
            "Body's torque is invalid.",
        );

        v_assert_sane(this.rot, "Body's rotation vector is invalid.");

        assert(
            this.vLimit === this.vLimit,
            "Body's velocity limit is invalid.",
        );
        assert(
            this.wLimit === this.wLimit,
            "Body's angular velocity limit is invalid.",
        );
    }

    activate(): void {
        if (!this.isRogue()) {
            this.nodeIdleTime = 0;
            componentActivate(componentRoot(this));
        }
    }

    activateStatic(filter: Shape): void {
        assert(
            this.isStatic(),
            "Body.activateStatic() called on a non-static body.",
        );

        for (let arb = this.arbiterList; arb; arb = arb.next(this)) {
            if (!filter || filter === arb.shapeA || filter === arb.shapeB) {
                (arb.bodyA === this ? arb.bodyB : arb.bodyA).activate();
            }
        }

        // TODO should also activate joints!
    }

    pushArbiter(arb: Arbiter): void {
        assertSoft(
            (
                arb.bodyA === this ? arb.threadNextA : arb.threadNextB
            ) === null,
            "Internal Error: Dangling contact graph pointers detected. (A)",
        );
        assertSoft(
            (
                arb.bodyA === this ? arb.threadPrevA : arb.threadPrevB
            ) === null,
            "Internal Error: Dangling contact graph pointers detected. (B)",
        );

        const next = this.arbiterList;
        assertSoft(
            (
                next === null ||
                (
                    next.bodyA === this
                        ? next.threadPrevA
                        : next.threadPrevB
                ) === null
            ),
            "Internal Error: Dangling contact graph pointers detected. (C)",
        );

        if (arb.bodyA === this) {
            arb.threadNextA = next;
        } else {
            arb.threadNextB = next;
        }

        if (next) {
            if (next.bodyA === this) {
                next.threadPrevA = arb;
            } else {
                next.threadPrevB = arb;
            }
        }
        this.arbiterList = arb;
    }

    sleep(): void {
        this.sleepWithGroup(null);
    }

    sleepWithGroup(group: Body): void {
        assert(
            !this.isStatic() && !this.isRogue(),
            "Rogue and static bodies cannot be put to sleep.",
        );

        const space = this.space;
        assert(space, "Cannot put a rogue body to sleep.");
        assert(
            !space.locked,
            "Bodies cannot be put to sleep during a query or a call to " +
            "cpSpaceStep(). Put these calls into a post-step callback.",
        );
        assert(
            group === null || group.isSleeping(),
            "Cannot use a non-sleeping body as a group identifier.",
        );

        if (this.isSleeping()) {
            assert(
                componentRoot(this) === componentRoot(group),
                "The body is already sleeping and it's group cannot be " +
                "reassigned.",
            );
            return;
        }

        for (const shape of this.shapeList) {
            shape.update(this.p, this.rot);
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
    assert(v.x === v.x && v.y === v.y, message);
}

function v_assert_infinite(v: Vect, message: string): void {
    assert(Math.abs(v.x) !== Infinity && Math.abs(v.y) !== Infinity, message);
}

function v_assert_sane(v: Vect, message: string): void {
    v_assert_nan(v, message); v_assert_infinite(v, message);
}
