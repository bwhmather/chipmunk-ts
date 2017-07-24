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

import { Body } from "./body";
import { Contact } from "./collision";
import {
    apply_bias_impulse, apply_impulses,
    k_scalar, normal_relative_velocity,
} from "./constraints/util";
import { Shape } from "./shapes";
import { Space } from "./space";
import { clamp } from "./util";
import { vdot2, Vect, vmult, vneg, vperp, vsub, vzero } from "./vect";

/// @defgroup cpArbiter cpArbiter
/// The cpArbiter struct controls pairs of colliding shapes.
/// They are also used in conjuction with collision handler callbacks
/// allowing you to retrieve information on the collision and control it.

const CP_MAX_CONTACTS_PER_ARBITER = 4;

/// A struct that wraps up the important collision data for an arbiter.
export class ContactPoint {
    point: Vect;
    normal: Vect;
    dist: number;

    constructor(point: Vect, normal: Vect, dist: number) {
        this.point = point;
        this.normal = normal;
        this.dist = dist;
    }
}

// **** Collision Handlers
//
// Collision handlers are user-defined objects to describe the behaviour of
// colliding objects.
export class CollisionHandler {
    a: number;
    b: number;

    constructor() {
        // The collision type
        this.a = this.b = 0;
    }

    /// Collision begin event callback
    /// Returning false from a begin callback causes the collision to be ignored
    /// until the the separate callback is called when the objects stop colliding.
    begin(arb: Arbiter, space: Space) {
        return true;
    }

    /// Collision pre-solve event callback
    /// Returning false from a pre-step callback causes the collision to be ignored
    /// until the next step.
    preSolve(arb: Arbiter, space: Space) {
        return true;
    }

    /// Collision post-solve event function callback type.
    postSolve(arb: Arbiter, space: Space) { }

    /// Collision separate event function callback type.
    separate(arb: Arbiter, space: Space) { }
}

type ArbiterState = (
    // Arbiter is active and its the first collision.
    "first-coll" |
    // Arbiter is active and its not the first collision.
    "normal" |
    // Collision has been explicitly ignored, either by returning false from a
    // begin collision handler or calling cpArbiterIgnore()
    "ignore" |
    // Collison is no longer active. A space will cache an arbiter for up to
    // cpSpace.collisionPersistence more steps.
    "cached"
);

/// A colliding pair of shapes.
export class Arbiter {
    /// Calculated value to use for the elasticity coefficient.
    /// Override in a pre-solve collision handler for custom behavior.
    e: number = 0;

    /// Calculated value to use for the friction coefficient.
    /// Override in a pre-solve collision handler for custom behavior.
    u: number = 0;

    /// Calculated value to use for applying surface velocities.
    /// Override in a pre-solve collision handler for custom behavior.
    surface_vr: Vect = vzero;

    a: Shape;
    body_a: Body;
    b: Shape;
    body_b: Body;

    thread_a_next: Arbiter;
    thread_a_prev: Arbiter;
    thread_b_next: Arbiter;
    thread_b_prev: Arbiter;

    contacts: Contact[];
    stamp: number;
    handler: CollisionHandler;
    swappedColl: boolean;
    state: ArbiterState;

    constructor(a: Shape, b: Shape) {
        this.a = a; this.body_a = a.body;
        this.b = b; this.body_b = b.body;

        this.thread_a_next = this.thread_a_prev = null;
        this.thread_b_next = this.thread_b_prev = null;

        this.contacts = null;

        this.stamp = 0;
        this.handler = null;
        this.swappedColl = false;
        this.state = "first-coll";
    }

    getShapes(): [Shape, Shape] {
        if (this.swappedColl) {
            return [this.b, this.a];
        } else {
            return [this.a, this.b];
        }
    }

    /// Calculate the total impulse that was applied by this arbiter.
    /// This function should only be called from a post-solve, post-step or
    /// cpBodyEachArbiter callback.
    totalImpulse(): Vect {
        const contacts = this.contacts;
        const sum = new Vect(0, 0);

        for (let i = 0, count = contacts.length; i < count; i++) {
            const con = contacts[i];
            sum.add(vmult(con.n, con.jnAcc));
        }

        return this.swappedColl ? sum : sum.neg();
    }

    /// Calculate the total impulse including the friction that was applied by
    /// this arbiter.
    /// This function should only be called from a post-solve, post-step or
    /// cpBodyEachArbiter callback.
    totalImpulseWithFriction(): Vect {
        const contacts = this.contacts;
        const sum = new Vect(0, 0);

        for (let i = 0, count = contacts.length; i < count; i++) {
            const con = contacts[i];
            sum.add(new Vect(con.jnAcc, con.jtAcc).rotate(con.n));
        }

        return this.swappedColl ? sum : sum.neg();
    }

    /// Calculate the amount of energy lost in a collision including static,
    /// but not dynamic friction.
    /// This function should only be called from a post-solve, post-step or
    /// cpBodyEachArbiter callback.
    totalKE(): number {
        const eCoef = (1 - this.e) / (1 + this.e);
        let sum = 0;

        const contacts = this.contacts;
        for (let i = 0, count = contacts.length; i < count; i++) {
            const con = contacts[i];
            const jnAcc = con.jnAcc;
            const jtAcc = con.jtAcc;

            sum += eCoef * jnAcc * jnAcc / con.nMass + jtAcc * jtAcc / con.tMass;
        }

        return sum;
    }

    /// Causes a collision pair to be ignored as if you returned false from a
    /// begin callback.
    /// If called from a pre-step callback, you will still need to return false
    /// if you want it to be ignored in the current step.
    ignore() {
        this.state = "ignore";
    }

    /// Return the colliding shapes involved for this arbiter.
    /// The order of their cpSpace.collision_type values will match
    /// the order set when the collision handler was registered.
    getA(): Shape {
        return this.swappedColl ? this.b : this.a;
    }

    getB(): Shape {
        return this.swappedColl ? this.a : this.b;
    }

    /// Returns true if this is the first step a pair of objects started
    /// colliding.
    isFirstContact(): boolean {
        return this.state === "first-coll";
    }

    /// Return a contact set from an arbiter.
    getContactPointSet(): ContactPoint[] {
        const set = new Array(this.contacts.length);

        let i;
        for (i = 0; i < set.length; i++) {
            set[i] = new ContactPoint(
                this.contacts[i].p, this.contacts[i].n, this.contacts[i].dist,
            );
        }

        return set;
    }

    /// Get the normal of the @c ith contact point.
    getNormal(i: number): Vect {
        const n = this.contacts[i].n;
        return this.swappedColl ? vneg(n) : n;
    }

    /// Get the position of the @c ith contact point.
    getPoint(i: number): Vect {
        return this.contacts[i].p;
    }

    /// Get the depth of the @c ith contact point.
    getDepth(i: number): number {
        return this.contacts[i].dist;
    }

    unthread(): void {
        unthreadHelper(this, this.body_a, this.thread_a_prev, this.thread_a_next);
        unthreadHelper(this, this.body_b, this.thread_b_prev, this.thread_b_next);
        this.thread_a_prev = this.thread_a_next = null;
        this.thread_b_prev = this.thread_b_next = null;
    }

    update(
        contacts: Contact[], handler: CollisionHandler, a: Shape, b: Shape,
    ): void {
        // Arbiters without contact data may exist if a collision function rejected the collision.
        if (this.contacts) {
            // Iterate over the possible pairs to look for hash value matches.
            for (const old of this.contacts) {
                for (const new_contact of contacts) {
                    // This could trigger false positives, but is fairly unlikely nor serious if it does.
                    if (new_contact.hash === old.hash) {
                        // Copy the persistant contact information.
                        new_contact.jnAcc = old.jnAcc;
                        new_contact.jtAcc = old.jtAcc;
                    }
                }
            }
        }

        this.contacts = contacts;

        this.handler = handler;
        this.swappedColl = (a.collisionType !== handler.a);

        this.e = a.restitutionCoef * b.restitutionCoef;
        this.u = a.frictionCoef * b.frictionCoef;
        this.surface_vr = vsub(a.surfaceVelocity, b.surfaceVelocity);

        // For collisions between two similar primitive types, the order could have been swapped.
        this.a = a; this.body_a = a.body;
        this.b = b; this.body_b = b.body;

        // mark it as new if it's been cached
        if (this.state == "cached") this.state = "first-coll";
    }

    preStep(dt: number, slop: number, bias: number): void {
        const a = this.body_a;
        const b = this.body_b;

        for (const con of this.contacts) {
            // Calculate the offsets.
            con.r1 = vsub(con.p, a.p);
            con.r2 = vsub(con.p, b.p);

            // Calculate the mass normal and mass tangent.
            con.nMass = 1 / k_scalar(a, b, con.r1, con.r2, con.n);
            con.tMass = 1 / k_scalar(a, b, con.r1, con.r2, vperp(con.n));

            // Calculate the target bias velocity.
            con.bias = -bias * Math.min(0, con.dist + slop) / dt;
            con.jBias = 0;

            // Calculate the target bounce velocity.
            con.bounce = normal_relative_velocity(
                a, b, con.r1, con.r2, con.n,
            ) * this.e;
        }
    }

    applyCachedImpulse(dt_coef: number): void {
        if (this.isFirstContact()) return;

        const a = this.body_a;
        const b = this.body_b;

        for (const con of this.contacts) {
            //var j = vrotate(con.n, new Vect(con.jnAcc, con.jtAcc));
            const nx = con.n.x;
            const ny = con.n.y;
            const jx = nx * con.jnAcc - ny * con.jtAcc;
            const jy = nx * con.jtAcc + ny * con.jnAcc;
            //apply_impulses(a, b, con.r1, con.r2, vmult(j, dt_coef));
            apply_impulses(a, b, con.r1, con.r2, jx * dt_coef, jy * dt_coef);
        }
    }

    applyImpulse() {
        numApplyImpulse++;
        //if (!this.contacts) { throw new Error('contacts is undefined'); }
        const a = this.body_a;
        const b = this.body_b;
        const surface_vr = this.surface_vr;
        const friction = this.u;

        this.contacts.forEach((con: Contact, i: number) => {
            numApplyContact++;
            const nMass = con.nMass;
            const n = con.n;
            const r1 = con.r1;
            const r2 = con.r2;

            //var vr = relative_velocity(a, b, r1, r2);
            const vrx = b.vx - r2.y * b.w - (a.vx - r1.y * a.w);
            const vry = b.vy + r2.x * b.w - (a.vy + r1.x * a.w);

            //var vb1 = vadd(vmult(vperp(r1), a.w_bias), a.v_bias);
            //var vb2 = vadd(vmult(vperp(r2), b.w_bias), b.v_bias);
            //var vbn = vdot(vsub(vb2, vb1), n);

            const vbn = (
                n.x * (
                    b.v_biasx - r2.y * b.w_bias -
                    a.v_biasx + r1.y * a.w_bias
                ) +
                n.y * (
                    r2.x * b.w_bias + b.v_biasy -
                    r1.x * a.w_bias - a.v_biasy
                )
            );

            const vrn = vdot2(vrx, vry, n.x, n.y);
            //var vrt = vdot(vadd(vr, surface_vr), vperp(n));
            const vrt = vdot2(
                vrx + surface_vr.x, vry + surface_vr.y,
                -n.y, n.x,
            );

            const jbn = (con.bias - vbn) * nMass;
            const jbnOld = con.jBias;
            con.jBias = Math.max(jbnOld + jbn, 0);

            const jn = -(con.bounce + vrn) * nMass;
            const jnOld = con.jnAcc;
            con.jnAcc = Math.max(jnOld + jn, 0);

            const jtMax = friction * con.jnAcc;
            const jt = -vrt * con.tMass;
            const jtOld = con.jtAcc;
            con.jtAcc = clamp(jtOld + jt, -jtMax, jtMax);

            //apply_bias_impulses(a, b, r1, r2, vmult(n, con.jBias - jbnOld));
            const bias_x = n.x * (con.jBias - jbnOld);
            const bias_y = n.y * (con.jBias - jbnOld);
            apply_bias_impulse(a, -bias_x, -bias_y, r1);
            apply_bias_impulse(b, bias_x, bias_y, r2);

            //apply_impulses(a, b, r1, r2, vrotate(n, new Vect(con.jnAcc - jnOld, con.jtAcc - jtOld)));
            const rot_x = con.jnAcc - jnOld;
            const rot_y = con.jtAcc - jtOld;

            // Inlining apply_impulses decreases speed for some reason :/
            apply_impulses(
                a, b, r1, r2,
                n.x * rot_x - n.y * rot_y,
                n.x * rot_y + n.y * rot_x,
            );
        });
    }

    callSeparate(space: Space) {
        // The handler needs to be looked up again as the handler cached on the
        // arbiter may have been deleted since the last step.
        const handler = space.lookupHandler(
            this.a.collisionType, this.b.collisionType,
        );
        handler.separate(this, space);
    }

    // From chipmunk_private.h
    next(body: Body) {
        return (this.body_a == body ? this.thread_a_next : this.thread_b_next);
    }
}

/*
Arbiter.prototype.threadForBody = function(body)
{
	return (this.body_a === body ? this.thread_a : this.thread_b);
};*/

function unthreadHelper(arb: Arbiter, body: Body, prev: Arbiter, next: Arbiter) {
    // thread_x_y is quite ugly, but it avoids making unnecessary js objects per
    // arbiter.
    if (prev) {
        // cpArbiterThreadForBody(prev, body)->next = next;
        if (prev.body_a === body) {
            prev.thread_a_next = next;
        } else {
            prev.thread_b_next = next;
        }
    } else if (body.arbiterList === arb) {
        body.arbiterList = next;
    }

    if (next) {
        // cpArbiterThreadForBody(next, body)->prev = prev;
        if (next.body_a === body) {
            next.thread_a_prev = prev;
        } else {
            next.thread_b_prev = prev;
        }
    }
}

// TODO is it worth splitting velocity/position correction?

let numApplyImpulse = 0;
let numApplyContact = 0;
