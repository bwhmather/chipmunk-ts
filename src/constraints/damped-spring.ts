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

import { Body } from "../body";
import { assertSoft } from "../util";
import {
    vadd,
    Vect, vlength, vmult,
    vrotate, vsub,
} from "../vect";
import { Constraint } from "./constraint";
import {
    applyImpulses, kScalar,
    normalRelativeVelocity,
} from "./util";

function defaultSpringForce(spring: DampedSpring, dist: number): number {
    return (spring.restLength - dist) * spring.stiffness;
}

export class DampedSpring extends Constraint {
    anchr1: Vect;
    anchr2: Vect;

    restLength: number;
    stiffness: number;
    damping: number;

    // TODO
    springForceFunc: (
        spring: DampedSpring, dist: number,
    ) => number;

    targetNormalRelativeVelocity: number;
    dragCoef: number;

    r1: Vect;
    r2: Vect;

    nMass: number;
    n: Vect;

    constructor(
        a: Body, b: Body,
        anchr1: Vect, anchr2: Vect,
        restLength: number,
        stiffness: number,
        damping: number,
    ) {
        super(a, b);

        this.anchr1 = anchr1;
        this.anchr2 = anchr2;

        this.restLength = restLength;
        this.stiffness = stiffness;
        this.damping = damping;
        this.springForceFunc = defaultSpringForce;

        this.targetNormalRelativeVelocity = this.dragCoef = 0;

        this.r1 = this.r2 = null;
        this.nMass = 0;
        this.n = null;
    }

    preStep(dt: number): void {
        const a = this.a;
        const b = this.b;

        this.r1 = vrotate(this.anchr1, a.rot);
        this.r2 = vrotate(this.anchr2, b.rot);

        const delta = vsub(vadd(b.p, this.r2), vadd(a.p, this.r1));
        const dist = vlength(delta);
        this.n = vmult(delta, 1 / (dist ? dist : Infinity));

        const k = kScalar(a, b, this.r1, this.r2, this.n);
        assertSoft(k !== 0, "Unsolvable this.");
        this.nMass = 1 / k;

        this.targetNormalRelativeVelocity = 0;
        this.dragCoef = 1 - Math.exp(-this.damping * dt * k);

        // apply this force
        const springForce = this.springForceFunc(this, dist);
        applyImpulses(
            a, b, this.r1, this.r2,
            this.n.x * springForce * dt, this.n.y * springForce * dt,
        );
    }

    applyCachedImpulse(dtCoef: number): void {
        // pass
    }

    applyImpulse(): void {
        // compute relative velocity
        const nrv = normalRelativeVelocity(
            this.a, this.b, this.r1,
            this.r2, this.n,
        );

        // compute velocity loss from drag
        let vDamped = (
            this.targetNormalRelativeVelocity -
            nrv
        ) * this.dragCoef;
        this.targetNormalRelativeVelocity = nrv + vDamped;

        vDamped *= this.nMass;
        applyImpulses(
            this.a, this.b, this.r1, this.r2,
            this.n.x * vDamped, this.n.y * vDamped,
        );
    }

    getImpulse(): number {
        return 0;
    }
}
