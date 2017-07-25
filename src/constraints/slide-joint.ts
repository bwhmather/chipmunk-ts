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
import { clamp } from "../util";
import {
    vadd, vdot,
    Vect, vlength,
    vneg, vnormalize_safe,
    vrotate, vsub,
    vzero,
} from "../vect";
import { Constraint } from "./constraint";
import {
    applyImpulses,
    biasCoef,
    kScalar,
    relativeVelocity,
} from "./util";

export class SlideJoint extends Constraint {
    anchr1: Vect;
    anchr2: Vect;
    min: number;
    max: number;

    r1: Vect;
    r2: Vect;
    n: Vect;
    nMass: number;

    jnAcc: number;
    jnMax: number;

    bias: number;

    constructor(
        a: Body, b: Body,
        anchr1: Vect, anchr2: Vect,
        min: number, max: number,
    ) {
        super(a, b);

        this.anchr1 = anchr1;
        this.anchr2 = anchr2;
        this.min = min;
        this.max = max;

        this.r1 = this.r2 = this.n = null;
        this.nMass = 0;

        this.jnAcc = this.jnMax = 0;
        this.bias = 0;
    }

    preStep(dt: number): void {
        const a = this.a;
        const b = this.b;

        this.r1 = vrotate(this.anchr1, a.rot);
        this.r2 = vrotate(this.anchr2, b.rot);

        const delta = vsub(vadd(b.p, this.r2), vadd(a.p, this.r1));
        const dist = vlength(delta);
        let pdist = 0;
        if (dist > this.max) {
            pdist = dist - this.max;
            this.n = vnormalize_safe(delta);
        } else if (dist < this.min) {
            pdist = this.min - dist;
            this.n = vneg(vnormalize_safe(delta));
        } else {
            this.n = vzero;
            this.jnAcc = 0;
        }

        // calculate mass normal
        this.nMass = 1 / kScalar(a, b, this.r1, this.r2, this.n);

        // calculate bias velocity
        const maxBias = this.maxBias;
        this.bias = clamp(
            -biasCoef(this.errorBias, dt) * pdist / dt,
            -maxBias, maxBias,
        );

        // compute max impulse
        this.jnMax = this.maxForce * dt;
    }

    applyCachedImpulse(dtCoef: number): void {
        const jn = this.jnAcc * dtCoef;
        applyImpulses(
            this.a, this.b,
            this.r1, this.r2,
            this.n.x * jn, this.n.y * jn,
        );
    }

    applyImpulse(): void {
        if (this.n.x === 0 && this.n.y === 0) {
            return;  // early exit
        }

        const a = this.a;
        const b = this.b;

        const n = this.n;
        const r1 = this.r1;
        const r2 = this.r2;

        // compute relative velocity
        const vr = relativeVelocity(a, b, r1, r2);
        const vrn = vdot(vr, n);

        // compute normal impulse
        let jn = (this.bias - vrn) * this.nMass;
        const jnOld = this.jnAcc;
        this.jnAcc = clamp(jnOld + jn, -this.jnMax, 0);
        jn = this.jnAcc - jnOld;

        // apply impulse
        applyImpulses(a, b, this.r1, this.r2, n.x * jn, n.y * jn);
    }

    getImpulse(): number {
        return Math.abs(this.jnAcc);
    }
}
