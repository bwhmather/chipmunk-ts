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

import { Constraint } from './constraint';
import { clamp } from '../util';
import { Body } from '../body';


export class SimpleMotor extends Constraint {
    rate: number;
    jAcc: number;
    iSum: number;
    jMax: number;

    constructor(a: Body, b: Body, rate: number) {
        super(a, b);

        this.rate = rate;

        this.jAcc = 0;

        this.iSum = this.jMax = 0;
    }

    preStep(dt: number): void {
        // calculate moment of inertia coefficient.
        this.iSum = 1 / (this.a.i_inv + this.b.i_inv);

        // compute max impulse
        this.jMax = this.maxForce * dt;
    }

    applyCachedImpulse(dt_coef: number): void {
        const a = this.a;
        const b = this.b;

        const j = this.jAcc * dt_coef;
        a.w -= j * a.i_inv;
        b.w += j * b.i_inv;
    }

    applyImpulse(): void {
        const a = this.a;
        const b = this.b;

        // compute relative rotational velocity
        const wr = b.w - a.w + this.rate;

        // compute normal impulse	
        let j = -wr * this.iSum;
        const jOld = this.jAcc;
        this.jAcc = clamp(jOld + j, -this.jMax, this.jMax);
        j = this.jAcc - jOld;

        // apply impulse
        a.w -= j * a.i_inv;
        b.w += j * b.i_inv;
    }

    getImpulse(): number {
        return Math.abs(this.jAcc);
    }
}

