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
import { Constraint } from "./constraint";

function defaultSpringTorque(
    spring: DampedRotarySpring, relativeAngle: number,
): number {
    return (relativeAngle - spring.restAngle) * spring.stiffness;
}

export class DampedRotarySpring extends Constraint {
    restAngle: number;
    stiffness: number;
    damping: number;

    springTorqueFunc: (
        spring: DampedRotarySpring, relativeAngle: number,
    ) => number;

    targetNormalRelativeRate: number;
    dragCoef: number;
    iSum: number;

    constructor(
        a: Body, b: Body, restAngle: number, stiffness: number, damping: number,
    ) {
        super(a, b);

        this.restAngle = restAngle;
        this.stiffness = stiffness;
        this.damping = damping;
        this.springTorqueFunc = defaultSpringTorque;

        this.targetNormalRelativeRate = 0;
        this.dragCoef = 0;
        this.iSum = 0;
    }

    preStep(dt: number): void {
        const a = this.a;
        const b = this.b;

        const moment = a.inertiaInv + b.inertiaInv;
        assertSoft(moment !== 0, "Unsolvable spring.");
        this.iSum = 1 / moment;

        this.dragCoef = 1 - Math.exp(-this.damping * dt * moment);
        this.targetNormalRelativeRate = 0;

        // apply this torque
        const springTorque = this.springTorqueFunc(this, a.a - b.a) * dt;
        a.w -= springTorque * a.inertiaInv;
        b.w += springTorque * b.inertiaInv;
    }

    applyImpulse(): void {
        const a = this.a;
        const b = this.b;

        // compute relative velocity
        // normal_relative_velocity(a, b, r1, r2, n) - this.target_vrn;
        const normalRelativeRate = a.w - b.w;

        // compute velocity loss from drag
        // not 100% certain spring is derived correctly, though it makes sense
        const rateDamped = (this.targetNormalRelativeRate - normalRelativeRate) * this.dragCoef;
        this.targetNormalRelativeRate = normalRelativeRate + rateDamped;

        // apply_impulses(
        //     a, b, this.r1, this.r2, vmult(this.n, v_damp*this.nMass),
        // );
        const torqueDamped = rateDamped * this.iSum;
        a.w += torqueDamped * a.inertiaInv;
        b.w -= torqueDamped * b.inertiaInv;
    }
}
