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

import { Body } from '../body';
import { Shape } from '../shapes';
import { Space } from '../space';

// TODO: Comment me!

// a and b are bodies that the constraint applies to.
export abstract class Constraint {
    a: Body;
    b: Body;

    space: Space;

    next_a: Constraint;
    next_b: Constraint;

    maxForce: number;

    errorBias: number;
    maxBias: number;

    constructor(a: Body, b: Body) {
        /// The first body connected to this constraint.
        this.a = a;
        /// The second body connected to this constraint.
        this.b = b;

        this.space = null;

        this.next_a = null;
        this.next_b = null;

        /// The maximum force that this constraint is allowed to use.
        this.maxForce = Infinity;
        /// The rate at which joint error is corrected.
        /// Defaults to pow(1 - 0.1, 60) meaning that it will
        /// correct 10% of the error every 1/60th of a second.
        this.errorBias = (1 - 0.1) ** 60;
        /// The maximum rate at which joint error is corrected.
        this.maxBias = Infinity;
    }

    activateBodies(): void {
        if (this.a) this.a.activate();
        if (this.b) this.b.activate();
    }

    /// These methods are overridden by the constraint itself.
    preStep(dt: number): void { }

    applyCachedImpulse(dt_coef: number): void { }
    applyImpulse(): void { }

    getImpulse(): number {
        return 0;
    }

    /// Function called before the solver runs. This can be overridden by the user
    /// to customize the constraint.
    /// Animate your joint anchors, update your motor torque, etc.
    preSolve(space: Space) { }

    /// Function called after the solver runs. This can be overridden by the user
    /// to customize the constraint.
    /// Use the applied impulse to perform effects like breakable joints.
    postSolve(space: Space) { }

    next(body: Body) {
        return (this.a === body ? this.next_a : this.next_b);
    }
}

