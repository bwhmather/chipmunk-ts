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
import { Constraint } from './constraint';
import {
    apply_impulses, normal_relative_velocity,
    k_scalar,
} from './util';
import { assertSoft } from '../util';
import {
    Vect,
    vadd, vsub, vmult,
    vlength, vrotate,
} from '../vect';

function defaultSpringForce(spring, dist) {
    return (spring.restLength - dist) * spring.stiffness;
};



export class DampedSpring extends Constraint {
    anchr1: Vect;
    anchr2: Vect;

    restLength: number;
    stiffness: number;
    damping: number;

    // TODO
    springForceFunc;

    target_vrn: number;
    v_coef: number;

    r1: Vect;
    r2: Vect;

    nMass: number;
    n: Vect;

    constructor(a, b, anchr1, anchr2, restLength, stiffness, damping) {
        super(a, b);

        this.anchr1 = anchr1;
        this.anchr2 = anchr2;

        this.restLength = restLength;
        this.stiffness = stiffness;
        this.damping = damping;
        this.springForceFunc = defaultSpringForce;

        this.target_vrn = this.v_coef = 0;

        this.r1 = this.r2 = null;
        this.nMass = 0;
        this.n = null;
    }

    preStep(dt) {
        const a = this.a;
        const b = this.b;

        this.r1 = vrotate(this.anchr1, a.rot);
        this.r2 = vrotate(this.anchr2, b.rot);

        const delta = vsub(vadd(b.p, this.r2), vadd(a.p, this.r1));
        const dist = vlength(delta);
        this.n = vmult(delta, 1 / (dist ? dist : Infinity));

        const k = k_scalar(a, b, this.r1, this.r2, this.n);
        assertSoft(k !== 0, "Unsolvable this.");
        this.nMass = 1 / k;

        this.target_vrn = 0;
        this.v_coef = 1 - Math.exp(-this.damping * dt * k);

        // apply this force
        const f_spring = this.springForceFunc(this, dist);
        apply_impulses(a, b, this.r1, this.r2, this.n.x * f_spring * dt, this.n.y * f_spring * dt);
    }

    applyCachedImpulse(dt_coef) { }

    applyImpulse() {
        const a = this.a;
        const b = this.b;

        const n = this.n;
        const r1 = this.r1;
        const r2 = this.r2;

        // compute relative velocity
        const vrn = normal_relative_velocity(a, b, r1, r2, n);

        // compute velocity loss from drag
        let v_damp = (this.target_vrn - vrn) * this.v_coef;
        this.target_vrn = vrn + v_damp;

        v_damp *= this.nMass;
        apply_impulses(a, b, this.r1, this.r2, this.n.x * v_damp, this.n.y * v_damp);
    }

    getImpulse() {
        return 0;
    }
}

