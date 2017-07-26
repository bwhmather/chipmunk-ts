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
import { vcross, vcross2, vdot, vdot2, Vect } from "../vect";

// a and b are bodies.
export function relativeVelocity(
    a: Body, b: Body, r1: Vect, r2: Vect,
) {
    // var v1_sum = vadd(a.v, vmult(vperp(r1), a.w));
    const v1sumx = a.vx + (-r1.y) * a.w;
    const v1sumy = a.vy + (r1.x) * a.w;

    // var v2_sum = vadd(b.v, vmult(vperp(r2), b.w));
    const v2sumx = b.vx + (-r2.y) * b.w;
    const v2sumy = b.vy + (r2.x) * b.w;

    // return vsub(v2_sum, v1_sum);
    return new Vect(v2sumx - v1sumx, v2sumy - v1sumy);
}

export function normalRelativeVelocity(
    a: Body, b: Body,
    r1: Vect, r2: Vect,
    n: Vect,
) {
    // return vdot(relative_velocity(a, b, r1, r2), n);
    const v1sumx = a.vx + (-r1.y) * a.w;
    const v1sumy = a.vy + (r1.x) * a.w;
    const v2sumx = b.vx + (-r2.y) * b.w;
    const v2sumy = b.vy + (r2.x) * b.w;

    return vdot2(v2sumx - v1sumx, v2sumy - v1sumy, n.x, n.y);
}

export function applyImpulse(
    body: Body, jx: number, jy: number, r: Vect,
) {
    // body.v = body.v.add(vmult(j, body.m_inv));
    body.vx += jx * body.massInv;
    body.vy += jy * body.massInv;
    // body.w += body.i_inv*vcross(r, j);
    body.w += body.inertiaInv * (r.x * jy - r.y * jx);
}

export function applyImpulses(
    a: Body, b: Body, r1: Vect, r2: Vect, jx: number, jy: number,
) {
    applyImpulse(a, -jx, -jy, r1);
    applyImpulse(b, jx, jy, r2);
}

export function applyBiasImpulse(
    body: Body, jx: number, jy: number, r: Vect,
) {
    // body.v_bias = vadd(body.v_bias, vmult(j, body.m_inv));
    body.vxBias += jx * body.massInv;
    body.vxBias += jy * body.massInv;
    body.wBias += body.inertiaInv * vcross2(r.x, r.y, jx, jy);
}

export function kScalarBody(body: Body, r: Vect, n: Vect) {
    const rcn = vcross(r, n);
    return body.massInv + body.inertiaInv * rcn * rcn;
}

export function kScalar(
    a: Body, b: Body, r1: Vect, r2: Vect, n: Vect,
): number {
    const value = kScalarBody(a, r1, n) + kScalarBody(b, r2, n);
    assertSoft(value !== 0, "Unsolvable collision or constraint.");

    return value;
}

// k1 and k2 are modified by the function to contain the outputs.
export function kTensor(
    a: Body, b: Body,
    r1: Vect, r2: Vect,
): [Vect, Vect] {
    let k11;

    let k12;
    let k21;
    let k22;
    const msum = a.massInv + b.massInv;

    // start with I*m_sum
    k11 = msum; k12 = 0;
    k21 = 0; k22 = msum;

    // add the influence from r1
    const r1xsq = r1.x * r1.x * a.inertiaInv;
    const r1ysq = r1.y * r1.y * a.inertiaInv;
    const r1nxy = -r1.x * r1.y * a.inertiaInv;
    k11 += r1ysq; k12 += r1nxy;
    k21 += r1nxy; k22 += r1xsq;

    // add the influnce from r2
    const r2xsq = r2.x * r2.x * b.inertiaInv;
    const r2ysq = r2.y * r2.y * b.inertiaInv;
    const r2nxy = -r2.x * r2.y * b.inertiaInv;
    k11 += r2ysq; k12 += r2nxy;
    k21 += r2nxy; k22 += r2xsq;

    // invert
    const determinant = k11 * k22 - k12 * k21;
    assertSoft(determinant !== 0, "Unsolvable constraint.");

    const determinantInv = 1 / determinant;

    return [
        new Vect(k22 * determinantInv, -k12 * determinantInv),
        new Vect(-k21 * determinantInv, k11 * determinantInv),
    ];
}

export function multK(vr: Vect, k1: Vect, k2: Vect): Vect {
    return new Vect(vdot(vr, k1), vdot(vr, k2));
}

export function biasCoef(errorBias: number, dt: number) {
    return 1 - errorBias ** dt;
}
