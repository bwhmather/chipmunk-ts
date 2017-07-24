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

import { assertSoft } from '../util';
import { Vect, vdot, vdot2, vcross, vcross2 } from '../vect';
import { Body } from '../body';


// a and b are bodies.
export function relative_velocity(
    a: Body, b: Body, r1: Vect, r2: Vect,
) {
    //var v1_sum = vadd(a.v, vmult(vperp(r1), a.w));
    const v1_sumx = a.vx + (-r1.y) * a.w;
    const v1_sumy = a.vy + (r1.x) * a.w;

    //var v2_sum = vadd(b.v, vmult(vperp(r2), b.w));
    const v2_sumx = b.vx + (-r2.y) * b.w;
    const v2_sumy = b.vy + (r2.x) * b.w;

    //	return vsub(v2_sum, v1_sum);
    return new Vect(v2_sumx - v1_sumx, v2_sumy - v1_sumy);
};

export function normal_relative_velocity(
    a: Body, b: Body,
    r1: Vect, r2: Vect,
    n: Vect,
) {
    //return vdot(relative_velocity(a, b, r1, r2), n);
    const v1_sumx = a.vx + (-r1.y) * a.w;
    const v1_sumy = a.vy + (r1.x) * a.w;
    const v2_sumx = b.vx + (-r2.y) * b.w;
    const v2_sumy = b.vy + (r2.x) * b.w;

    return vdot2(v2_sumx - v1_sumx, v2_sumy - v1_sumy, n.x, n.y);
};

export function apply_impulse(
    body: Body, jx: number, jy: number, r: Vect,
) {
    //	body.v = body.v.add(vmult(j, body.m_inv));
    body.vx += jx * body.m_inv;
    body.vy += jy * body.m_inv;
    //	body.w += body.i_inv*vcross(r, j);
    body.w += body.i_inv * (r.x * jy - r.y * jx);
};

export function apply_impulses(
    a: Body, b: Body, r1: Vect, r2: Vect, jx: number, jy: number,
) {
    apply_impulse(a, -jx, -jy, r1);
    apply_impulse(b, jx, jy, r2);
};

export function apply_bias_impulse(
    body: Body, jx: number, jy: number, r: Vect,
) {
    //body.v_bias = vadd(body.v_bias, vmult(j, body.m_inv));
    body.v_biasx += jx * body.m_inv;
    body.v_biasy += jy * body.m_inv;
    body.w_bias += body.i_inv * vcross2(r.x, r.y, jx, jy);
};

export function k_scalar_body(body: Body, r: Vect, n: Vect) {
    const rcn = vcross(r, n);
    return body.m_inv + body.i_inv * rcn * rcn;
};

export function k_scalar(
    a: Body, b: Body, r1: Vect, r2: Vect, n: Vect,
): number {
    const value = k_scalar_body(a, r1, n) + k_scalar_body(b, r2, n);
    assertSoft(value !== 0, "Unsolvable collision or constraint.");

    return value;
};

// k1 and k2 are modified by the function to contain the outputs.
export function k_tensor(
    a: Body, b: Body,
    r1:Vect, r2: Vect,
    k1: Vect, k2: Vect,
): void {
    // calculate mass matrix
    // If I wasn't lazy and wrote a proper matrix class, this wouldn't be so gross...
    let k11;

    let k12;
    let k21;
    let k22;
    const m_sum = a.m_inv + b.m_inv;

    // start with I*m_sum
    k11 = m_sum; k12 = 0;
    k21 = 0; k22 = m_sum;

    // add the influence from r1
    const a_i_inv = a.i_inv;
    const r1xsq = r1.x * r1.x * a_i_inv;
    const r1ysq = r1.y * r1.y * a_i_inv;
    const r1nxy = -r1.x * r1.y * a_i_inv;
    k11 += r1ysq; k12 += r1nxy;
    k21 += r1nxy; k22 += r1xsq;

    // add the influnce from r2
    const b_i_inv = b.i_inv;
    const r2xsq = r2.x * r2.x * b_i_inv;
    const r2ysq = r2.y * r2.y * b_i_inv;
    const r2nxy = -r2.x * r2.y * b_i_inv;
    k11 += r2ysq; k12 += r2nxy;
    k21 += r2nxy; k22 += r2xsq;

    // invert
    const determinant = k11 * k22 - k12 * k21;
    assertSoft(determinant !== 0, "Unsolvable constraint.");

    const det_inv = 1 / determinant;

    k1.x = k22 * det_inv; k1.y = -k12 * det_inv;
    k2.x = -k21 * det_inv; k2.y = k11 * det_inv;
};

export function mult_k(vr: Vect, k1: Vect, k2:Vect): Vect {
    return new Vect(vdot(vr, k1), vdot(vr, k2));
};

export function bias_coef(errorBias: number, dt: number) {
    return 1 - errorBias ** dt;
};