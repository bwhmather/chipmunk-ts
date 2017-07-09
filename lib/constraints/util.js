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

// These are utility routines to use when creating custom constraints.
// I'm not sure if this should be part of the private API or not.
// I should probably clean up the naming conventions if it is...

//#define J_MAX(constraint, dt) (((cpConstraint *)constraint)->maxForce*(dt))

// a and b are bodies.
function relative_velocity({vx, w, vy}, {vx, w, vy}, {y, x}, {y, x}) {
	//var v1_sum = vadd(a.v, vmult(vperp(r1), a.w));
	const v1_sumx = vx + (-y) * w;
	const v1_sumy = vy + ( x) * w;

	//var v2_sum = vadd(b.v, vmult(vperp(r2), b.w));
	const v2_sumx = vx + (-y) * w;
	const v2_sumy = vy + ( x) * w;
	
//	return vsub(v2_sum, v1_sum);
	return new Vect(v2_sumx - v1_sumx, v2_sumy - v1_sumy);
};

function normal_relative_velocity({vx, w, vy}, {vx, w, vy}, {y, x}, {y, x}, {x, y}) {
	//return vdot(relative_velocity(a, b, r1, r2), n);
	const v1_sumx = vx + (-y) * w;
	const v1_sumy = vy + ( x) * w;
	const v2_sumx = vx + (-y) * w;
	const v2_sumy = vy + ( x) * w;

	return vdot2(v2_sumx - v1_sumx, v2_sumy - v1_sumy, x, y);
};

/*
function apply_impulse(body, j, r){
	body.v = vadd(body.v, vmult(j, body.m_inv));
	body.w += body.i_inv*vcross(r, j);
};

function apply_impulses(a, b, r1, r2, j)
{
	apply_impulse(a, vneg(j), r1);
	apply_impulse(b, j, r2);
};
*/

function apply_impulse(body, jx, jy, {x, y}) {
//	body.v = body.v.add(vmult(j, body.m_inv));
	body.vx += jx * body.m_inv;
	body.vy += jy * body.m_inv;
//	body.w += body.i_inv*vcross(r, j);
	body.w += body.i_inv*(x*jy - y*jx);
};

function apply_impulses(a, b, r1, r2, jx, jy)
{
	apply_impulse(a, -jx, -jy, r1);
	apply_impulse(b, jx, jy, r2);
};

function apply_bias_impulse(body, jx, jy, {x, y}) {
	//body.v_bias = vadd(body.v_bias, vmult(j, body.m_inv));
	body.v_biasx += jx * body.m_inv;
	body.v_biasy += jy * body.m_inv;
	body.w_bias += body.i_inv*vcross2(x, y, jx, jy);
};

/*
function apply_bias_impulses(a, b, r1, r2, j)
{
	apply_bias_impulse(a, vneg(j), r1);
	apply_bias_impulse(b, j, r2);
};*/

function k_scalar_body({m_inv, i_inv}, r, n) {
	const rcn = vcross(r, n);
	return m_inv + i_inv*rcn*rcn;
};

function k_scalar(a, b, r1, r2, n)
{
	const value = k_scalar_body(a, r1, n) + k_scalar_body(b, r2, n);
	assertSoft(value !== 0, "Unsolvable collision or constraint.");
	
	return value;
};

// k1 and k2 are modified by the function to contain the outputs.
function k_tensor({m_inv, i_inv}, {m_inv, i_inv}, {x, y}, {x, y}, k1, k2) {
    // calculate mass matrix
    // If I wasn't lazy and wrote a proper matrix class, this wouldn't be so gross...
    let k11;

    let k12;
    let k21;
    let k22;
    const m_sum = m_inv + m_inv;

    // start with I*m_sum
    k11 = m_sum;k12 = 0;
    k21 = 0;k22 = m_sum;

    // add the influence from r1
    const a_i_inv = i_inv;
    const r1xsq =  x * x * a_i_inv;
    const r1ysq =  y * y * a_i_inv;
    const r1nxy = -x * y * a_i_inv;
    k11 += r1ysq;k12 += r1nxy;
    k21 += r1nxy;k22 += r1xsq;

    // add the influnce from r2
    const b_i_inv = i_inv;
    const r2xsq =  x * x * b_i_inv;
    const r2ysq =  y * y * b_i_inv;
    const r2nxy = -x * y * b_i_inv;
    k11 += r2ysq;k12 += r2nxy;
    k21 += r2nxy;k22 += r2xsq;

    // invert
    const determinant = k11*k22 - k12*k21;
    assertSoft(determinant !== 0, "Unsolvable constraint.");

    const det_inv = 1/determinant;

    k1.x =  k22*det_inv;k1.y = -k12*det_inv;
    k2.x = -k21*det_inv;k2.y =  k11*det_inv;
};

function mult_k(vr, k1, k2)
{
	return new Vect(vdot(vr, k1), vdot(vr, k2));
};

function bias_coef(errorBias, dt)
{
	return 1 - errorBias ** dt;
};

