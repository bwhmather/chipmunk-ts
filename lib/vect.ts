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

// I'm using an array tuple here because (at time of writing) its about 3x faster
// than an object on firefox, and the same speed on chrome.

//var numVects = 0;

cp.v = (x, y) => new Vect(x, y);

class Vect {
    constructor(x, y) {
        this.x = x;
        this.y = y;
        //numVects++;

        //	var s = new Error().stack;
        //	traces[s] = traces[s] ? traces[s]+1 : 1;
    }

    add({ x, y }) {
        this.x += x;
        this.y += y;
        return this;
    }

    sub({ x, y }) {
        this.x -= x;
        this.y -= y;
        return this;
    }

    mult(s) {
        this.x *= s;
        this.y *= s;
        return this;
    }

    project(v2) {
        this.mult(vdot(this, v2) / vlengthsq(v2));
        return this;
    }

    neg() {
        this.x = -this.x;
        this.y = -this.y;
        return this;
    }

    rotate({ x, y }) {
        this.x = this.x * x - this.y * y;
        this.y = this.x * y + this.y * x;
        return this;
    }
}


















const vzero = cp.vzero = new Vect(0, 0);

// The functions below *could* be rewritten to be instance methods on Vect. I don't
// know how that would effect performance. For now, I'm keeping the JS similar to
// the original C code.

/// Vector dot product.
function vdot({ x, y }, { x, y }) {
    return x * x + y * y;
}

const vdot2 = (x1, y1, x2, y2) => x1 * x2 + y1 * y2;

/// Returns the length of v.
function vlength(v) {
    return Math.sqrt(vdot(v, v));
}

function vlength2(x, y) {
    return Math.sqrt(x * x + y * y);
}

/// Check if two vectors are equal. (Be careful when comparing floating point numbers!)
function veql({ x, y }, { x, y }) {
    return x === x && y === y;
}

/// Add two vectors
function vadd({ x, y }, { x, y }) {
    return new Vect(x + x, y + y);
}

/// Subtract two vectors.
function vsub({ x, y }, { x, y }) {
    return new Vect(x - x, y - y);
}
/// Negate a vector.
function vneg({ x, y }) {
    return new Vect(-x, -y);
}

/// Scalar multiplication.
function vmult({ x, y }, s) {
    return new Vect(x * s, y * s);
}
/// 2D vector cross product analog.
/// The cross product of 2D vectors results in a 3D vector with only a z component.
/// This function returns the magnitude of the z value.
function vcross({ x, y }, { y, x }) {
    return x * y - y * x;
}

const vcross2 = (x1, y1, x2, y2) => x1 * y2 - y1 * x2;

/// Returns a perpendicular vector. (90 degree rotation)
function vperp({ y, x }) {
    return new Vect(-y, x);
}

/// Returns a perpendicular vector. (-90 degree rotation)
function vpvrperp({ y, x }) {
    return new Vect(y, -x);
}

/// Returns the vector projection of v1 onto v2.
function vproject(v1, v2) {
    return vmult(v2, vdot(v1, v2) / vlengthsq(v2));
}

/// Uses complex number multiplication to rotate v1 by v2. Scaling will occur if v1 is not a unit vector.
function vrotate({ x, y }, { x, y }) {
    return new Vect(x * x - y * y, x * y + y * x);
}

/// Inverse of vrotate().
function vunrotate({ x, y }, { x, y }) {
    return new Vect(x * x + y * y, y * x - x * y);
}

/// Returns the squared length of v. Faster than vlength() when you only need to compare lengths.
function vlengthsq(v) {
    return vdot(v, v);
}

function vlengthsq2(x, y) {
    return x * x + y * y;
}

/// Linearly interpolate between v1 and v2.
function vlerp(v1, v2, t) {
    return vadd(vmult(v1, 1 - t), vmult(v2, t));
}

/// Returns a normalized copy of v.
function vnormalize(v) {
    return vmult(v, 1 / vlength(v));
}

/// Returns a normalized copy of v or vzero if v was already vzero. Protects against divide by zero errors.
function vnormalize_safe(v) {
    return (v.x === 0 && v.y === 0 ? vzero : vnormalize(v));
}

/// Clamp v to length len.
function vclamp(v, len) {
    return (vdot(v, v) > len * len) ? vmult(vnormalize(v), len) : v;
}

/// Linearly interpolate between v1 towards v2 by distance d.
function vlerpconst(v1, v2, d) {
    return vadd(v1, vclamp(vsub(v2, v1), d));
}

/// Returns the distance between v1 and v2.
function vdist(v1, v2) {
    return vlength(vsub(v1, v2));
}

/// Returns the squared distance between v1 and v2. Faster than vdist() when you only need to compare distances.
function vdistsq(v1, v2) {
    return vlengthsq(vsub(v1, v2));
}

/// Returns true if the distance between v1 and v2 is less than dist.
function vnear(v1, v2, dist) {
    return vdistsq(v1, v2) < dist * dist;
}

/// Spherical linearly interpolate between v1 and v2.
function vslerp(v1, v2, t) {
    const omega = Math.acos(vdot(v1, v2));

    if (omega) {
        const denom = 1 / Math.sin(omega);
        return vadd(vmult(v1, Math.sin((1 - t) * omega) * denom), vmult(v2, Math.sin(t * omega) * denom));
    } else {
        return v1;
    }
}

/// Spherical linearly interpolate between v1 towards v2 by no more than angle a radians
function vslerpconst(v1, v2, a) {
    const angle = Math.acos(vdot(v1, v2));
    return vslerp(v1, v2, min(a, angle) / angle);
}

/// Returns the unit length vector for the given angle (in radians).
function vforangle(a) {
    return new Vect(Math.cos(a), Math.sin(a));
}

/// Returns the angular direction v is pointing in (in radians).
function vtoangle({ y, x }) {
    return Math.atan2(y, x);
}

///	Returns a string representation of v. Intended mostly for debugging purposes and not production use.
function vstr({ x, y }) {
    return "(" + x.toFixed(3) + ", " + y.toFixed(3) + ")";
}

