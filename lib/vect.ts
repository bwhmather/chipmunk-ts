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

export class Vect {
    x: number;
    y: number;

    constructor(x, y) {
        this.x = x;
        this.y = y;
    }

    add(v2) {
        this.x += v2.x;
        this.y += v2.y;
        return this;
    }

    sub(v2) {
        this.x -= v2.x;
        this.y -= v2.y;
        return this;
    }

    neg() {
        this.x = -this.x;
        this.y = -this.y;
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

    rotate(v2) {
        this.x = this.x * v2.x - this.y * v2.y;
        this.y = this.x * v2.y + this.y * v2.x;
        return this;
    }
}

export function v(x, y) {
    return new Vect(x, y);
}

export const vzero = new Vect(0, 0);

/// Vector dot product.
export function vdot(v1: Vect, v2: Vect): number {
    return v1.x * v2.x + v1.y * v2.y;
}

export function vdot2(x1: number, y1: number, x2: number, y2: number): number {
    return x1 * x2 + y1 * y2;
}

/// Returns the length of v.
export function vlength(v: Vect): number {
    return Math.sqrt(vdot(v, v));
}

export function vlength2(x: number, y: number): number {
    return Math.sqrt(x * x + y * y);
}

/// Check if two vectors are equal. (Be careful when comparing floating point numbers!)
export function veql(v1: Vect, v2: Vect): boolean {
    return (v1.x === v2.x && v1.y === v2.y);
}

/// Add two vectors
export function vadd(v1: Vect, v2: Vect): Vect {
    return new Vect(v1.x + v2.x, v1.y + v2.y);
}

/// Subtract two vectors.
export function vsub(v1: Vect, v2: Vect): Vect {
    return new Vect(v1.x - v2.x, v1.y - v2.y);
}

/// Negate a vector.
export function vneg(v: Vect): Vect {
    return new Vect(-v.x, -v.y);
}

/// Scalar multiplication.
export function vmult(v: Vect, s: number): Vect {
    return new Vect(v.x * s, v.y * s);
}

/// 2D vector cross product analog.
/// The cross product of 2D vectors results in a 3D vector with only a z component.
/// This function returns the magnitude of the z value.
export function vcross(v1: Vect, v2: Vect): number {
    return v1.x * v2.y - v1.y * v2.x;
}

export function vcross2(x1: number, y1: number, x2: number, y2: number): number {
    return x1 * y2 - y1 * x2;
}

/// Returns a perpendicular vector. (90 degree rotation)
export function vperp(v: Vect): Vect {
    return new Vect(-v.y, v.x);
}

/// Returns a perpendicular vector. (-90 degree rotation)
export function vpvrperp(v: Vect): Vect {
    return new Vect(v.y, -v.x);
}

/// Returns the vector projection of v1 onto v2.
export function vproject(v1: Vect, v2: Vect): Vect {
    return vmult(v2, vdot(v1, v2) / vlengthsq(v2));
}

/// Uses complex number multiplication to rotate v1 by v2. Scaling will occur if v1 is not a unit vector.
export function vrotate(v1: Vect, v2: Vect): Vect {
    return new Vect(v1.x * v2.x - v1.y * v2.y, v1.x * v2.y + v1.y * v2.x);
}


/// Inverse of vrotate().
export function vunrotate(v1: Vect, v2: Vect): Vect {
    return new Vect(v1.x * v2.x + v1.y * v2.y, v1.y * v2.x - v1.x * v2.y);
}

/// Returns the squared length of v. Faster than vlength() when you only need to compare lengths.
export function vlengthsq(v: Vect): number {
    return vdot(v, v);
}

export function vlengthsq2(x: number, y: number): number {
    return x * x + y * y;
}

/// Linearly interpolate between v1 and v2.
export function vlerp(v1: Vect, v2: Vect, t: number): Vect {
    return vadd(vmult(v1, 1 - t), vmult(v2, t));
}

/// Returns a normalized copy of v.
export function vnormalize(v: Vect): Vect {
    return vmult(v, 1 / vlength(v));
}

/// Returns a normalized copy of v or vzero if v was already vzero. Protects against divide by zero errors.
export function vnormalize_safe(v: Vect): Vect {
    return (v.x === 0 && v.y === 0 ? vzero : vnormalize(v));
}

/// Clamp v to length len.
export function vclamp(v: Vect, len: number): Vect {
    return (vdot(v, v) > len * len) ? vmult(vnormalize(v), len) : v;
}

/// Linearly interpolate between v1 towards v2 by distance d.
export function vlerpconst(v1: Vect, v2: Vect, d: number): Vect {
    return vadd(v1, vclamp(vsub(v2, v1), d));
}

/// Returns the distance between v1 and v2.
export function vdist(v1: Vect, v2: Vect): number {
    return vlength(vsub(v1, v2));
}

/// Returns the squared distance between v1 and v2. Faster than vdist() when you only need to compare distances.
export function vdistsq(v1: Vect, v2: Vect): number {
    return vlengthsq(vsub(v1, v2));
}

/// Returns true if the distance between v1 and v2 is less than dist.
export function vnear(v1: Vect, v2: Vect, dist): boolean {
    return vdistsq(v1, v2) < dist * dist;
}

/// Spherical linearly interpolate between v1 and v2.
export function vslerp(v1: Vect, v2: Vect, t: number): Vect {
    const omega = Math.acos(vdot(v1, v2));

    if (omega) {
        const denom = 1 / Math.sin(omega);
        return vadd(vmult(v1, Math.sin((1 - t) * omega) * denom), vmult(v2, Math.sin(t * omega) * denom));
    } else {
        return v1;
    }
}

/// Spherical linearly interpolate between v1 towards v2 by no more than angle a radians
export function vslerpconst(v1: Vect, v2: Vect, a): Vect {
    const angle = Math.acos(vdot(v1, v2));
    return vslerp(v1, v2, Math.min(a, angle) / angle);
}

/// Returns the unit length vector for the given angle (in radians).
export function vforangle(a: number): Vect {
    return new Vect(Math.cos(a), Math.sin(a));
}

/// Returns the angular direction v is pointing in (in radians).
export function vtoangle(v: Vect): number {
    return Math.atan2(v.y, v.x);
}

///	Returns a string representation of v. Intended mostly for debugging purposes and not production use.
export function vstr(v: Vect): string {
    return "(" + v.x.toFixed(3) + ", " + v.y.toFixed(3) + ")";
}


