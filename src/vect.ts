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

export class Vect {
    readonly x: number;
    readonly y: number;

    constructor(x: number, y: number) {
        this.x = x;
        this.y = y;
    }
}

export function v(x: number, y: number): Vect {
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
export function vlength(vect: Vect): number {
    return Math.sqrt(vdot(vect, vect));
}

export function vlength2(x: number, y: number): number {
    return Math.sqrt(x * x + y * y);
}

/// Check if two vectors are equal. (Be careful when comparing floating point
/// numbers!)
export function veql(v1: Vect, v2: Vect): boolean {
    return (v1.x === v2.x && v1.y === v2.y);
}

/// Add two vectors
export function vadd(...vectors: Vect[]) {
    let x = 0;
    let y = 0;
    for (const vector of vectors) {
        x += vector.x;
        y += vector.y;
    }
    return new Vect(x, y);
}

/// Subtract two vectors.
export function vsub(v1: Vect, v2: Vect): Vect {
    return new Vect(v1.x - v2.x, v1.y - v2.y);
}

/// Negate a vector.
export function vneg(vect: Vect): Vect {
    return new Vect(-vect.x, -vect.y);
}

/// Scalar multiplication.
export function vmult(vect: Vect, s: number): Vect {
    return new Vect(vect.x * s, vect.y * s);
}

/// 2D vector cross product analog.
/// The cross product of 2D vectors results in a 3D vector with only a z
/// component.  This function returns the magnitude of the z value..
export function vcross(v1: Vect, v2: Vect): number {
    return v1.x * v2.y - v1.y * v2.x;
}

export function vcross2(
    x1: number, y1: number, x2: number, y2: number,
): number {
    return x1 * y2 - y1 * x2;
}

/// Returns a perpendicular vector. (90 degree rotation)
export function vperp(vect: Vect): Vect {
    return new Vect(-vect.y, vect.x);
}

/// Returns a perpendicular vector. (-90 degree rotation)
export function vpvrperp(vect: Vect): Vect {
    return new Vect(vect.y, -vect.x);
}

/// Returns the vector projection of v1 onto v2.
export function vproject(v1: Vect, v2: Vect): Vect {
    return vmult(v2, vdot(v1, v2) / vlengthsq(v2));
}

/// Uses complex number multiplication to rotate v1 by v2. Scaling will occur
/// if v1 is not a unit vector.
export function vrotate(v1: Vect, v2: Vect): Vect {
    return new Vect(v1.x * v2.x - v1.y * v2.y, v1.x * v2.y + v1.y * v2.x);
}

/// Inverse of vrotate().
export function vunrotate(v1: Vect, v2: Vect): Vect {
    return new Vect(v1.x * v2.x + v1.y * v2.y, v1.y * v2.x - v1.x * v2.y);
}

/// Returns the squared length of v. Faster than vlength() when you only need to
/// compare lengths.
export function vlengthsq(vect: Vect): number {
    return vdot(vect, vect);
}

export function vlengthsq2(x: number, y: number): number {
    return x * x + y * y;
}

/// Linearly interpolate between v1 and v2.
export function vlerp(v1: Vect, v2: Vect, t: number): Vect {
    return vadd(vmult(v1, 1 - t), vmult(v2, t));
}

/// Returns a normalized copy of v.
export function vnormalize(vect: Vect): Vect {
    return vmult(vect, 1 / vlength(vect));
}

/// Returns a normalized copy of v or vzero if v was already vzero. Protects
/// against divide by zero errors.
export function vnormalize_safe(vect: Vect): Vect {
    return (vect.x === 0 && vect.y === 0 ? vzero : vnormalize(vect));
}

/// Clamp v to length len.
export function vclamp(vect: Vect, len: number): Vect {
    return (vdot(vect, vect) > len * len) ? vmult(vnormalize(vect), len) : vect;
}

/// Linearly interpolate between v1 towards v2 by distance d.
export function vlerpconst(v1: Vect, v2: Vect, d: number): Vect {
    return vadd(v1, vclamp(vsub(v2, v1), d));
}

/// Returns the distance between v1 and v2.
export function vdist(v1: Vect, v2: Vect): number {
    return vlength(vsub(v1, v2));
}

/// Returns the squared distance between v1 and v2. Faster than vdist() when you
/// only need to compare distances.
export function vdistsq(v1: Vect, v2: Vect): number {
    return vlengthsq(vsub(v1, v2));
}

/// Returns true if the distance between v1 and v2 is less than dist.
export function vnear(v1: Vect, v2: Vect, dist: number): boolean {
    return vdistsq(v1, v2) < dist * dist;
}

/// Spherical linearly interpolate between v1 and v2.
export function vslerp(v1: Vect, v2: Vect, t: number): Vect {
    const omega = Math.acos(vdot(v1, v2));

    if (omega) {
        const denom = 1 / Math.sin(omega);
        return vadd(
            vmult(v1, Math.sin((1 - t) * omega) * denom),
            vmult(v2, Math.sin(t * omega) * denom),
        );
    } else {
        return v1;
    }
}

/// Spherical linearly interpolate between v1 towards v2 by no more than angle
/// `a` radians.
export function vslerpconst(v1: Vect, v2: Vect, a: number): Vect {
    const angle = Math.acos(vdot(v1, v2));
    return vslerp(v1, v2, Math.min(a, angle) / angle);
}

/// Returns the unit length vector for the given angle (in radians).
export function vforangle(a: number): Vect {
    return new Vect(Math.cos(a), Math.sin(a));
}

/// Returns the angular direction v is pointing in (in radians).
export function vtoangle(vect: Vect): number {
    return Math.atan2(vect.y, vect.x);
}

/// Returns a string representation of v. Intended mostly for debugging purposes
/// and not production use.
export function vstr(vect: Vect): string {
    return "(" + vect.x.toFixed(3) + ", " + vect.y.toFixed(3) + ")";
}
