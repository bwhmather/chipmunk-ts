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

import {
    Vect,
    vadd, vsub, vmult,
    vcross, vcross2,
    vdist, vdistsq,
    vdot, vdot2,
    vlength, vlengthsq, vlengthsq2,
} from './vect';
import { BB } from './bb';

export function assert(value: any, message: string): void {
    if (!value) {
        throw new Error('Assertion failed: ' + message);
    }
}

export function assertSoft(value: any, message: string): void {
    if (!value && console && console.warn) {
        console.warn("ASSERTION FAILED: " + message);
        if (console.trace) {
            console.trace();
        }
    }
}

/* The hashpair function takes two numbers and returns a hash code for them.
 * Required that hashPair(a, b) === hashPair(b, a).
 * Chipmunk's hashPair function is defined as:
 *   #define CP_HASH_COEF (3344921057ul)
 *   #define CP_HASH_PAIR(A, B) ((cpHashValue)(A)*CP_HASH_COEF ^ (cpHashValue)(B)*CP_HASH_COEF)
 * But thats not suitable in javascript because multiplying by a large number will make the number
 * a large float.
 *
 * The result of hashPair is used as the key in objects, so it returns a string.
 */
export function hashPair(a: number, b: number): string {
    //assert(typeof(a) === 'number', "HashPair used on something not a number");
    return a < b ? a + ' ' + b : b + ' ' + a;
}

export function deleteObjFromList<T>(arr: T[], obj: T): void {
    for (let i = 0; i < arr.length; i++) {
        if (arr[i] === obj) {
            arr[i] = arr[arr.length - 1];
            arr.length--;

            return;
        }
    }
}

export function closestPointOnSegment(p: Vect, a: Vect, b: Vect): Vect {
    const delta = vsub(a, b);
    const t = clamp01(vdot(delta, vsub(p, b)) / vlengthsq(delta));
    return vadd(b, vmult(delta, t));
}

export function closestPointOnSegment2(
    px: number, py: number, ax: number, ay: number, bx: number, by: number,
): Vect {
    const deltax = ax - bx;
    const deltay = ay - by;
    const t = clamp01(
        vdot2(deltax, deltay, px - bx, py - by) /
        vlengthsq2(deltax, deltay)
    );
    return new Vect(bx + deltax * t, by + deltay * t);
}

export function momentForCircle(
    m: number, r1: number, r2: number, offset: Vect
): number {
    return m * (0.5 * (r1 * r1 + r2 * r2) + vlengthsq(offset));
}

export function areaForCircle(r1: number, r2: number): number {
    return Math.PI * Math.abs(r1 * r1 - r2 * r2);
}

export function momentForSegment(m: number, a: Vect, b: Vect): number {
    const offset = vmult(vadd(a, b), 0.5);
    return m * (vdistsq(b, a) / 12 + vlengthsq(offset));
}

export function areaForSegment(a: Vect, b: Vect, r: number): number {
    return r * (Math.PI * r + 2 * vdist(a, b));
}

export function momentForPoly(
    m: number, verts: number[], offset: Vect,
): number {
    let sum1 = 0;
    let sum2 = 0;
    const len = verts.length;
    for (let i = 0; i < len; i += 2) {
        const v1x = verts[i] + offset.x;
        const v1y = verts[i + 1] + offset.y;
        const v2x = verts[(i + 2) % len] + offset.x;
        const v2y = verts[(i + 3) % len] + offset.y;

        const a = vcross2(v2x, v2y, v1x, v1y);
        const b = vdot2(v1x, v1y, v1x, v1y) + vdot2(v1x, v1y, v2x, v2y) + vdot2(v2x, v2y, v2x, v2y);

        sum1 += a * b;
        sum2 += a;
    }

    return (m * sum1) / (6 * sum2);
}

export function areaForPoly(verts: number[]): number {
    let area = 0;
    for (let i = 0, len = verts.length; i < len; i += 2) {
        area += vcross(new Vect(verts[i], verts[i + 1]), new Vect(verts[(i + 2) % len], verts[(i + 3) % len]));
    }

    return -area / 2;
}

export function centroidForPoly(verts: number[]): Vect {
    let sum = 0;
    let vsum = new Vect(0, 0);

    for (let i = 0, len = verts.length; i < len; i += 2) {
        const v1 = new Vect(verts[i], verts[i + 1]);
        const v2 = new Vect(verts[(i + 2) % len], verts[(i + 3) % len]);
        const cross = vcross(v1, v2);

        sum += cross;
        vsum = vadd(vsum, vmult(vadd(v1, v2), cross));
    }

    return vmult(vsum, 1 / (3 * sum));
}

export function recenterPoly(verts: number[]): void {
    const centroid = centroidForPoly(verts);

    for (let i = 0; i < verts.length; i += 2) {
        verts[i] -= centroid.x;
        verts[i + 1] -= centroid.y;
    }
}

export function momentForBox(m: number, width: number, height: number): number {
    return m * (width * width + height * height) / 12;
}

export function momentForBox2(m: number, bb: BB): number {
    const width = bb.r - bb.l;
    const height = bb.t - bb.b;
    const offset = vmult(new Vect(bb.l + bb.r, bb.b + bb.t), 0.5);

    // TODO NaN when offset is 0 and m is INFINITY	
    return momentForBox(m, width, height) + m * vlengthsq(offset);
}

/// Clamp @c f to be between @c min and @c max.
export function clamp(f: number, minv: number, maxv: number): number {
    return Math.min(Math.max(f, minv), maxv);
}

/// Clamp @c f to be between 0 and 1.
export function clamp01(f: number): number {
    return Math.max(0, Math.min(f, 1));
}

/// Linearly interpolate (or extrapolate) between @c f1 and @c f2 by @c t percent.
export function lerp(f1: number, f2: number, t: number): number {
    return f1 * (1 - t) + f2 * t;
}

/// Linearly interpolate from @c f1 to @c f2 by no more than @c d.
export function lerpconst(f1: number, f2: number, d: number): number {
    return f1 + clamp(f2 - f1, -d, d);
}
