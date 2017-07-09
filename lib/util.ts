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
//var VERSION = CP_VERSION_MAJOR + "." + CP_VERSION_MINOR + "." + CP_VERSION_RELEASE;

import {
    Vect,
    vadd, vsub, vmult,
    vcross, vcross2,
    vdist, vdistsq,
    vdot, vdot2,
    vlength, vlengthsq, vlengthsq2,
} from './vect';

export function assert(value, message) {
    if (!value) {
        throw new Error('Assertion failed: ' + message);
    }
}

export function assertSoft(value, message) {
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
export function hashPair(a, b) {
    //assert(typeof(a) === 'number', "HashPair used on something not a number");
    return a < b ? a + ' ' + b : b + ' ' + a;
}

export function deleteObjFromList(arr, obj) {
    for (let i = 0; i < arr.length; i++) {
        if (arr[i] === obj) {
            arr[i] = arr[arr.length - 1];
            arr.length--;

            return;
        }
    }
}

export function closestPointOnSegment(p, a, b) {
    const delta = vsub(a, b);
    const t = clamp01(vdot(delta, vsub(p, b)) / vlengthsq(delta));
    return vadd(b, vmult(delta, t));
}

export function closestPointOnSegment2(px, py, ax, ay, bx, by) {
    const deltax = ax - bx;
    const deltay = ay - by;
    const t = clamp01(vdot2(deltax, deltay, px - bx, py - by) / vlengthsq2(deltax, deltay));
    return new Vect(bx + deltax * t, by + deltay * t);
}

export function momentForCircle(m, r1, r2, offset) {
    return m * (0.5 * (r1 * r1 + r2 * r2) + vlengthsq(offset));
}

export function areaForCircle(r1, r2) {
    return Math.PI * Math.abs(r1 * r1 - r2 * r2);
}

export function momentForSegment(m, a, b) {
    const offset = vmult(vadd(a, b), 0.5);
    return m * (vdistsq(b, a) / 12 + vlengthsq(offset));
}

export function areaForSegment(a, b, r) {
    return r * (Math.PI * r + 2 * vdist(a, b));
}

export function momentForPoly(m, verts, {x, y}) {
    let sum1 = 0;
    let sum2 = 0;
    const len = verts.length;
    for (let i = 0; i < len; i += 2) {
        const v1x = verts[i] + x;
        const v1y = verts[i + 1] + y;
        const v2x = verts[(i + 2) % len] + x;
        const v2y = verts[(i + 3) % len] + y;

        const a = vcross2(v2x, v2y, v1x, v1y);
        const b = vdot2(v1x, v1y, v1x, v1y) + vdot2(v1x, v1y, v2x, v2y) + vdot2(v2x, v2y, v2x, v2y);

        sum1 += a * b;
        sum2 += a;
    }

    return (m * sum1) / (6 * sum2);
}

export function areaForPoly(verts) {
    let area = 0;
    for (let i = 0, len = verts.length; i < len; i += 2) {
        area += vcross(new Vect(verts[i], verts[i + 1]), new Vect(verts[(i + 2) % len], verts[(i + 3) % len]));
    }

    return -area / 2;
}

export function centroidForPoly(verts) {
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

export function recenterPoly(verts) {
    const centroid = centroidForPoly(verts);

    for (let i = 0; i < verts.length; i += 2) {
        verts[i] -= centroid.x;
        verts[i + 1] -= centroid.y;
    }
}

export function momentForBox(m, width, height) {
    return m * (width * width + height * height) / 12;
}

export function momentForBox2(m, {r, l, t, b}) {
    const width = r - l;
    const height = t - b;
    const offset = vmult(new Vect(l + r, b + t), 0.5);

    // TODO NaN when offset is 0 and m is INFINITY	
    return momentForBox(m, width, height) + m * vlengthsq(offset);
}

// Quick hull

export function loopIndexes(verts) {
    let start = 0;
    let end = 0;
    let minx;
    let miny;
    let maxx;
    let maxy;
    minx = maxx = verts[0];
    miny = maxy = verts[1];

    const count = verts.length >> 1;
    for (let i = 1; i < count; i++) {
        const x = verts[i * 2];
        const y = verts[i * 2 + 1];

        if (x < minx || (x == minx && y < miny)) {
            minx = x;
            miny = y;
            start = i;
        } else if (x > maxx || (x == maxx && y > maxy)) {
            maxx = x;
            maxy = y;
            end = i;
        }
    }
    return [start, end];
};

function SWAP(arr, idx1, idx2) {
    let tmp = arr[idx1 * 2];
    arr[idx1 * 2] = arr[idx2 * 2];
    arr[idx2 * 2] = tmp;

    tmp = arr[idx1 * 2 + 1];
    arr[idx1 * 2 + 1] = arr[idx2 * 2 + 1];
    arr[idx2 * 2 + 1] = tmp;
}

function QHullPartition(verts, offs, count, a, b, tol) {
    if (count === 0) return 0;

    let max = 0;
    let pivot = offs;

    const delta = vsub(b, a);
    const valueTol = tol * vlength(delta);

    let head = offs;
    for (let tail = offs + count - 1; head <= tail;) {
        const v = new Vect(verts[head * 2], verts[head * 2 + 1]);
        const value = vcross(delta, vsub(v, a));
        if (value > valueTol) {
            if (value > max) {
                max = value;
                pivot = head;
            }

            head++;
        } else {
            SWAP(verts, head, tail);
            tail--;
        }
    }

    // move the new pivot to the front if it's not already there.
    if (pivot != offs) SWAP(verts, offs, pivot);
    return head - offs;
}

function QHullReduce(tol, verts, offs, count, a, pivot, b, resultPos) {
    if (count < 0) {
        return 0;
    } else if (count == 0) {
        verts[resultPos * 2] = pivot.x;
        verts[resultPos * 2 + 1] = pivot.y;
        return 1;
    } else {
        const left_count = QHullPartition(verts, offs, count, a, pivot, tol);
        const left = new Vect(verts[offs * 2], verts[offs * 2 + 1]);
        let index = QHullReduce(tol, verts, offs + 1, left_count - 1, a, left, pivot, resultPos);

        const pivotPos = resultPos + index++;
        verts[pivotPos * 2] = pivot.x;
        verts[pivotPos * 2 + 1] = pivot.y;

        const right_count = QHullPartition(verts, offs + left_count, count - left_count, pivot, b, tol);
        const right = new Vect(verts[(offs + left_count) * 2], verts[(offs + left_count) * 2 + 1]);
        return index + QHullReduce(tol, verts, offs + left_count + 1, right_count - 1, pivot, right, b, resultPos + index);
    }
}

// QuickHull seemed like a neat algorithm, and efficient-ish for large input sets.
// My implementation performs an in place reduction using the result array as scratch space.
//
// Pass an Array into result to put the result of the calculation there. Otherwise, pass null
// and the verts list will be edited in-place.
//
// Expects the verts to be described in the same way as cpPolyShape - which is to say, it should
// be a list of [x1,y1,x2,y2,x3,y3,...].
//
// tolerance is in world coordinates. Eg, 2.
export function convexHull(verts, result, tolerance) {
    if (result) {
        // Copy the line vertexes into the empty part of the result polyline to use as a scratch buffer.
        for (let i = 0; i < verts.length; i++) {
            result[i] = verts[i];
        }
    } else {
        // If a result array was not specified, reduce the input instead.
        result = verts;
    }

    // Degenerate case, all points are the same.
    const indexes = loopIndexes(verts);
    const start = indexes[0];
    const end = indexes[1];
    if (start == end) {
        //if(first) (*first) = 0;
        result.length = 2;
        return result;
    }

    SWAP(result, 0, start);
    SWAP(result, 1, end == 0 ? start : end);

    const a = new Vect(result[0], result[1]);
    const b = new Vect(result[2], result[3]);

    const count = verts.length >> 1;
    //if(first) (*first) = start;
    const resultCount = QHullReduce(tolerance, result, 2, count - 2, a, b, a, 1) + 1;
    result.length = resultCount * 2;

    // assertSoft(polyValidate(result),
    //     "Internal error: cpConvexHull() and cpPolyValidate() did not agree." +
    //     "Please report this error with as much info as you can.");
    return result;
}

/// Clamp @c f to be between @c min and @c max.
export function clamp(f, minv, maxv) {
    return Math.min(Math.max(f, minv), maxv);
}

/// Clamp @c f to be between 0 and 1.
export function clamp01(f) {
    return Math.max(0, Math.min(f, 1));
}

/// Linearly interpolate (or extrapolate) between @c f1 and @c f2 by @c t percent.
export function lerp(f1, f2, t) {
    return f1 * (1 - t) + f2 * t;
}

/// Linearly interpolate from @c f1 to @c f2 by no more than @c d.
export function lerpconst(f1, f2, d) {
    return f1 + clamp(f2 - f1, -d, d);
}

