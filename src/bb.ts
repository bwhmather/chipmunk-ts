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

/// Chipmunk's axis-aligned 2D bounding box type along with a few handy
/// routines.

import { Vect } from "./vect";

// Bounding boxes are JS objects with {l, b, r, t} = left, bottom, right, top,
// respectively.
export class BB {
    readonly l: number;
    readonly b: number;
    readonly r: number;
    readonly t: number;

    constructor(l: number, b: number, r: number, t: number) {
        this.l = l;
        this.b = b;
        this.r = r;
        this.t = t;
    }
}

export function bb(l: number, b: number, r: number, t: number): BB {
    return new BB(l, b, r, t);
}

export function bbNewForCircle(c: Vect, r: number) {
    return new BB(
        c.x - r,
        c.y - r,
        c.x + r,
        c.y + r,
    );
}

/// Returns true if @c a and @c b intersect.
export function bbIntersects(boxA: BB, boxB: BB): boolean {
    return (
        boxA.l <= boxB.r &&
        boxB.l <= boxA.r &&
        boxA.b <= boxB.t &&
        boxB.b <= boxA.t
    );
}
export function bbIntersects2(
    box: BB,
    l: number, b: number, r: number, t: number,
): boolean {
    return (box.l <= r && l <= box.r && box.b <= t && b <= box.t);
}

/// Returns true if @c other lies completely within @c bb.
export function bbContainsBB(boxA: BB, boxB: BB): boolean {
    return (
        boxA.l <= boxB.l &&
        boxA.r >= boxB.r &&
        boxA.b <= boxB.b &&
        boxA.t >= boxB.t
    );
}

/// Returns true if @c bb contains @c v.
export function bbContainsVect(box: BB, vect: Vect): boolean {
    return (
        box.l <= vect.x &&
        box.r >= vect.x &&
        box.b <= vect.y &&
        box.t >= vect.y
    );
}

export function bbContainsVect2(
    l: number, b: number, r: number, t: number,
    vect: Vect,
): boolean {
    return (
        l <= vect.x &&
        r >= vect.x &&
        b <= vect.y &&
        t >= vect.y
    );
}

/// Returns a bounding box that holds both bounding boxes.
export function bbMerge(boxA: BB, boxB: BB): BB {
    return new BB(
        Math.min(boxA.l, boxB.l),
        Math.min(boxA.b, boxB.b),
        Math.max(boxA.r, boxB.r),
        Math.max(boxA.t, boxB.t),
    );
}

/// Returns a bounding box that holds both @c bb and @c v.
export function bbExpand(box: BB, vect: Vect): BB {
    return new BB(
        Math.min(box.l, vect.x),
        Math.min(box.b, vect.y),
        Math.max(box.r, vect.x),
        Math.max(box.t, vect.y),
    );
}

/// Returns the area of the bounding box.
export function bbArea(box: BB): number {
    return (box.r - box.l) * (box.t - box.b);
}

/// Merges @c a and @c b and returns the area of the merged bounding box.
export function bbMergedArea(boxA: BB, boxB: BB): number {
    return (
        (Math.max(boxA.r, boxB.r) - Math.min(boxA.l, boxB.l)) *
        (Math.max(boxA.t, boxB.t) - Math.min(boxA.b, boxB.b))
    );
}

export function bbMergedArea2(
    box: BB,
    l: number, b: number, r: number, t: number,
): number {
    return (
        (Math.max(box.r, r) - Math.min(box.l, l)) *
        (Math.max(box.t, t) - Math.min(box.b, b))
    );
}

/// Clamp a vector to a bounding box.
export function bbClampVect(box: BB, vect: Vect): Vect {
    const x = Math.min(Math.max(box.l, vect.x), box.r);
    const y = Math.min(Math.max(box.b, vect.y), box.t);
    return new Vect(x, y);
}

// TODO edge case issue
/// Wrap a vector to a bounding box.
export function bbWrapVect(box: BB, vect: Vect): Vect {
    const ix = Math.abs(box.r - box.l);
    const modx = (vect.x - box.l) % ix;
    const x = (modx > 0) ? modx : modx + ix;

    const iy = Math.abs(box.t - box.b);
    const mody = (vect.y - box.b) % iy;
    const y = (mody > 0) ? mody : mody + iy;

    return new Vect(x + box.l, y + box.b);
}
