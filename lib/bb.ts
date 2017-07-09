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

/// Chipmunk's axis-aligned 2D bounding box type along with a few handy routines.

let numBB = 0;

// Bounding boxes are JS objects with {l, b, r, t} = left, bottom, right, top, respectively.
export class BB {
    constructor(l, b, r, t) {
        this.l = l;
        this.b = b;
        this.r = r;
        this.t = t;

        numBB++;
    }
}

export function bb(l, b, r, t) {
    return new BB(l, b, r, t);
}

export function bbNewForCircle({ x, y }, r) {
    return new BB(
        x - r,
        y - r,
        x + r,
        y + r
    );
};

/// Returns true if @c a and @c b intersect.
export function bbIntersects(a, b) {
    return (a.l <= b.r && b.l <= a.r && a.b <= b.t && b.b <= a.t);
};
export function bbIntersects2(bb, l, b, r, t) {
    return (bb.l <= r && l <= bb.r && bb.b <= t && b <= bb.t);
};

/// Returns true if @c other lies completely within @c bb.
export function bbContainsBB({ l, r, b, t }, { l, r, b, t }) {
    return l <= l && r >= r && b <= b && t >= t;
};

/// Returns true if @c bb contains @c v.
export function bbContainsVect({ l, r, b, t }, { x, y }) {
    return l <= x && r >= x && b <= y && t >= y;
};
export function bbContainsVect2(l, b, r, t, { x, y }) {
    return l <= x && r >= x && b <= y && t >= y;
};

/// Returns a bounding box that holds both bounding boxes.
export function bbMerge(a, b) {
    return new BB(
        min(a.l, b.l),
        min(a.b, b.b),
        max(a.r, b.r),
        max(a.t, b.t)
    );
};

/// Returns a bounding box that holds both @c bb and @c v.
export function bbExpand({ l, b, r, t }, { x, y }) {
    return new BB(
        min(l, x),
        min(b, y),
        max(r, x),
        max(t, y)
    );
};

/// Returns the area of the bounding box.
export function bbArea({ r, l, t, b }) {
    return (r - l) * (t - b);
};

/// Merges @c a and @c b and returns the area of the merged bounding box.
export function bbMergedArea(a, b) {
    return (max(a.r, b.r) - min(a.l, b.l)) * (max(a.t, b.t) - min(a.b, b.b));
};

export function bbMergedArea2(bb, l, b, r, t) {
    return (max(bb.r, r) - min(bb.l, l)) * (max(bb.t, t) - min(bb.b, b));
};

/// Return true if the bounding box intersects the line segment with ends @c a and @c b.
export function bbIntersectsSegment(bb, a, b) {
    return (bbSegmentQuery(bb, a, b) != Infinity);
};

/// Clamp a vector to a bounding box.
export function bbClampVect({ l, r, b, t }, v) {
    const x = min(max(l, v.x), r);
    const y = min(max(b, v.y), t);
    return new Vect(x, y);
};

// TODO edge case issue
/// Wrap a vector to a bounding box.
export function bbWrapVect({ r, l, t, b }, v) {
    const ix = Math.abs(r - l);
    const modx = (v.x - l) % ix;
    const x = (modx > 0) ? modx : modx + ix;

    const iy = Math.abs(t - b);
    const mody = (v.y - b) % iy;
    const y = (mody > 0) ? mody : mody + iy;

    return new Vect(x + l, y + b);
};
