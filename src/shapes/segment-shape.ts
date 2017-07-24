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

import { assert, closestPointOnSegment } from "../util";
import {
    vadd, vcross,
    vdist, vdot,
    Vect, vlength2, vlerp,
    vmult, vneg,
    vnormalize, vperp,
    vrotate, vsub, vzero,
} from "../vect";

import { BB } from "../bb";
import { Body } from "../body";
import { Space } from "../space";
import { NearestPointQueryInfo, SegmentQueryInfo, Shape } from "./base";
import { circleSegmentQuery } from "./util";

export class SegmentShape extends Shape {
    a: Vect;
    b: Vect;
    n: Vect;

    // TODO
    ta: Vect;
    tb: Vect;
    tn: Vect;

    r: number;

    tangentA: Vect;
    tangentB: Vect;

    constructor(body: Body, a: Vect, b: Vect, r: number) {
        super(body);

        this.a = a;
        this.b = b;
        this.n = vperp(vnormalize(vsub(b, a)));

        this.ta = this.tb = this.tn = null;

        this.r = r;

        this.tangentA = vzero;
        this.tangentB = vzero;

        this.type = "segment";
    }

    cacheData(p: Vect, rot: Vect): void {
        this.ta = vadd(p, vrotate(this.a, rot));
        this.tb = vadd(p, vrotate(this.b, rot));
        this.tn = vrotate(this.n, rot);

        let l;
        let r;
        let b;
        let t;

        if (this.ta.x < this.tb.x) {
            l = this.ta.x;
            r = this.tb.x;
        } else {
            l = this.tb.x;
            r = this.ta.x;
        }

        if (this.ta.y < this.tb.y) {
            b = this.ta.y;
            t = this.tb.y;
        } else {
            b = this.tb.y;
            t = this.ta.y;
        }

        const rad = this.r;

        this.bbL = l - rad;
        this.bbB = b - rad;
        this.bbR = r + rad;
        this.bbT = t + rad;
    }

    nearestPointQuery(p: Vect): NearestPointQueryInfo {
        const closest = closestPointOnSegment(p, this.ta, this.tb);

        const deltax = p.x - closest.x;
        const deltay = p.y - closest.y;
        const d = vlength2(deltax, deltay);
        const r = this.r;

        const nearestp = (
            d ? vadd(closest, vmult(new Vect(deltax, deltay), r / d)) : closest
        );
        return new NearestPointQueryInfo(this, nearestp, d - r);
    }

    segmentQuery(a: Vect, b: Vect): SegmentQueryInfo {
        const n = this.tn;
        const d = vdot(vsub(this.ta, a), n);
        const r = this.r;

        const flippedNormal = (d > 0 ? vneg(n) : n);
        const normalOffset = vsub(vmult(flippedNormal, r), a);

        const segA = vadd(this.ta, normalOffset);
        const segB = vadd(this.tb, normalOffset);
        const delta = vsub(b, a);

        if (vcross(delta, segA) * vcross(delta, segB) <= 0) {
            const dOffset = d + (d > 0 ? -r : r);
            const ad = -dOffset;
            const bd = vdot(delta, n) - dOffset;

            if (ad * bd < 0) {
                return new SegmentQueryInfo(
                    this, ad / (ad - bd), flippedNormal,
                );
            }
        } else if (r !== 0) {
            const info1 = circleSegmentQuery(this, this.ta, this.r, a, b);
            const info2 = circleSegmentQuery(this, this.tb, this.r, a, b);

            if (info1) {
                return info2 && info2.t < info1.t ? info2 : info1;
            } else {
                return info2;
            }
        }
    }

    setNeighbors(prev: Vect, next: Vect): void {
        this.tangentA = vsub(prev, this.a);
        this.tangentB = vsub(next, this.b);
    }

    setEndpoints(a: Vect, b: Vect): void {
        this.a = a;
        this.b = b;
        this.n = vperp(vnormalize(vsub(b, a)));
    }
}
