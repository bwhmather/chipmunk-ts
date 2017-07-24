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

import { Body } from "../body";
import { Vect, vlength2, vrotate } from "../vect";
import { NearestPointQueryInfo, SegmentQueryInfo, Shape } from "./base";
import { circleSegmentQuery } from "./util";

export class CircleShape extends Shape {
    c: Vect;
    tc: Vect;

    bb_l: number;
    bb_b: number;
    bb_r: number;
    bb_t: number;

    r: number;

    constructor(body: Body, radius: number, offset: Vect) {
        super(body);

        this.c = this.tc = offset;
        this.r = radius;

        this.type = "circle";
    }

    cacheData(p: Vect, rot: Vect): void {
        //var c = this.tc = vadd(p, vrotate(this.c, rot));
        const c = this.tc = vrotate(this.c, rot).add(p);
        //this.bb = bbNewForCircle(c, this.r);
        const r = this.r;
        this.bb_l = c.x - r;
        this.bb_b = c.y - r;
        this.bb_r = c.x + r;
        this.bb_t = c.y + r;
    }

    nearestPointQuery(p: Vect): NearestPointQueryInfo {
        const deltax = p.x - this.tc.x;
        const deltay = p.y - this.tc.y;
        const d = vlength2(deltax, deltay);
        const r = this.r;

        const nearestp = new Vect(this.tc.x + deltax * r / d, this.tc.y + deltay * r / d);
        return new NearestPointQueryInfo(this, nearestp, d - r);
    }

    segmentQuery(a: Vect, b: Vect): SegmentQueryInfo {
        return circleSegmentQuery(this, this.tc, this.r, a, b);
    }
}
