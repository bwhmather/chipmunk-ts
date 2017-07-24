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

import { BB } from '../bb';
import { Shape, NearestPointQueryInfo, SegmentQueryInfo } from './base'
import { assert, closestPointOnSegment2 } from '../util';
import { Body } from '../body';
import {
    Vect, vzero,
    vcross, vcross2,
    vdot, vdot2,
    vdist, vlerp,
    vnormalize,
    vperp, vrotate,
} from '../vect';

/// Check that a set of vertexes is convex and has a clockwise winding.
function polyValidate(verts: number[]): boolean {
    const len = verts.length;
    for (let i = 0; i < len; i += 2) {
        const ax = verts[i];
        const ay = verts[i + 1];
        const bx = verts[(i + 2) % len];
        const by = verts[(i + 3) % len];
        const cx = verts[(i + 4) % len];
        const cy = verts[(i + 5) % len];

        //if(vcross(vsub(b, a), vsub(c, b)) > 0){
        if (vcross2(bx - ax, by - ay, cx - bx, cy - by) > 0) {
            return false;
        }
    }

    return true;
}

export class SplittingPlane {
    n: Vect;
    d: number;

    constructor(n: Vect, d: number) {
        this.n = n;
        this.d = d;
    }

    compare(v: Vect): number {
        return vdot(this.n, v) - this.d;
    }
}

/// Initialize a polygon shape.
/// The vertexes must be convex and have a clockwise winding.
export class PolyShape extends Shape {
    verts: number[]
    tVerts: number[]
    // TODO
    planes: SplittingPlane[];
    tPlanes: SplittingPlane[];

    constructor(body: Body, verts: number[], offset: Vect) {
        super(body);
        this.setVerts(verts, offset);
        this.type = 'poly';
    }

    setVerts(verts: number[], offset: Vect): void {
        assert(verts.length >= 4, "Polygons require some verts");
        assert(typeof (verts[0]) === 'number',
            'Polygon verticies should be specified in a flattened list (eg [x1,y1,x2,y2,x3,y3,...])');

        // Fail if the user attempts to pass a concave poly, or a bad winding.
        assert(polyValidate(verts), "Polygon is concave or has a reversed winding. Consider using cpConvexHull()");

        const len = verts.length;
        const numVerts = len >> 1;

        // This a pretty bad way to do this in javascript. As a first pass, I want to keep
        // the code similar to the C.
        this.verts = new Array(len);
        this.tVerts = new Array(len);
        this.planes = new Array(numVerts);
        this.tPlanes = new Array(numVerts);

        for (let i = 0; i < len; i += 2) {
            //var a = vadd(offset, verts[i]);
            //var b = vadd(offset, verts[(i+1)%numVerts]);
            const ax = verts[i] + offset.x;
            const ay = verts[i + 1] + offset.y;
            const bx = verts[(i + 2) % len] + offset.x;
            const by = verts[(i + 3) % len] + offset.y;

            // Inefficient, but only called during object initialization.
            const n = vnormalize(vperp(new Vect(bx - ax, by - ay)));

            this.verts[i] = ax;
            this.verts[i + 1] = ay;
            this.planes[i >> 1] = new SplittingPlane(n, vdot2(n.x, n.y, ax, ay));
            this.tPlanes[i >> 1] = new SplittingPlane(new Vect(0, 0), 0);
        }
    }

    transformVerts(p: Vect, rot: Vect): void {
        const src = this.verts;
        const dst = this.tVerts;

        let l = Infinity;
        let r = -Infinity;
        let b = Infinity;
        let t = -Infinity;

        for (let i = 0; i < src.length; i += 2) {
            //var v = vadd(p, vrotate(src[i], rot));
            const x = src[i];
            const y = src[i + 1];

            const vx = p.x + x * rot.x - y * rot.y;
            const vy = p.y + x * rot.y + y * rot.x;

            //console.log('(' + x + ',' + y + ') -> (' + vx + ',' + vy + ')');

            dst[i] = vx;
            dst[i + 1] = vy;

            l = Math.min(l, vx);
            r = Math.max(r, vx);
            b = Math.min(b, vy);
            t = Math.max(t, vy);
        }

        this.bb_l = l;
        this.bb_b = b;
        this.bb_r = r;
        this.bb_t = t;
    }

    transformAxes(p: Vect, rot: Vect) {
        const src = this.planes;
        const dst = this.tPlanes;

        for (let i = 0; i < src.length; i++) {
            const n = vrotate(src[i].n, rot);
            dst[i].n = n;
            dst[i].d = vdot(p, n) + src[i].d;
        }
    }

    cacheData(p: Vect, rot: Vect) {
        this.transformAxes(p, rot);
        this.transformVerts(p, rot);
    }

    nearestPointQuery(p: Vect): NearestPointQueryInfo {
        const planes = this.tPlanes;
        const verts = this.tVerts;

        let v0x = verts[verts.length - 2];
        let v0y = verts[verts.length - 1];
        let minDist = Infinity;
        let closestPoint = vzero;
        let outside = false;

        for (let i = 0; i < planes.length; i++) {
            if (planes[i].compare(p) > 0) outside = true;

            const v1x = verts[i * 2];
            const v1y = verts[i * 2 + 1];
            const closest = closestPointOnSegment2(p.x, p.y, v0x, v0y, v1x, v1y);

            const dist = vdist(p, closest);
            if (dist < minDist) {
                minDist = dist;
                closestPoint = closest;
            }

            v0x = v1x;
            v0y = v1y;
        }

        return new NearestPointQueryInfo(this, closestPoint, (outside ? minDist : -minDist));
    }

    segmentQuery(a: Vect, b: Vect): SegmentQueryInfo {
        const axes = this.tPlanes;
        const verts = this.tVerts;
        const numVerts = axes.length;
        const len = numVerts * 2;

        for (let i = 0; i < numVerts; i++) {
            const n = axes[i].n;
            const an = vdot(a, n);
            if (axes[i].d > an) continue;

            const bn = vdot(b, n);
            const t = (axes[i].d - an) / (bn - an);
            if (t < 0 || 1 < t) continue;

            const point = vlerp(a, b, t);
            const dt = -vcross(n, point);
            const dtMin = -vcross2(n.x, n.y, verts[i * 2], verts[i * 2 + 1]);
            const dtMax = -vcross2(n.x, n.y, verts[(i * 2 + 2) % len], verts[(i * 2 + 3) % len]);

            if (dtMin <= dt && dt <= dtMax) {
                // josephg: In the original C code, this function keeps
                // looping through axes after finding a match. I *think*
                // this code is equivalent...
                return new SegmentQueryInfo(this, t, n);
            }
        }
    }

    valueOnAxis(n: Vect, d: number): number {
        const verts = this.tVerts;
        let m = vdot2(n.x, n.y, verts[0], verts[1]);

        for (let i = 2; i < verts.length; i += 2) {
            m = Math.min(m, vdot2(n.x, n.y, verts[i], verts[i + 1]));
        }

        return m - d;
    }

    containsVert(vx: number, vy: number): boolean {
        const planes = this.tPlanes;

        for (let i = 0; i < planes.length; i++) {
            const n = planes[i].n;
            const dist = vdot2(n.x, n.y, vx, vy) - planes[i].d;
            if (dist > 0) return false;
        }

        return true;
    }

    containsVertPartial(vx: number, vy: number, n: Vect) {
        const planes = this.tPlanes;

        for (let i = 0; i < planes.length; i++) {
            const n2 = planes[i].n;
            if (vdot(n2, n) < 0) continue;
            const dist = vdot2(n2.x, n2.y, vx, vy) - planes[i].d;
            if (dist > 0) return false;
        }

        return true;
    }

    // These methods are provided for API compatibility with Chipmunk. I recommend against using
    // them - just access the poly.verts list directly.
    getNumVerts(): number {
        return this.verts.length / 2;
    }

    getVert(i: number): Vect {
        return new Vect(this.verts[i * 2], this.verts[i * 2 + 1]);
    }
}