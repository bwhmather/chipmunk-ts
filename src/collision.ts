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

import { CircleShape, PolyShape, SegmentShape, Shape } from "./shapes";
import { SplittingPlane } from "./shapes/poly-shape";
import { assert, clamp01, hashPair } from "./util";
import {
    vadd, vcross, vcross2,
    vdot, vdot2, Vect,
    vlengthsq, vmult,
    vneg, vsub,
    vzero,
} from "./vect";

let numContacts = 0;

export class Contact {
    // The contact point.
    p: Vect;
    // The contact normal.
    n: Vect;
    dist: number;
    r1: Vect;
    r2: Vect;
    nMass: number = 0;
    tMass: number = 0;
    bounce: number = 0;
    bias: number = 0;
    jnAcc: number = 0;
    jtAcc: number = 0;
    jBias: number = 0;

    hash: any;

    constructor(p: Vect, n: Vect, dist: number, hash: any) {
        this.p = p;
        this.n = n;
        this.dist = dist;

        this.r1 = this.r2 = vzero;

        this.hash = hash;
        numContacts++;
    }
}

// Add contact points for circle to circle collisions.
// Used by several collision tests.
function circle2circleQuery(
    p1: Vect, p2: Vect, r1: number, r2: number,
): Contact {
    const mindist = r1 + r2;
    const delta = vsub(p2, p1);
    const distsq = vlengthsq(delta);
    if (distsq >= mindist * mindist) {
        return;
    }

    const dist = Math.sqrt(distsq);

    // Allocate and initialize the contact.
    return new Contact(
        vadd(p1, vmult(
            delta, 0.5 + (r1 - 0.5 * mindist) / (dist ? dist : Infinity),
        )),
        (dist ? vmult(delta, 1 / dist) : new Vect(1, 0)),
        dist - mindist,
        0,
    );
}

// Collide circle shapes.
function circle2circle(circ1: CircleShape, circ2: CircleShape) {
    const contact = circle2circleQuery(
        circ1.tc, circ2.tc, circ1.radius, circ2.radius,
    );
    return contact ? [contact] : [];
}

function circle2segment(circleShape: CircleShape, segmentShape: SegmentShape) {
    const segA = segmentShape.ta;
    const segB = segmentShape.tb;
    const center = circleShape.tc;

    const segDelta = vsub(segB, segA);
    const closestT = clamp01(
        vdot(segDelta, vsub(center, segA)) / vlengthsq(segDelta),
    );
    const closest = vadd(segA, vmult(segDelta, closestT));

    const contact = circle2circleQuery(
        center, closest, circleShape.radius, segmentShape.r,
    );
    if (contact) {
        const n = contact.n;

        // Reject endcap collisions if tangents are provided.
        return (
            (closestT === 0 && vdot(n, segmentShape.tangentA) < 0) ||
            (closestT === 1 && vdot(n, segmentShape.tangentB) < 0)
        ) ? [] : [contact];
    } else {
        return [];
    }
}

function segment2segment(seg1: SegmentShape, seg2: SegmentShape): Contact[] {
    return [];
}

// Find the minimum separating axis for the given poly and axis list.
//
// This function needs to return two values - the index of the min. separating
// axis and the value itself. Short of inlining MSA, returning values through a
// global like this is the fastest implementation.
//
// See: http://jsperf.com/return-two-values-from-function/2
let lastMsaMin = 0;
function findMSA(poly: PolyShape, planes: SplittingPlane[]) {
    let indexMin = 0;
    let min = poly.valueOnAxis(planes[0].n, planes[0].d);
    if (min > 0) {
        return -1;
    }

    for (let i = 1; i < planes.length; i++) {
        const dist = poly.valueOnAxis(planes[i].n, planes[i].d);
        if (dist > 0) {
            return -1;
        } else if (dist > min) {
            min = dist;
            indexMin = i;
        }
    }

    lastMsaMin = min;
    return indexMin;
}

// Add contacts for probably penetrating vertexes.
// This handles the degenerate case where an overlap was detected, but no
// vertexes fall inside the opposing polygon. (like a star of david)
function findVertsFallback(
    poly1: PolyShape, poly2: PolyShape, n: Vect, dist: number,
): Contact[] {
    const arr = [];

    const verts1 = poly1.tVerts;
    for (let i = 0; i < verts1.length; i += 2) {
        const vx = verts1[i];
        const vy = verts1[i + 1];
        if (poly2.containsVertPartial(vx, vy, vneg(n))) {
            arr.push(new Contact(
                new Vect(vx, vy), n, dist, hashPair(poly1.hashid, i)),
            );
        }
    }

    const verts2 = poly2.tVerts;
    for (let i = 0; i < verts2.length; i += 2) {
        const vx = verts2[i];
        const vy = verts2[i + 1];
        if (poly1.containsVertPartial(vx, vy, n)) {
            arr.push(new Contact(
                new Vect(vx, vy), n, dist, hashPair(poly2.hashid, i)),
            );
        }
    }

    return arr;
}

// Add contacts for penetrating vertexes.
function findVerts(
    poly1: PolyShape, poly2: PolyShape, n: Vect, dist: number,
): Contact[] {
    const arr = [];

    const verts1 = poly1.tVerts;
    for (let i = 0; i < verts1.length; i += 2) {
        const vx = verts1[i];
        const vy = verts1[i + 1];
        if (poly2.containsVert(vx, vy)) {
            arr.push(new Contact(
                new Vect(vx, vy), n, dist, hashPair(poly1.hashid, i >> 1)),
            );
        }
    }

    const verts2 = poly2.tVerts;
    for (let i = 0; i < verts2.length; i += 2) {
        const vx = verts2[i];
        const vy = verts2[i + 1];
        if (poly1.containsVert(vx, vy)) {
            arr.push(new Contact
                (new Vect(vx, vy), n, dist, hashPair(poly2.hashid, i >> 1),
            ));
        }
    }

    return (arr.length ? arr : findVertsFallback(poly1, poly2, n, dist));
}

// Collide poly shapes together.
function poly2poly(poly1: PolyShape, poly2: PolyShape): Contact[] {
    const mini1 = findMSA(poly2, poly1.tPlanes);
    if (mini1 === -1) {
        return [];
    }
    const min1 = lastMsaMin;

    const mini2 = findMSA(poly1, poly2.tPlanes);
    if (mini2 === -1) {
        return [];
    }
    const min2 = lastMsaMin;

    // There is overlap, find the penetrating verts
    if (min1 > min2) {
        return findVerts(poly1, poly2, poly1.tPlanes[mini1].n, min1);
    } else {
        return findVerts(poly1, poly2, vneg(poly2.tPlanes[mini2].n), min2);
    }
}

// Like cpPolyValueOnAxis(), but for segments.
function segValueOnAxis(seg: SegmentShape, n: Vect, d: number) {
    const a = vdot(n, seg.ta) - seg.r;
    const b = vdot(n, seg.tb) - seg.r;
    return Math.min(a, b) - d;
}

// Identify vertexes that have penetrated the segment.
function findPointsBehindSeg(
    arr: Contact[], seg: SegmentShape, poly: PolyShape,
    pDist: number, coef: number,
): void {
    const dta = vcross(seg.tn, seg.ta);
    const dtb = vcross(seg.tn, seg.tb);
    const n = vmult(seg.tn, coef);

    const verts = poly.tVerts;
    for (let i = 0; i < verts.length; i += 2) {
        const vx = verts[i];
        const vy = verts[i + 1];
        if (vdot2(vx, vy, n.x, n.y) < vdot(seg.tn, seg.ta) * coef + seg.r) {
            const dt = vcross2(seg.tn.x, seg.tn.y, vx, vy);
            if (dta >= dt && dt >= dtb) {
                arr.push(new Contact(
                    new Vect(vx, vy), n, pDist, hashPair(poly.hashid, i),
                ));
            }
        }
    }
}

// This one is complicated and gross. Just don't go there...
// TODO: Comment me!
function segment2poly(seg: SegmentShape, poly: PolyShape): Contact[] {
    const arr = [];

    const planes = poly.tPlanes;
    const numVerts = planes.length;

    const segD = vdot(seg.tn, seg.ta);
    const minNorm = poly.valueOnAxis(seg.tn, segD) - seg.r;
    const minNeg = poly.valueOnAxis(vneg(seg.tn), -segD) - seg.r;
    if (minNeg > 0 || minNorm > 0) {
        return [];
    }

    let mini = 0;
    let polyMin = segValueOnAxis(seg, planes[0].n, planes[0].d);
    if (polyMin > 0) {
        return [];
    }
    for (let i = 0; i < numVerts; i++) {
        const dist = segValueOnAxis(seg, planes[i].n, planes[i].d);
        if (dist > 0) {
            return [];
        } else if (dist > polyMin) {
            polyMin = dist;
            mini = i;
        }
    }

    const polyN = vneg(planes[mini].n);

    const va = vadd(seg.ta, vmult(polyN, seg.r));
    const vb = vadd(seg.tb, vmult(polyN, seg.r));
    if (poly.containsVert(va.x, va.y)) {
        arr.push(new Contact(va, polyN, polyMin, hashPair(seg.hashid, 0)));
    }
    if (poly.containsVert(vb.x, vb.y)) {
        arr.push(new Contact(vb, polyN, polyMin, hashPair(seg.hashid, 1)));
    }

    // Floating point precision problems here.
    // This will have to do for now.
    // poly_min -= cp_collision_slop; // TODO is this needed anymore?

    if (minNorm >= polyMin || minNeg >= polyMin) {
        if (minNorm > minNeg) {
            findPointsBehindSeg(arr, seg, poly, minNorm, 1);
        } else {
            findPointsBehindSeg(arr, seg, poly, minNeg, -1);
        }
    }

    // If no other collision points are found, try colliding endpoints.
    if (arr.length === 0) {
        const mini2 = mini * 2;
        const verts = poly.tVerts;

        const polyA = new Vect(
            verts[mini2], verts[mini2 + 1],
        );
        let con;

        con = circle2circleQuery(seg.ta, polyA, seg.r, 0);
        if (con) { return [con]; }

        con = circle2circleQuery(seg.tb, polyA, seg.r, 0);
        if (con) { return [con]; }

        const len = numVerts * 2;
        const polyB = new Vect(
            verts[(mini2 + 2) % len], verts[(mini2 + 3) % len],
        );
        con = circle2circleQuery(seg.ta, polyB, seg.r, 0);
        if (con) { return [con]; }

        con = circle2circleQuery(seg.tb, polyB, seg.r, 0);
        if (con) { return [con]; }
    }

    // console.log(poly.tVerts, poly.tPlanes);
    // console.log('seg2poly', arr);
    return arr;
}

// This one is less gross, but still gross.
// TODO: Comment me!
function circle2poly(circ: CircleShape, poly: PolyShape): Contact[] {
    const planes = poly.tPlanes;

    let mini = 0;
    let min = vdot(planes[0].n, circ.tc) - planes[0].d - circ.radius;
    for (let i = 0; i < planes.length; i++) {
        const dist = vdot(planes[i].n, circ.tc) - planes[i].d - circ.radius;
        if (dist > 0) {
            return [];
        } else if (dist > min) {
            min = dist;
            mini = i;
        }
    }

    const n = planes[mini].n;

    const verts = poly.tVerts;
    const len = verts.length;
    const mini2 = mini << 1;

    const ax = verts[mini2];
    const ay = verts[mini2 + 1];
    const bx = verts[(mini2 + 2) % len];
    const by = verts[(mini2 + 3) % len];

    const dta = vcross2(n.x, n.y, ax, ay);
    const dtb = vcross2(n.x, n.y, bx, by);
    const dt = vcross(n, circ.tc);

    if (dt < dtb) {
        const con = circle2circleQuery(
            circ.tc, new Vect(bx, by), circ.radius, 0,
        );
        return con ? [con] : [];
    } else if (dt < dta) {
        return [new Contact(
            vsub(circ.tc, vmult(n, circ.radius + min / 2)),
            vneg(n),
            min,
            0,
        )];
    } else {
        const con = circle2circleQuery(
            circ.tc, new Vect(ax, ay), circ.radius, 0,
        );
        return con ? [con] : [];
    }
}

// The javascripty way to do this would be either nested object or methods on
// the prototypes.
//
// However, the *fastest* way is the method below.
// See: http://jsperf.com/dispatch

// These are copied from the prototypes into the actual objects in the Shape
// constructor.
CircleShape.prototype.collisionCode = 0;
SegmentShape.prototype.collisionCode = 1;
PolyShape.prototype.collisionCode = 2;

CircleShape.prototype.collisionTable = [
    circle2circle,
    circle2segment,
    circle2poly,
];

SegmentShape.prototype.collisionTable = [
    null,
    segment2segment,
    segment2poly,
];

PolyShape.prototype.collisionTable = [
    null,
    null,
    poly2poly,
];

export function collideShapes(a: Shape, b: Shape) {
    assert(
        a.collisionCode <= b.collisionCode,
        "Collided shapes must be sorted by type",
    );
    return a.collisionTable[b.collisionCode](a, b);
}
