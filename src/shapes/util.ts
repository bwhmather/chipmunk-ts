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
    Vect, vsub,
    vmult, vcross, vdot,
    vlerp, vnormalize,
} from '../vect';
import { Shape, SegmentQueryInfo } from './base'


export function circleSegmentQuery(
    shape: Shape, center: Vect, r: number, a: Vect, b: Vect,
) {
    // offset the line to be relative to the circle
    a = vsub(a, center);
    b = vsub(b, center);

    const qa = vdot(a, a) - 2 * vdot(a, b) + vdot(b, b);
    const qb = -2 * vdot(a, a) + 2 * vdot(a, b);
    const qc = vdot(a, a) - r * r;

    const det = qb * qb - 4 * qa * qc;

    if (det >= 0) {
        const t = (-qb - Math.sqrt(det)) / (2 * qa);
        if (0 <= t && t <= 1) {
            return new SegmentQueryInfo(shape, t, vnormalize(vlerp(a, b, t)));
        }
    }
};