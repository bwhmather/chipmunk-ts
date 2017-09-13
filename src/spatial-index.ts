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

/**
 * @defgroup cpSpatialIndex cpSpatialIndex
 *
 * Spatial indexes are data structures that are used to accelerate collision
 * detection and spatial queries. Chipmunk provides a number of spatial index
 * algorithms to pick from and they are programmed in a generic way so that you
 * can use them for holding more than just Shapes.
 *
 * It works by using pointers to the objects you add and using a callback to ask
 * your code for bounding boxes when it needs them. Several types of queries can
 * be performed an index as well as reindexing and full collision information.
 * All communication to the spatial indexes is performed through callback
 * functions.
 *
 * Spatial indexes should be treated as opaque structs. This means you shouldn't
 * be reading any of the fields directly.
 */
import { BB } from "./bb";
import { Shape } from "./shapes";
import { Vect } from "./vect";


export abstract class SpatialIndex {

    // The number of objects in the spatial index.
    count: number = 0;

    // Returns true if the spatial index contains the given object.
    // Most spatial indexes use hashed storage, so you must provide a hash
    // value too.
    abstract contains(obj: Shape): boolean;

    // Add an object to a spatial index.
    abstract insert(obj: Shape): void;

    // Insert a static object into the spatial index.
    abstract insertStatic(obj: Shape): void;

    // Remove an object from a spatial index.
    abstract remove(obj: Shape): void;

    // Reindex a single object in the spatial index.
    reindexObject(obj: Shape): void {
        // Pass.
    }

    // Perform a re-index of all active shapes in the spatial index.
    reindex(): void {
        // Pass.
    }

    // Perform a full re-index of all static and active shapes in the spatial
    // index.
    reindexStatic(): void {
        // Pass.
    }

    // Finds all potentially intersecting shapes in the index.  `func` will be
    // called once for each candidate pair.  Shapes marked as static cannot
    // collide with each other.
    abstract touchingQuery(
        func: (a: Shape, b: Shape) => any,
    ): void;

    // Perform a point query against the spatial index, calling `func` for each
    // potential match. A pointer to the point will be passed as `obj1` of
    // `func`.
    abstract pointQuery(
        point: Vect,
        func: (obj: Shape) => any,
    ): void;

    // Perform a segment query against the spatial index, calling @c func for
    // each potential match.
    abstract segmentQuery(
        vectA: Vect, vectB: Vect, tExit: number,
        func: (obj: Shape) => any,
    ): void;

    // Perform a rectangle query against the spatial index, calling @c func for
    // each potential match.
    abstract query(
        bb: BB,
        func: (obj: Shape) => any,
    ): void;

    // Iterate the objects in the spatial index. @c func will be called once
    // for each object.
    abstract each(
        f: (obj: Shape) => any,
    ): void;
}
