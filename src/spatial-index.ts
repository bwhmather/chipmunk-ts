/* Copyright (c) 2010 Scott Lembcke
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
	@defgroup cpSpatialIndex cpSpatialIndex
	
	Spatial indexes are data structures that are used to accelerate collision detection
	and spatial queries. Chipmunk provides a number of spatial index algorithms to pick from
	and they are programmed in a generic way so that you can use them for holding more than
	just Shapes.
	
	It works by using pointers to the objects you add and using a callback to ask your code
	for bounding boxes when it needs them. Several types of queries can be performed an index as well
	as reindexing and full collision information. All communication to the spatial indexes is performed
	through callback functions.
	
	Spatial indexes should be treated as opaque structs.
	This means you shouldn't be reading any of the fields directly.
 */
import { BB } from './bb';
import { Vect } from './vect';
import { Shape } from './shapes';


export abstract class SpatialIndex {

    // The number of objects in the spatial index.
    count: number;

    staticIndex: SpatialIndex;
    dynamicIndex: SpatialIndex;

    constructor(staticIndex: SpatialIndex) {
        this.staticIndex = staticIndex;

        if (staticIndex) {
            if (staticIndex.dynamicIndex) {
                throw new Error("This static index is already associated with a dynamic index.");
            }
            staticIndex.dynamicIndex = this;
        }
    }

    // Collide the objects in an index against the objects in a staticIndex using the query callback function.
    protected collideStatic(
        staticIndex: SpatialIndex,
        func: (a: Shape, b: Shape) => any,
    ) {
        if (staticIndex.count > 0) {
            this.each((obj) => {
                staticIndex.query(
                    new BB(obj.bb_l, obj.bb_b, obj.bb_r, obj.bb_t),
                    func as any,  // TODO TODO TODO
                );
            });
        }
    }

    // Returns true if the spatial index contains the given object.
    // Most spatial indexes use hashed storage, so you must provide a hash value too.
    abstract contains(obj: Shape): boolean;

    // Add an object to a spatial index.
    abstract insert(obj: Shape): void;

    // Remove an object from a spatial index.
    abstract remove(obj: Shape): void;

    // Perform a full reindex of a spatial index.
    reindex(): void { }

    // Reindex a single object in the spatial index.
    reindexObject(obj: Shape): void { }

    // Perform a point query against the spatial index, calling @c func for each potential match.
    // A pointer to the point will be passed as @c obj1 of @c func.
    // func(shape);
    abstract pointQuery(
        point: Vect,
        func: (obj: Shape) => any,
    ): void;

    // Perform a segment query against the spatial index, calling @c func for each potential match.
    // func(shape);
    abstract segmentQuery(
        vect_a: Vect, vect_b: Vect, t_exit: number,
        func: (obj: Shape) => any,
    ): void;

    // Perform a rectangle query against the spatial index, calling @c func for each potential match.
    // func(shape);
    abstract query(
        bb: BB,
        func: (obj: Shape) => any,
    ): void;

    // Simultaneously reindex and find all colliding objects.
    // @c func will be called once for each potentially overlapping pair of objects found.
    // If the spatial index was initialized with a static index, it will collide it's objects against that as well.
    abstract reindexQuery(
        func: (a: Shape, b: Shape) => any,
    ): void;

    // Iterate the objects in the spatial index. @c func will be called once for each object.
    abstract each(
        f: (obj: Shape) => any,
    ): void;
}


