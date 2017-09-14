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

import { BB } from "./bb";
import { Shape } from "./shapes";
import { SpatialIndex } from "./spatial-index";
import { assertSoft } from "./util";
import { Vect, vmult } from "./vect";

interface INode {
    parent: Branch;

    bbL: number;
    bbB: number;
    bbR: number;
    bbT: number;

    insert(leaf: Leaf): INode;

    markIfTouching(leaf: Leaf, tree: BBTree): void;

    bbArea(): number;

    query(bb: BB, func: (shape: Shape) => any): void;

    /// Returns the fraction along the segment query the node hits. Returns
    /// Infinity if it doesn't hit.
    segmentQuery(
        a: Vect, b: Vect, tExit: number,
        func: (shape: Shape) => any,
    ): number;
}

class Branch implements INode {
    bbL: number;
    bbB: number;
    bbR: number;
    bbT: number;

    parent: Branch;
    childA: INode;
    childB: INode;

    constructor(a: INode, b: INode) {
        this.bbL = Math.min(a.bbL, b.bbL);
        this.bbB = Math.min(a.bbB, b.bbB);
        this.bbR = Math.max(a.bbR, b.bbR);
        this.bbT = Math.max(a.bbT, b.bbT);
        this.parent = null;

        this.setA(a);
        this.setB(b);
    }

    setA(value: INode): void {
        this.childA = value;
        value.parent = this;
    }

    setB(value: INode): void {
        this.childB = value;
        value.parent = this;
    }

    otherChild(child: INode): INode {
        return (this.childA === child ? this.childB : this.childA);
    }

    replaceChild(child: INode, value: INode): void {
        assertSoft(
            child === this.childA || child === this.childB,
            "Node is not a child of parent.",
        );

        if (this.childA === child) {
            this.setA(value);
        } else {
            this.setB(value);
        }

        for (let branch: Branch = this; branch; branch = branch.parent) {
            // node.bb = bbMerge(node.A.bb, node.B.bb);
            const a = branch.childA;
            const b = branch.childB;
            branch.bbL = Math.min(a.bbL, b.bbL);
            branch.bbB = Math.min(a.bbB, b.bbB);
            branch.bbR = Math.max(a.bbR, b.bbR);
            branch.bbT = Math.max(a.bbT, b.bbT);
        }
    }

    insert(leaf: Leaf): INode {
        let costA = this.childB.bbArea() + bbTreeMergedArea(this.childA, leaf);
        let costB = this.childA.bbArea() + bbTreeMergedArea(this.childB, leaf);

        if (costA === costB) {
            costA = bbProximity(this.childA, leaf);
            costB = bbProximity(this.childB, leaf);
        }

        if (costB < costA) {
            this.setB(this.childB.insert(leaf));
        } else {
            this.setA(this.childA.insert(leaf));
        }

        this.bbL = Math.min(this.bbL, leaf.bbL);
        this.bbB = Math.min(this.bbB, leaf.bbB);
        this.bbR = Math.max(this.bbR, leaf.bbR);
        this.bbT = Math.max(this.bbT, leaf.bbT);

        return this;
    }

    markIfTouching(leaf: Leaf, tree: BBTree): void {
        if (bbTreeIntersectsNode(leaf, this)) {
            this.childA.markIfTouching(leaf, tree);
            this.childB.markIfTouching(leaf, tree);
        }
    }

    intersectsBB(bb: BB): boolean {
        return (
            this.bbL <= bb.r &&
            bb.l <= this.bbR &&
            this.bbB <= bb.t &&
            bb.b <= this.bbT
        );
    }

    bbArea(): number {
        return (this.bbR - this.bbL) * (this.bbT - this.bbB);
    }

    query(bb: BB, func: (shape: Shape) => any): void {
        // if(bbIntersectsBB(subtree.bb, bb)){
        if (this.intersectsBB(bb)) {
            this.childA.query(bb, func);
            this.childB.query(bb, func);
        }
    }

    /// Returns the fraction along the segment query the node hits. Returns
    /// Infinity if it doesn't hit.
    private childSegmentQuery(
        child: INode, a: Vect, b: Vect,
    ): number {
        const idx = 1 / (b.x - a.x);
        const txA = (child.bbL === a.x ? -Infinity : (child.bbL - a.x) * idx);
        const txB = (child.bbR === a.x ? Infinity : (child.bbR - a.x) * idx);
        const txMin = Math.min(txA, txB);
        const txMax = Math.max(txA, txB);

        const idy = 1 / (b.y - a.y);
        const tyA = (child.bbB === a.y ? -Infinity : (child.bbB - a.y) * idy);
        const tyB = (child.bbT === a.y ? Infinity : (child.bbT - a.y) * idy);
        const tyMin = Math.min(tyA, tyB);
        const tyMax = Math.max(tyA, tyB);

        if (tyMin <= txMax && txMin <= tyMax) {
            const tMin = Math.max(txMin, tyMin);
            const tMax = Math.min(txMax, tyMax);

            if (0.0 <= tMax && tMin <= 1.0) {
                return Math.max(tMin, 0.0);
            }
        }

        return Infinity;
    }

    segmentQuery(
        a: Vect, b: Vect, tExit: number,
        func: (shape: Shape) => any,
    ): number {
        const tA = this.childSegmentQuery(this.childA, a, b);
        const tB = this.childSegmentQuery(this.childB, a, b);

        if (tA < tB) {
            if (tA < tExit) {
                tExit = Math.min(
                    tExit, this.childA.segmentQuery(a, b, tExit, func),
                );
            }
            if (tB < tExit) {
                tExit = Math.min(
                    tExit, this.childB.segmentQuery(a, b, tExit, func),
                );
            }
        } else {
            if (tB < tExit) {
                tExit = Math.min(
                    tExit, this.childB.segmentQuery(a, b, tExit, func),
                );
            }
            if (tA < tExit) {
                tExit = Math.min(
                    tExit, this.childA.segmentQuery(a, b, tExit, func),
                );
            }
        }

        return tExit;
    }
}

class Leaf implements INode {
    bbL: number;
    bbB: number;
    bbR: number;
    bbT: number;
    shape: Shape;
    parent: Branch;
    number: number;
    stamp: number;

    touching: Set<Leaf>;

    constructor(tree: BBTree, shape: Shape) {
        this.shape = shape;
        tree.getBB(shape, this);

        this.parent = null;

        this.stamp = 1;
        this.touching = new Set();
    }

    insert(leaf: Leaf): INode {
        return new Branch(leaf, this);
    }

    clearPairs(tree: BBTree): void {
        this.touching.forEach((other: Leaf) => {
            other.touching.delete(this);
        });

        this.touching.clear();
    }

    markIfTouching(leaf: Leaf, tree: BBTree): void {
        if (leaf === this) {
            return;
        }

        if (bbTreeIntersectsNode(leaf, this)) {
            this.touching.add(leaf);
            leaf.touching.add(this);
        }
    }

    markTouching(tree: BBTree): void {
        if (this.stamp === tree.getStamp()) {
            // Shape has been changed in the most recent step.  Rebuild the
            // list of neighbours.
            tree.root.markIfTouching(this, tree);
        }
    }

    // **** Leaf Functions
    containsShape(shape: Shape): boolean {
        return (
            this.bbL <= shape.bbL &&
            this.bbR >= shape.bbR &&
            this.bbB <= shape.bbB &&
            this.bbT >= shape.bbT
        );
    }

    update(tree: BBTree): boolean {
        let root = tree.root;
        const shape = this.shape;

        // if(!bbContainsBB(this.bb, bb)){
        if (!this.containsShape(shape)) {
            tree.getBB(this.shape, this);

            root = subtreeRemove(root, this, tree);
            if (tree.root) {
                tree.root = tree.root.insert(this);
            } else {
                tree.root = this;
            }

            this.clearPairs(tree);
            this.stamp = tree.getStamp();

            return true;
        }

        return false;
    }

    addPairs(tree: BBTree): void {
        if (tree.root) {
            tree.root.markIfTouching(this, tree);
        }
    }

    intersectsBB(bb: BB): boolean {
        return (
            this.bbL <= bb.r &&
            bb.l <= this.bbR &&
            this.bbB <= bb.t &&
            bb.b <= this.bbT
        );
    }

    bbArea(): number {
        return (this.bbR - this.bbL) * (this.bbT - this.bbB);
    }

    query(bb: BB, func: (shape: Shape) => any): void {
        if (this.intersectsBB(bb)) {
            func(this.shape);
        }
    }

    segmentQuery(
        a: Vect, b: Vect, tExit: number,
        func: (shape: Shape) => any,
    ): number {
        return func(this.shape);
    }
}

function bbTreeMergedArea(a: INode, b: INode): number {
    return (
        (Math.max(a.bbR, b.bbR) - Math.min(a.bbL, b.bbL)) *
        (Math.max(a.bbT, b.bbT) - Math.min(a.bbB, b.bbB))
    );
}

function bbProximity(a: INode, b: INode): number {
    return (
        Math.abs(a.bbL + a.bbR - b.bbL - b.bbR) +
        Math.abs(a.bbB + a.bbT - b.bbB - b.bbT)
    );
}

function subtreeRemove(subtree: INode, leaf: Leaf, tree: BBTree): INode {
    if (leaf === subtree) {
        return null;
    } else {
        if (leaf.parent === subtree) {
            const other = leaf.parent.otherChild(leaf);
            other.parent = subtree.parent;
            return other;
        } else {
            leaf.parent.parent.replaceChild(
                leaf.parent, leaf.parent.otherChild(leaf));
            return subtree;
        }
    }
}

function bbTreeIntersectsNode(a: INode, b: INode): boolean {
    return (
        a.bbL <= b.bbR &&
        b.bbL <= a.bbR &&
        a.bbB <= b.bbT &&
        b.bbB <= a.bbT
    );
}

function bbTreeMergedArea2(
    node: INode, l: number, b: number, r: number, t: number,
): number {
    return (
        (Math.max(node.bbR, r) - Math.min(node.bbL, l)) *
        (Math.max(node.bbT, t) - Math.min(node.bbB, b))
    );
}

class BBTree {
    private velocityFunc: (shape: Shape) => Vect = null;

    leaves: Map<Shape, Leaf> = new Map();
    private stamp: number = 0;
    root: INode = null;

    getStamp() {
        return this.stamp;
    }

    incrementStamp() {
        this.stamp++;
    }

    // TODO move to leaf
    getBB(shape: Shape, dest: Leaf): void {
        const velocityFunc = this.velocityFunc;
        if (velocityFunc) {
            const coef = 0.1;
            const x = (shape.bbR - shape.bbL) * coef;
            const y = (shape.bbT - shape.bbB) * coef;

            const v = vmult(velocityFunc(shape), 0.1);

            dest.bbL = shape.bbL + Math.min(-x, v.x);
            dest.bbB = shape.bbB + Math.min(-y, v.y);
            dest.bbR = shape.bbR + Math.max(x, v.x);
            dest.bbT = shape.bbT + Math.max(y, v.y);
        } else {
            dest.bbL = shape.bbL;
            dest.bbB = shape.bbB;
            dest.bbR = shape.bbR;
            dest.bbT = shape.bbT;
        }
    }

    // **** Insert/Remove
    insert(shape: Shape): void {
        const leaf = new Leaf(this, shape);

        this.leaves.set(shape, leaf);
        if (this.root) {
            this.root = this.root.insert(leaf);
        } else {
            this.root = leaf;
        }

        leaf.stamp = this.getStamp();
        leaf.addPairs(this);
        this.incrementStamp();
    }

    remove(shape: Shape) {
        const leaf = this.leaves.get(shape);

        this.leaves.delete(shape);
        this.root = subtreeRemove(this.root, leaf, this);

        leaf.clearPairs(this);
    }

    contains(shape: Shape) {
        return this.leaves.has(shape);
    }

    reindex(shapes: Shape[]): void {
        const leaves = shapes.map((shape: Shape) => {
            return this.leaves.get(shape);
        });

        leaves.forEach((leaf: Leaf) => {
            leaf.update(this);
        });

        leaves.forEach((leaf: Leaf) => {
            leaf.markTouching(this);
        });

        this.incrementStamp();
    }

    // **** Query

    shapeQuery(shape: Shape, func: (other: Shape) => any): void {
        const leaf = this.leaves.get(shape);
        if (leaf) {
            // Leaf is in the index.  Use the cached sets of touching leaves.
            leaf.touching.forEach((other: Leaf) => {
                func(other.shape);
            });
        } else {
            // Shape is not in the index.  Perform a regular query using the
            // shape's bounding box.
            // TODO use velocityFunc
            // TODO it's possible that we don't want to provide this fallback
            this.query(shape.getBB(), func);
        }
    }

    pointQuery(
        v: Vect,
        func: (shape: Shape) => any,
    ): void {
        this.query(new BB(v.x, v.y, v.x, v.y), func);
    }

    segmentQuery(
        a: Vect, b: Vect, tExit: number,
        func: (shape: Shape) => any,
    ): void {
        if (this.root) {
            this.root.segmentQuery(a, b, tExit, func);
        }
    }

    query(
        bb: BB,
        func: (shape: Shape) => any,
    ): void {
        if (this.root) {
            this.root.query(bb, func);
        }
    }

    each(func: (shape: Shape) => any) {
        this.leaves.forEach((leaf: Leaf) => {
            func(leaf.shape);
        });
    }
}

export class BBTreeIndex extends SpatialIndex {
    private tree = new BBTree();

    private activeShapes: Set<Shape> = new Set();

    // **** Insert/Remove
    insertStatic(shape: Shape): void {
        this.tree.insert(shape);
        this.count++;
    }

    insert(shape: Shape): void {
        this.tree.insert(shape);
        this.activeShapes.add(shape);
        this.count++;
    }

    remove(shape: Shape) {
        this.tree.remove(shape);
        this.activeShapes.delete(shape);
        this.count--;
    }

    contains(shape: Shape) {
        return this.tree.contains(shape);
    }

    reindexStatic(): void {
        const shapes: Shape[] = [];
        this.tree.leaves.forEach((leaf: Leaf, shape: Shape) => {
            shapes.push(shape);
        });
        this.tree.reindex(shapes);
    }

    reindex(): void {
        const shapes: Shape[] = [];
        this.activeShapes.forEach((shape: Shape) => {
            shapes.push(shape);
        });
        this.tree.reindex(shapes);
    }

    reindexShape(shape: Shape): void {
        this.tree.reindex([shape]);
    }

    // **** Query

    touchingQuery(func: (a: Shape, b: Shape) => any): void {
        const visited = new Set();
        this.activeShapes.forEach((shape: Shape) => {
            this.tree.shapeQuery(shape, (other: Shape) => {
                if (visited.has(other)) {
                    func(shape, other);
                }
                visited.add(shape);
            });
        });
    }

    pointQuery(
        v: Vect,
        func: (shape: Shape) => any,
    ): void {
        return this.tree.pointQuery(v, func);
    }

    segmentQuery(
        a: Vect, b: Vect, tExit: number,
        func: (shape: Shape) => any,
    ): void {
        return this.tree.segmentQuery(a, b, tExit, func);
    }

    query(
        bb: BB,
        func: (shape: Shape) => any,
    ): void {
        this.tree.query(bb, func);
    }

    each(func: (shape: Shape) => any) {
        this.tree.leaves.forEach((leaf: Leaf) => {
            func(leaf.shape);
        });
    }
}
