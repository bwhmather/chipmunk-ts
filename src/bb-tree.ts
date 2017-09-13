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

function voidQueryFunc(obj1: Shape, obj2: Shape) {
    // Pass.
}

interface INode {
    parent: Branch;

    bbL: number;
    bbB: number;
    bbR: number;
    bbT: number;

    insert(leaf: Leaf): INode;

    markIfTouchingQuery(leaf: Leaf, tree: BBTree): void;

    bbArea(): number;

    query(bb: BB, func: (obj: Shape) => any): void;

    /// Returns the fraction along the segment query the node hits. Returns
    /// Infinity if it doesn't hit.
    segmentQuery(
        a: Vect, b: Vect, tExit: number,
        func: (obj: Shape) => any,
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

    markIfTouchingQuery(leaf: Leaf, tree: BBTree): void {
        if (bbTreeIntersectsNode(leaf, this)) {
            this.childA.markIfTouchingQuery(leaf, tree);
            this.childB.markIfTouchingQuery(leaf, tree);
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

    query(bb: BB, func: (obj: Shape) => any): void {
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
        func: (obj: Shape) => any,
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
    obj: Shape;
    parent: Branch;
    number: number;
    stamp: number;

    touching: Set<Leaf>;

    constructor(tree: BBTree, obj: Shape) {
        this.obj = obj;
        tree.getBB(obj, this);

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

    markIfTouchingQuery(leaf: Leaf, tree: BBTree): void {
        if (bbTreeIntersectsNode(leaf, this)) {
            this.touching.add(leaf);
            leaf.touching.add(this);
        }
    }

    markTouchingQuery(tree: BBTree, staticRoot: INode): void {
        if (this.stamp === tree.getStamp()) {
            // Shape has been changed in the most recent step.  Rebuild the
            // list of neighbours.
            if (staticRoot) {
                staticRoot.markIfTouchingQuery(this, tree);
            }

            for (let node: INode = this; node.parent; node = node.parent) {
                if (node === node.parent.childA) {
                    node.parent.childB.markIfTouchingQuery(this, tree);
                } else {
                    node.parent.childA.markIfTouchingQuery(this, tree);
                }
            }
        }
    }

    // **** Leaf Functions
    containsObj(obj: Shape): boolean {
        return (
            this.bbL <= obj.bbL &&
            this.bbR >= obj.bbR &&
            this.bbB <= obj.bbB &&
            this.bbT >= obj.bbT
        );
    }

    update(tree: BBTree): boolean {
        let root = tree.root;
        const obj = this.obj;

        // if(!bbContainsBB(this.bb, bb)){
        if (!this.containsObj(obj)) {
            tree.getBB(this.obj, this);

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
        const dynamicIndex = tree.dynamicIndex;
        if (dynamicIndex) {
            const dynamicRoot = dynamicIndex.root;
            if (dynamicRoot) {
                dynamicRoot.markIfTouchingQuery(this, dynamicIndex);
            }
        } else {
            const staticRoot = tree.staticIndex.root;
            this.markTouchingQuery(tree, staticRoot);
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

    query(bb: BB, func: (obj: Shape) => any): void {
        if (this.intersectsBB(bb)) {
            func(this.obj);
        }
    }

    segmentQuery(
        a: Vect, b: Vect, tExit: number,
        func: (obj: Shape) => any,
    ): number {
        return func(this.obj);
    }
}

export class BBTree extends SpatialIndex {
    velocityFunc: (obj: Shape) => Vect = null;
    leaves: Map<Shape, Leaf>;  // TODO TODO TODO
    root: INode = null;
    stamp: number = 0;

    staticIndex: BBTree;
    dynamicIndex: BBTree;

    constructor(staticIndex: BBTree) {
        super(staticIndex);

        // This is a hash from object ID -> object for the objects stored in the
        // BBTree.
        this.leaves = new Map();
    }

    getBB(obj: Shape, dest: Leaf): void {
        const velocityFunc = this.velocityFunc;
        if (velocityFunc) {
            const coef = 0.1;
            const x = (obj.bbR - obj.bbL) * coef;
            const y = (obj.bbT - obj.bbB) * coef;

            const v = vmult(velocityFunc(obj), 0.1);

            dest.bbL = obj.bbL + Math.min(-x, v.x);
            dest.bbB = obj.bbB + Math.min(-y, v.y);
            dest.bbR = obj.bbR + Math.max(x, v.x);
            dest.bbT = obj.bbT + Math.max(y, v.y);
        } else {
            dest.bbL = obj.bbL;
            dest.bbB = obj.bbB;
            dest.bbR = obj.bbR;
            dest.bbT = obj.bbT;
        }
    }

    getStamp() {
        const dynamic = this.dynamicIndex;
        return (dynamic && dynamic.stamp ? dynamic.stamp : this.stamp);
    }

    incrementStamp() {
        if (this.dynamicIndex && this.dynamicIndex.stamp) {
            this.dynamicIndex.stamp++;
        } else {
            this.stamp++;
        }
    }

    // **** Insert/Remove
    insert(obj: Shape): void {
        const leaf = new Leaf(this, obj);

        this.leaves.set(obj, leaf);
        if (this.root) {
            this.root = this.root.insert(leaf);
        } else {
            this.root = leaf;
        }
        this.count++;

        leaf.stamp = this.getStamp();
        leaf.addPairs(this);
        this.incrementStamp();
    }

    remove(obj: Shape) {
        const leaf = this.leaves.get(obj);

        this.leaves.delete(obj);
        this.root = subtreeRemove(this.root, leaf, this);
        this.count--;

        leaf.clearPairs(this);
    }

    contains(obj: Shape) {
        return this.leaves.has(obj);
    }

    reindex(): void {
        if (!this.root) {
            return;
        }

        this.leaves.forEach((leaf: Leaf) => {
            leaf.update(this);
        });

        const staticIndex = this.staticIndex;
        const staticRoot = staticIndex && staticIndex.root;


        this.leaves.forEach((leaf: Leaf) => {
            leaf.markTouchingQuery(this, staticRoot);
        });

        this.incrementStamp();
    }

    // DEPRECATED
    reindexQuery(func: (a: Shape, b: Shape) => any): void {
        this.reindex();
        this.touchingQuery(func);
    }

    reindexObject(obj: Shape): void {
        const leaf = this.leaves.get(obj);
        if (leaf) {
            if (leaf.update(this)) {
                leaf.addPairs(this);
            }
            this.incrementStamp();
        }
    }

    // **** Query

    touchingQuery(func: (a: Shape, b: Shape) => any): void {
        const visited = new Set()
        this.leaves.forEach((leaf: Leaf) => {
            leaf.touching.forEach((touching: Leaf) => {
                // TODO I think this should be the other way round.
                if (visited.has(touching)) {
                    func(leaf.obj, touching.obj)
                }
                visited.add(leaf);
            });
        });
    }

    pointQuery(
        v: Vect,
        func: (obj: Shape) => any,
    ): void {
        this.query(new BB(v.x, v.y, v.x, v.y), func);
    }

    segmentQuery(
        a: Vect, b: Vect, tExit: number,
        func: (obj: Shape) => any,
    ): void {
        if (this.root) {
            this.root.segmentQuery(a, b, tExit, func);
        }
    }

    query(
        bb: BB,
        func: (obj: Shape) => any,
    ): void {
        if (this.root) {
            this.root.query(bb, func);
        }
    }

    each(func: (obj: Shape) => any) {
        this.leaves.forEach((leaf: Leaf) => {
            func(leaf.obj);
        });
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
