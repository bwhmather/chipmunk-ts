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

import { SpatialIndex } from './spatial-index';
import { BB } from './bb';
import { Vect, vmult } from './vect';
import { assertSoft } from './util';
import { Shape } from './shapes';


function voidQueryFunc(obj1: Shape, obj2: Shape) { }


interface Node {
    parent: Branch;

    bb_l: number;
    bb_b: number;
    bb_r: number;
    bb_t: number;

    insert(leaf: Leaf): Node;

    markLeafQuery(
        leaf: Leaf, left: boolean, tree: BBTree,
        func: (a: Shape, b: Shape) => any,
    ): void;

    markSubtree(
        tree: BBTree, staticRoot: Node,
        func: (a: Shape, b: Shape) => any,
    ): void;

    bbArea(): number;

    query(bb: BB, func: (obj: Shape) => any): void;

    /// Returns the fraction along the segment query the node hits. Returns
    /// Infinity if it doesn't hit.
    segmentQuery(
        a: Vect, b: Vect, t_exit: number,
        func: (obj: Shape) => any,
    ): number;
}


export class Branch implements Node {
    bb_l: number;
    bb_b: number;
    bb_r: number;
    bb_t: number;
    parent: Branch;

    A: Node;
    B: Node;

    constructor(a: Node, b: Node) {
        this.bb_l = Math.min(a.bb_l, b.bb_l);
        this.bb_b = Math.min(a.bb_b, b.bb_b);
        this.bb_r = Math.max(a.bb_r, b.bb_r);
        this.bb_t = Math.max(a.bb_t, b.bb_t);
        this.parent = null;

        this.setA(a);
        this.setB(b);
    }

    setA(value: Node): void {
        this.A = value;
        value.parent = this;
    }

    setB(value: Node): void {
        this.B = value;
        value.parent = this;
    }

    otherChild(child: Node): Node {
        return (this.A == child ? this.B : this.A);
    }

    replaceChild(child: Node, value: Node): void {
        assertSoft(child == this.A || child == this.B, "Node is not a child of parent.");

        if (this.A == child) {
            this.setA(value);
        } else {
            this.setB(value);
        }

        for (let branch: Branch = this; branch; branch = branch.parent) {
            //node.bb = bbMerge(node.A.bb, node.B.bb);
            const a = branch.A;
            const b = branch.B;
            branch.bb_l = Math.min(a.bb_l, b.bb_l);
            branch.bb_b = Math.min(a.bb_b, b.bb_b);
            branch.bb_r = Math.max(a.bb_r, b.bb_r);
            branch.bb_t = Math.max(a.bb_t, b.bb_t);
        }
    }

    insert(leaf: Leaf): Node {
        let cost_a = this.B.bbArea() + bbTreeMergedArea(this.A, leaf);
        let cost_b = this.A.bbArea() + bbTreeMergedArea(this.B, leaf);

        if (cost_a === cost_b) {
            cost_a = bbProximity(this.A, leaf);
            cost_b = bbProximity(this.B, leaf);
        }

        if (cost_b < cost_a) {
            this.setB(this.B.insert(leaf));
        } else {
            this.setA(this.A.insert(leaf));
        }

        this.bb_l = Math.min(this.bb_l, leaf.bb_l);
        this.bb_b = Math.min(this.bb_b, leaf.bb_b);
        this.bb_r = Math.max(this.bb_r, leaf.bb_r);
        this.bb_t = Math.max(this.bb_t, leaf.bb_t);

        return this;
    }

    markLeafQuery(
        leaf: Leaf, left: boolean, tree: BBTree,
        func: (a: Shape, b: Shape) => any,
    ) {
        if (bbTreeIntersectsNode(leaf, this)) {
            this.A.markLeafQuery(leaf, left, tree, func);
            this.B.markLeafQuery(leaf, left, tree, func);
        }
    }

    markSubtree(
        tree: BBTree, staticRoot: Node,
        func: (a: Shape, b: Shape) => any,
    ) {
        this.A.markSubtree(tree, staticRoot, func);
        this.B.markSubtree(tree, staticRoot, func);
    }

    intersectsBB(bb: BB): boolean {
        return (
            this.bb_l <= bb.r &&
            bb.l <= this.bb_r &&
            this.bb_b <= bb.t &&
            bb.b <= this.bb_t
        );
    }

    bbArea(): number {
        return (this.bb_r - this.bb_l) * (this.bb_t - this.bb_b);
    }

    query(bb: BB, func: (obj: Shape) => any): void {
        //if(bbIntersectsBB(subtree.bb, bb)){
        if (this.intersectsBB(bb)) {
            this.A.query(bb, func);
            this.B.query(bb, func);
        }
    }

    /// Returns the fraction along the segment query the node hits. Returns Infinity if it doesn't hit.
    private childSegmentQuery(
        child: Node, a: Vect, b: Vect,
    ): number {
        const idx = 1 / (b.x - a.x);
        const tx1 = (child.bb_l == a.x ? -Infinity : (child.bb_l - a.x) * idx);
        const tx2 = (child.bb_r == a.x ? Infinity : (child.bb_r - a.x) * idx);
        const txmin = Math.min(tx1, tx2);
        const txmax = Math.max(tx1, tx2);

        const idy = 1 / (b.y - a.y);
        const ty1 = (child.bb_b == a.y ? -Infinity : (child.bb_b - a.y) * idy);
        const ty2 = (child.bb_t == a.y ? Infinity : (child.bb_t - a.y) * idy);
        const tymin = Math.min(ty1, ty2);
        const tymax = Math.max(ty1, ty2);

        if (tymin <= txmax && txmin <= tymax) {
            const min_ = Math.max(txmin, tymin);
            const max_ = Math.min(txmax, tymax);

            if (0.0 <= max_ && min_ <= 1.0) return Math.max(min_, 0.0);
        }

        return Infinity;
    };

    segmentQuery(
        a: Vect, b: Vect, t_exit: number,
        func: (obj: Shape) => any,
    ): number {
        const t_a = this.childSegmentQuery(this.A, a, b);
        const t_b = this.childSegmentQuery(this.B, a, b);

        if (t_a < t_b) {
            if (t_a < t_exit) {
                t_exit = Math.min(
                    t_exit, this.A.segmentQuery(a, b, t_exit, func),
                );
            }
            if (t_b < t_exit) {
                t_exit = Math.min(
                    t_exit, this.B.segmentQuery(a, b, t_exit, func),
                );
            }
        } else {
            if (t_b < t_exit) {
                t_exit = Math.min(
                    t_exit, this.B.segmentQuery(a, b, t_exit, func),
                );
            }
            if (t_a < t_exit) {
                t_exit = Math.min(
                    t_exit, this.A.segmentQuery(a, b, t_exit, func),
                );
            }
        }

        return t_exit;
    };
}


export class Leaf implements Node {
    bb_l: number;
    bb_b: number;
    bb_r: number;
    bb_t: number;
    obj: Shape;
    parent: Branch;
    number: number;
    stamp: number;

    touching_right: Set<Leaf>;
    touching_left: Set<Leaf>;

    constructor(tree: BBTree, obj: Shape) {
        this.obj = obj;
        tree.getBB(obj, this);

        this.parent = null;

        this.stamp = 1;
        this.touching_left = new Set();
        this.touching_right = new Set();
    }

    insert(leaf: Leaf): Node {
        return new Branch(leaf, this);
    }

    clearPairs(tree: BBTree): void {
        this.touching_left.forEach((other: Leaf) => {
            other.touching_right.delete(this);
        });
        this.touching_right.forEach((other: Leaf) => {
            other.touching_left.delete(this);
        });

        this.touching_left.clear();
        this.touching_right.clear();
    }

    markLeafQuery(
        leaf: Leaf, left: boolean, tree: BBTree,
        func: (a: Shape, b: Shape) => any,
    ): void {
        if (bbTreeIntersectsNode(leaf, this)) {
            if (left) {
                leaf.touching_left.add(this);
                this.touching_right.add(leaf);
            } else {
                // we don't have to check if the other leaf has been updated
                // as this operation is idempotent.
                leaf.touching_right.add(this);
                this.touching_left.add(leaf);

                if (func) func(leaf.obj, this.obj);
            }
        }
    }

    markSubtree(
        tree: BBTree, staticRoot: Node,
        func: (a: Shape, b: Shape) => any,
    ): void {
        if (this.stamp == tree.getStamp()) {
            // Shape has been changed in the most recent step.  Rebuild the
            // list of neighbours.
            if (staticRoot) staticRoot.markLeafQuery(this, false, tree, func);

            for (let node: Node = this; node.parent; node = node.parent) {
                if (node == node.parent.A) {
                    node.parent.B.markLeafQuery(this, true, tree, func);
                } else {
                    node.parent.A.markLeafQuery(this, false, tree, func);
                }
            }
        } else {
            // Shape has not been changed in the most recent step.  Use the
            // cached list of neighbours.
            if (func) {
                this.touching_right.forEach((other: Leaf) => {
                    func(this.obj, other.obj);
                });
            }
        }
    }

    // **** Leaf Functions
    containsObj(obj: Shape): boolean {
        return (
            this.bb_l <= obj.bb_l &&
            this.bb_r >= obj.bb_r &&
            this.bb_b <= obj.bb_b &&
            this.bb_t >= obj.bb_t
        );
    }

    update(tree: BBTree): boolean {
        let root = tree.root;
        const obj = this.obj;

        //if(!bbContainsBB(this.bb, bb)){
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
                dynamicRoot.markLeafQuery(this, true, dynamicIndex, null);
            }
        } else {
            const staticRoot = tree.staticIndex.root;
            this.markSubtree(tree, staticRoot, null);
        }
    }

    intersectsBB(bb: BB): boolean {
        return (
            this.bb_l <= bb.r &&
            bb.l <= this.bb_r &&
            this.bb_b <= bb.t &&
            bb.b <= this.bb_t
        );
    }

    bbArea(): number {
        return (this.bb_r - this.bb_l) * (this.bb_t - this.bb_b);
    }

    query(bb: BB, func: (obj: Shape) => any): void {
        if (this.intersectsBB(bb)) {
            func(this.obj);
        }
    }

    segmentQuery(
        a: Vect, b: Vect, t_exit: number,
        func: (obj: Shape) => any,
    ): number {
        return func(this.obj);
    }
}


export class BBTree extends SpatialIndex {
    velocityFunc: (obj: Shape) => Vect = null;
    leaves: Map<Shape, Leaf>;  // TODO TODO TODO
    root: Node = null;
    stamp: number = 0;

    staticIndex: BBTree;
    dynamicIndex: BBTree;

    constructor(staticIndex: BBTree) {
        super(staticIndex);

        // This is a hash from object ID -> object for the objects stored in the BBTree.
        this.leaves = new Map();
    }

    getBB(obj: Shape, dest: Leaf): void {
        const velocityFunc = this.velocityFunc;
        if (velocityFunc) {
            const coef = 0.1;
            const x = (obj.bb_r - obj.bb_l) * coef;
            const y = (obj.bb_t - obj.bb_b) * coef;

            const v = vmult(velocityFunc(obj), 0.1);

            dest.bb_l = obj.bb_l + Math.min(-x, v.x);
            dest.bb_b = obj.bb_b + Math.min(-y, v.y);
            dest.bb_r = obj.bb_r + Math.max(x, v.x);
            dest.bb_t = obj.bb_t + Math.max(y, v.y);
        } else {
            dest.bb_l = obj.bb_l;
            dest.bb_b = obj.bb_b;
            dest.bb_r = obj.bb_r;
            dest.bb_t = obj.bb_t;
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
            this.root = leaf
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

    reindexQuery(func: (a: Shape, b: Shape) => any): void {
        if (!this.root) return;

        this.leaves.forEach((leaf: Leaf) => {
            leaf.update(this);
        });

        const staticIndex = this.staticIndex;
        const staticRoot = staticIndex && staticIndex.root;

        this.root.markSubtree(this, staticRoot, func);
        if (staticIndex && !staticRoot) {
            this.collideStatic(staticIndex, func);
        }

        this.incrementStamp();
    }

    reindex(): void {
        this.reindexQuery(voidQueryFunc);
    }

    reindexObject(obj: Shape): void {
        const leaf = this.leaves.get(obj);
        if (leaf) {
            if (leaf.update(this)) leaf.addPairs(this);
            this.incrementStamp();
        }
    }

    // **** Query

    // This has since been removed from upstream Chipmunk - which recommends you just use query() below
    // directly.
    pointQuery(
        v: Vect,
        func: (obj: Shape) => any,
    ): void {
        this.query(new BB(v.x, v.y, v.x, v.y), func);
    }

    segmentQuery(
        a: Vect, b: Vect, t_exit: number,
        func: (obj: Shape) => any,
    ): void {
        if (this.root) {
            this.root.segmentQuery(a, b, t_exit, func);
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

    log(): void {
        if (this.root) {
            //nodeRender(this.root, 0);
        }
    }

    each(func: (obj: Shape) => any) {
        this.leaves.forEach((leaf: Leaf) => {
            func(leaf.obj);
        });
    }
}


function bbTreeMergedArea(a: Node, b: Node): number {
    return (
        (Math.max(a.bb_r, b.bb_r) - Math.min(a.bb_l, b.bb_l)) *
        (Math.max(a.bb_t, b.bb_t) - Math.min(a.bb_b, b.bb_b))
    );
};


function bbProximity(a: Node, b: Node): number {
    return (
        Math.abs(a.bb_l + a.bb_r - b.bb_l - b.bb_r) +
        Math.abs(a.bb_b + a.bb_t - b.bb_b - b.bb_t)
    );
};



function subtreeRemove(subtree: Node, leaf: Leaf, tree: BBTree): Node {
    if (leaf == subtree) {
        return null;
    } else {
        if (leaf.parent == subtree) {
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


function bbTreeIntersectsNode(a: Node, b: Node): boolean {
    return (
        a.bb_l <= b.bb_r &&
        b.bb_l <= a.bb_r &&
        a.bb_b <= b.bb_t &&
        b.bb_b <= a.bb_t
    );
};


function bbTreeMergedArea2(
    node: Node, l: number, b: number, r: number, t: number,
): number {
    return (
        (Math.max(node.bb_r, r) - Math.min(node.bb_l, l)) *
        (Math.max(node.bb_t, t) - Math.min(node.bb_b, b))
    );
};


/* export function nodeRender(node: Node, depth: number): void {
    if (!node.isLeaf && depth <= 10) {
        nodeRender(node.A, depth + 1);
        nodeRender(node.B, depth + 1);
    }

    let str = '';
    for (let i = 0; i < depth; i++) {
        str += ' ';
    }

    console.log(str + node.bb_b + ' ' + node.bb_t);
}*/
