/* Copyright (c) 2009 Scott Lembcke
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


// This file implements a modified AABB tree for collision detection.

export class BBTree extends SpatialIndex {
    velocityFunc;
    leaves;
    root;
    pooledNodes;
    pooledPairs;
    stamp;
    dynamicIndex;

    constructor(staticIndex) {
        super(staticIndex);

        this.velocityFunc = null;

        // This is a hash from object ID -> object for the objects stored in the BBTree.
        this.leaves = {};
        // A count of the number of leaves in the BBTree.
        this.count = 0;

        this.root = null;

        // A linked list containing an object pool of tree nodes and pairs.
        this.pooledNodes = null;
        this.pooledPairs = null;

        this.stamp = 0;
    }

    makeNode(a, b) {
        const node = this.pooledNodes;
        if (node) {
            this.pooledNodes = node.parent;
            node.constructor(this, a, b);
            return node;
        } else {
            numNodes++;
            return new Node(this, a, b);
        }
    }

    getBB(obj, dest) {
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

    makePair(leafA, nextA, leafB, nextB) {
        //return new Pair(leafA, nextA, leafB, nextB);
        const pair = this.pooledPairs;
        if (pair) {
            this.pooledPairs = pair.prevA;

            pair.prevA = null;
            pair.leafA = leafA;
            pair.nextA = nextA;

            pair.prevB = null;
            pair.leafB = leafB;
            pair.nextB = nextB;

            //pair.constructor(leafA, nextA, leafB, nextB);
            return pair;
        } else {
            numPairs++;
            return new Pair(leafA, nextA, leafB, nextB);
        }
    }

    subtreeRecycle(node) {
        if (node.isLeaf) {
            this.subtreeRecycle(node.A);
            this.subtreeRecycle(node.B);
            node.recycle(this);
        }
    }

    // **** Insert/Remove

    insert(obj, hashid) {
        const leaf = new Leaf(this, obj);

        this.leaves[hashid] = leaf;
        this.root = subtreeInsert(this.root, leaf, this);
        this.count++;

        leaf.stamp = this.getStamp();
        leaf.addPairs(this);
        this.incrementStamp();
    }

    remove(obj, hashid) {
        const leaf = this.leaves[hashid];

        delete this.leaves[hashid];
        this.root = subtreeRemove(this.root, leaf, this);
        this.count--;

        leaf.clearPairs(this);
        leaf.recycle(this);
    }

    contains(obj, hashid) {
        return this.leaves[hashid] != null;
    }

    reindexQuery(func) {
        if (!this.root) return;

        // LeafUpdate() may modify this.root. Don't cache it.
        let hashid;

        const leaves = this.leaves;
        for (hashid in leaves) {
            leaves[hashid].update(this);
        }

        const staticIndex = this.staticIndex;
        const staticRoot = staticIndex && staticIndex.root;

        this.root.markSubtree(this, staticRoot, func);
        if (staticIndex && !staticRoot) this.collideStatic(this, staticIndex, func);

        this.incrementStamp();
    }

    reindex() {
        this.reindexQuery(voidQueryFunc);
    }

    reindexObject(obj, hashid) {
        const leaf = this.leaves[hashid];
        if (leaf) {
            if (leaf.update(this)) leaf.addPairs(this);
            this.incrementStamp();
        }
    }

    // **** Query

    // This has since been removed from upstream Chipmunk - which recommends you just use query() below
    // directly.
    pointQuery({ x, y }, func) {
        this.query(new BB(x, y, x, y), func);
    }

    segmentQuery(a, b, t_exit, func) {
        if (this.root) subtreeSegmentQuery(this.root, a, b, t_exit, func);
    }

    query(bb, func) {
        if (this.root) subtreeQuery(this.root, bb, func);
    }

    optimize() {
        const nodes = new Array(this.count);
        let i = 0;

        for (const hashid in this.leaves) {
            nodes[i++] = this.nodes[hashid];
        }

        tree.subtreeRecycle(root);
        this.root = partitionNodes(tree, nodes, nodes.length);
    }

    log() {
        if (this.root) nodeRender(this.root, 0);
    }

    count() {
        return this.count;
    }

    each(func) {
        let hashid;
        for (hashid in this.leaves) {
            func(this.leaves[hashid].obj);
        }
    }
}




// **** Reindex
function voidQueryFunc(obj1, obj2) { }




var numNodes = 0;

export class Node {
    isLeaf: boolean;
    obj;
    bb_l: number;
    bb_b: number;
    bb_r: number;
    bb_t: number;
    parent;

    A;
    B;

    constructor(tree, a, b) {
        this.isLeaf = false;
        this.obj = null;
        this.bb_l = Math.min(a.bb_l, b.bb_l);
        this.bb_b = Math.min(a.bb_b, b.bb_b);
        this.bb_r = Math.max(a.bb_r, b.bb_r);
        this.bb_t = Math.max(a.bb_t, b.bb_t);
        this.parent = null;

        this.setA(a);
        this.setB(b);
    }

    recycle(tree) {
        this.parent = tree.pooledNodes;
        tree.pooledNodes = this;
    }

    setA(value) {
        this.A = value;
        value.parent = this;
    }

    setB(value) {
        this.B = value;
        value.parent = this;
    }

    otherChild(child) {
        return (this.A == child ? this.B : this.A);
    }

    replaceChild(child, value, tree) {
        assertSoft(child == this.A || child == this.B, "Node is not a child of parent.");

        if (this.A == child) {
            this.A.recycle(tree);
            this.setA(value);
        } else {
            this.B.recycle(tree);
            this.setB(value);
        }

        for (let node = this; node; node = node.parent) {
            //node.bb = bbMerge(node.A.bb, node.B.bb);
            const a = node.A;
            const b = node.B;
            node.bb_l = Math.min(a.bb_l, b.bb_l);
            node.bb_b = Math.min(a.bb_b, b.bb_b);
            node.bb_r = Math.max(a.bb_r, b.bb_r);
            node.bb_t = Math.max(a.bb_t, b.bb_t);
        }
    }

    markLeafQuery(leaf, left, tree, func) {
        if (bbTreeIntersectsNode(leaf, this)) {
            this.A.markLeafQuery(leaf, left, tree, func);
            this.B.markLeafQuery(leaf, left, tree, func);
        }
    }

    markSubtree(tree, staticRoot, func) {
        this.A.markSubtree(tree, staticRoot, func);
        this.B.markSubtree(tree, staticRoot, func);
    }

    intersectsBB(bb) {
        return (
            this.bb_l <= bb.r &&
            bb.l <= this.bb_r &&
            this.bb_b <= bb.t &&
            bb.b <= this.bb_t
        );
    }

    bbArea() {
        return (this.bb_r - this.bb_l) * (this.bb_t - this.bb_b);
    }
}











let numLeaves = 0;

export class Leaf {
    isLeaf: boolean;
    bb_l: number;
    bb_b: number;
    bb_r: number;
    bb_t: number;
    obj;
    parent;
    stamp;
    pairs;

    constructor(tree, obj) {
        this.isLeaf = true;
        this.obj = obj;
        tree.getBB(obj, this);

        this.parent = null;

        this.stamp = 1;
        this.pairs = null;
        numLeaves++;
    }

    clearPairs(tree) {
        let pair = this.pairs;
        let next;

        this.pairs = null;

        while (pair) {
            if (pair.leafA === this) {
                next = pair.nextA;
                unlinkThread(pair.prevB, pair.leafB, pair.nextB);
            } else {
                next = pair.nextB;
                unlinkThread(pair.prevA, pair.leafA, pair.nextA);
            }
            pair.recycle(tree);
            pair = next;
        }
    }

    recycle(tree) {
        // Its not worth the overhead to recycle leaves.
    }

    markLeafQuery(leaf, left, tree, func) {
        if (bbTreeIntersectsNode(leaf, this)) {
            if (left) {
                pairInsert(leaf, this, tree);
            } else {
                if (this.stamp < leaf.stamp) pairInsert(this, leaf, tree);
                if (func) func(leaf.obj, this.obj);
            }
        }
    }

    markSubtree(tree, staticRoot, func) {
        if (this.stamp == tree.getStamp()) {
            if (staticRoot) staticRoot.markLeafQuery(this, false, tree, func);

            for (let node = this; node.parent; node = node.parent) {
                if (node == node.parent.A) {
                    node.parent.B.markLeafQuery(this, true, tree, func);
                } else {
                    node.parent.A.markLeafQuery(this, false, tree, func);
                }
            }
        } else {
            let pair = this.pairs;
            while (pair) {
                if (this === pair.leafB) {
                    if (func) func(pair.leafA.obj, this.obj);
                    pair = pair.nextB;
                } else {
                    pair = pair.nextA;
                }
            }
        }
    }

    // **** Leaf Functions

    containsObj({ bb_l, bb_r, bb_b, bb_t }) {
        return this.bb_l <= bb_l && this.bb_r >= bb_r && this.bb_b <= bb_b && this.bb_t >= bb_t;
    }

    update(tree) {
        let root = tree.root;
        const obj = this.obj;

        //if(!bbContainsBB(this.bb, bb)){
        if (!this.containsObj(obj)) {
            tree.getBB(this.obj, this);

            root = subtreeRemove(root, this, tree);
            tree.root = subtreeInsert(root, this, tree);

            this.clearPairs(tree);
            this.stamp = tree.getStamp();

            return true;
        }

        return false;
    }

    addPairs(tree) {
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

    intersectsBB(bb) {
        return (
            this.bb_l <= bb.r &&
            bb.l <= this.bb_r &&
            this.bb_b <= bb.t &&
            bb.b <= this.bb_t
        );
    }

    bbArea() {
        return (this.bb_r - this.bb_l) * (this.bb_t - this.bb_b);
    }
}



// **** Pair/Thread Functions

var numPairs = 0;

// Objects created with constructors are faster than object literals. :(
export class Pair {
    // TODO
    prevA;
    leafA;
    nextA;
    prevB;
    leafB;
    nextB;

    constructor(leafA, nextA, leafB, nextB) {
        this.prevA = null;
        this.leafA = leafA;
        this.nextA = nextA;

        this.prevB = null;
        this.leafB = leafB;
        this.nextB = nextB;
    }

    recycle(tree) {
        this.prevA = tree.pooledPairs;
        tree.pooledPairs = this;
    }
}












function unlinkThread(prev, leaf, next) {
    if (next) {
        if (next.leafA === leaf) next.prevA = prev; else next.prevB = prev;
    }

    if (prev) {
        if (prev.leafA === leaf) prev.nextA = next; else prev.nextB = next;
    } else {
        leaf.pairs = next;
    }
}

function pairInsert(a, b, tree) {
    const nextA = a.pairs;
    const nextB = b.pairs;
    const pair = tree.makePair(a, nextA, b, nextB);
    a.pairs = b.pairs = pair;

    if (nextA) {
        if (nextA.leafA === a) nextA.prevA = pair; else nextA.prevB = pair;
    }

    if (nextB) {
        if (nextB.leafA === b) nextB.prevA = pair; else nextB.prevB = pair;
    }
}


function bbTreeMergedArea(a, b) {
    return (
        (Math.max(a.bb_r, b.bb_r) - Math.min(a.bb_l, b.bb_l)) *
        (Math.max(a.bb_t, b.bb_t) - Math.min(a.bb_b, b.bb_b))
    );
};


// **** Subtree Functions

// Would it be better to make these functions instance methods on Node and Leaf?

function bbProximity(a, b) {
    return (
        Math.abs(a.bb_l + a.bb_r - b.bb_l - b.bb_r) +
        Math.abs(a.bb_b + a.bb_t - b.bb_b - b.bb_t)
    );
};



function subtreeInsert(subtree, leaf, tree) {
    //	var s = new Error().stack;
    //	traces[s] = traces[s] ? traces[s]+1 : 1;

    if (subtree == null) {
        return leaf;
    } else if (subtree.isLeaf) {
        return tree.makeNode(leaf, subtree);
    } else {
        let cost_a = subtree.B.bbArea() + bbTreeMergedArea(subtree.A, leaf);
        let cost_b = subtree.A.bbArea() + bbTreeMergedArea(subtree.B, leaf);

        if (cost_a === cost_b) {
            cost_a = bbProximity(subtree.A, leaf);
            cost_b = bbProximity(subtree.B, leaf);
        }

        if (cost_b < cost_a) {
            subtree.setB(subtreeInsert(subtree.B, leaf, tree));
        } else {
            subtree.setA(subtreeInsert(subtree.A, leaf, tree));
        }

        //		subtree.bb = bbMerge(subtree.bb, leaf.bb);
        subtree.bb_l = Math.min(subtree.bb_l, leaf.bb_l);
        subtree.bb_b = Math.min(subtree.bb_b, leaf.bb_b);
        subtree.bb_r = Math.max(subtree.bb_r, leaf.bb_r);
        subtree.bb_t = Math.max(subtree.bb_t, leaf.bb_t);

        return subtree;
    }
}
function subtreeQuery(subtree, bb, func) {
    //if(bbIntersectsBB(subtree.bb, bb)){
    if (subtree.intersectsBB(bb)) {
        if (subtree.isLeaf) {
            func(subtree.obj);
        } else {
            subtreeQuery(subtree.A, bb, func);
            subtreeQuery(subtree.B, bb, func);
        }
    }
}

/// Returns the fraction along the segment query the node hits. Returns Infinity if it doesn't hit.
function nodeSegmentQuery(node, a, b)
{
	var idx = 1/(b.x - a.x);
	var tx1 = (node.bb_l == a.x ? -Infinity : (node.bb_l - a.x) * idx);
	var tx2 = (node.bb_r == a.x ?  Infinity : (node.bb_r - a.x) * idx);
	var txmin = Math.min(tx1, tx2);
	var txmax = Math.max(tx1, tx2);
	
	var idy = 1/(b.y - a.y);
	var ty1 = (node.bb_b == a.y ? -Infinity : (node.bb_b - a.y) * idy);
	var ty2 = (node.bb_t == a.y ?  Infinity : (node.bb_t - a.y) * idy);
	var tymin = Math.min(ty1, ty2);
	var tymax = Math.max(ty1, ty2);
	
	if (tymin <= txmax && txmin <= tymax){
		var min_ = Math.max(txmin, tymin);
		var max_ = Math.min(txmax, tymax);
		
		if (0.0 <= max_ && min_ <= 1.0) return max(min_, 0.0);
	}
	
	return Infinity;
};


function subtreeSegmentQuery(subtree, a, b, t_exit, func)
{
	if(subtree.isLeaf){
		return func(subtree.obj);
	} else {
		var t_a = nodeSegmentQuery(subtree.A, a, b);
		var t_b = nodeSegmentQuery(subtree.B, a, b);
		
		if(t_a < t_b){
            if (t_a < t_exit) {
                t_exit = Math.min(
                    t_exit, subtreeSegmentQuery(subtree.A, a, b, t_exit, func),
                );
            }
            if (t_b < t_exit) {
                t_exit = Math.min(
                    t_exit, subtreeSegmentQuery(subtree.B, a, b, t_exit, func),
                );
            }
		} else {
            if (t_b < t_exit) {
                t_exit = Math.min(
                    t_exit, subtreeSegmentQuery(subtree.B, a, b, t_exit, func),
                );
            }
            if (t_a < t_exit) {
                t_exit = Math.min(
                    t_exit, subtreeSegmentQuery(subtree.A, a, b, t_exit, func),
                );
            }
		}
		
		return t_exit;
	}
};




function subtreeRemove(subtree, leaf, tree) {
    if (leaf == subtree) {
        return null;
    } else {
        const parent = leaf.parent;
        if (parent == subtree) {
            const other = subtree.otherChild(leaf);
            other.parent = subtree.parent;
            subtree.recycle(tree);
            return other;
        } else {
            parent.parent.replaceChild(parent, parent.otherChild(leaf), tree);
            return subtree;
        }
    }
}

// **** Marking Functions

/*
typedef struct MarkContext {
	bbTree *tree;
	Node *staticRoot;
	cpSpatialIndexQueryFunc func;
} MarkContext;
*/

function bbTreeIntersectsNode(a, b) {
    return (
        a.bb_l <= b.bb_r &&
        b.bb_l <= a.bb_r &&
        a.bb_b <= b.bb_t &&
        b.bb_b <= a.bb_t
    );
};




// **** Misc
// **** Tree Optimization

function bbTreeMergedArea2(node, l, b, r, t)
{
    return (
        (Math.max(node.bb_r, r) - Math.min(node.bb_l, l)) *
        (Math.max(node.bb_t, t) - Math.min(node.bb_b, b))
    );
};

function partitionNodes(tree, nodes, offset, count) {
    if (count == 1) {
        return nodes[offset];
    } else if (count == 2) {
        return tree.makeNode(nodes[offset], nodes[offset + 1]);
    }

    // Find the AABB for these nodes
    //var bb = nodes[offset].bb;
    var node = nodes[offset];
    let bb_l = node.bb_l;
    let bb_b = node.bb_b;
    let bb_r = node.bb_r;
    let bb_t = node.bb_t;

    const end = offset + count;
    for (var i = offset + 1; i < end; i++) {
        //bb = bbMerge(bb, nodes[i].bb);
        node = nodes[i];
        bb_l = Math.min(bb_l, node.bb_l);
        bb_b = Math.min(bb_b, node.bb_b);
        bb_r = Math.max(bb_r, node.bb_r);
        bb_t = Math.max(bb_t, node.bb_t);
    }

    // Split it on it's longest axis
    const splitWidth = (bb_r - bb_l > bb_t - bb_b);

    // Sort the bounds and use the median as the splitting point
    const bounds = new Array(count * 2);
    if (splitWidth) {
        for (var i = offset; i < end; i++) {
            bounds[2 * i + 0] = nodes[i].bb_l;
            bounds[2 * i + 1] = nodes[i].bb_r;
        }
    } else {
        for (var i = offset; i < end; i++) {
            bounds[2 * i + 0] = nodes[i].bb_b;
            bounds[2 * i + 1] = nodes[i].bb_t;
        }
    }

    bounds.sort((a, b) => // This might run faster if the function was moved out into the global scope.
        a - b);
    const split = (bounds[count - 1] + bounds[count]) * 0.5; // use the median as the split

    // Generate the child BBs
    //var a = bb, b = bb;
    const a_l = bb_l;

    const a_b = bb_b;
    let a_r = bb_r;
    let a_t = bb_t;
    let b_l = bb_l;
    let b_b = bb_b;
    const b_r = bb_r;
    const b_t = bb_t;

    if (splitWidth) a_r = b_l = split; else a_t = b_b = split;

    // Partition the nodes
    let right = end;
    for (let left = offset; left < right;) {
        var node = nodes[left];
        //	if(bbMergedArea(node.bb, b) < bbMergedArea(node.bb, a)){
        if (bbTreeMergedArea2(node, b_l, b_b, b_r, b_t) < bbTreeMergedArea2(node, a_l, a_b, a_r, a_t)) {
            right--;
            nodes[left] = nodes[right];
            nodes[right] = node;
        } else {
            left++;
        }
    }

    if (right == count) {
        var node = null;
        for (var i = offset; i < end; i++) node = subtreeInsert(node, nodes[i], tree);
        return node;
    }

    // Recurse and build the node!
    return NodeNew(tree,
        partitionNodes(tree, nodes, offset, right - offset),
        partitionNodes(tree, nodes, right, end - right)
    );
}

//static void
//bbTreeOptimizeIncremental(bbTree *tree, int passes)
//{
//	for(int i=0; i<passes; i++){
//		Node *root = tree.root;
//		Node *node = root;
//		int bit = 0;
//		unsigned int path = tree.opath;
//		
//		while(!NodeIsLeaf(node)){
//			node = (path&(1<<bit) ? node.a : node.b);
//			bit = (bit + 1)&(sizeof(unsigned int)*8 - 1);
//		}
//		
//		root = subtreeRemove(root, node, tree);
//		tree.root = subtreeInsert(root, node, tree);
//	}
//}
// **** Debug Draw

export function nodeRender(node, depth) {
    if (!node.isLeaf && depth <= 10) {
        nodeRender(node.A, depth + 1);
        nodeRender(node.B, depth + 1);
    }

    let str = '';
    for (let i = 0; i < depth; i++) {
        str += ' ';
    }

    console.log(str + node.bb_b + ' ' + node.bb_t);
}
