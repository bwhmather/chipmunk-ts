/* Copyright (c) 2007 Scott Lembcke
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

/// **** Sleeping Functions
var componentRoot = function(body) {
    return (body ? body.nodeRoot : null);
};

var componentActivate = function(root) {
    if (!root || !root.isSleeping(root)) return;
    assert(!root.isRogue(), "Internal Error: componentActivate() called on a rogue body.");

    var space = root.space;
    var body = root;
    while (body) {
        var next = body.nodeNext;

        body.nodeIdleTime = 0;
        body.nodeRoot = null;
        body.nodeNext = null;
        space.activateBody(body);

        body = next;
    }

    deleteObjFromList(space.sleepingComponents, root);
};
var componentAdd = function(root, body) {
    body.nodeRoot = root;

    if (body !== root) {
        body.nodeNext = root.nodeNext;
        root.nodeNext = body;
    }
};

var floodFillComponent = function(root, body) {
    // Rogue bodies cannot be put to sleep and prevent bodies they are touching from sleeping anyway.
    // Static bodies (which are a type of rogue body) are effectively sleeping all the time.
    if (!body.isRogue()) {
        var other_root = componentRoot(body);
        if (other_root == null) {
            componentAdd(root, body);
            for (var arb = body.arbiterList; arb; arb = arb.next(body)) {
                floodFillComponent(root, (body == arb.body_a ? arb.body_b : arb.body_a));
            }
            for (var constraint = body.constraintList; constraint; constraint = constraint.next(body)) {
                floodFillComponent(root, (body == constraint.a ? constraint.b : constraint.a));
            }
        } else {
            assertSoft(other_root === root, "Internal Error: Inconsistency detected in the contact graph.");
        }
    }
};

var componentActive = function(root, threshold) {
    for (var body = root; body; body = body.nodeNext) {
        if (body.nodeIdleTime < threshold) return true;
    }

    return false;
};

