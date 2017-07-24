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

import { BB, bbNewForCircle, bbIntersects2 } from './bb';
import { Body } from './body';
import { BBTree } from './bb-tree';
import { Arbiter, CollisionHandler, ContactPoint } from './arbiter';
import { Constraint } from './constraints/constraint';
import { hashPair, deleteObjFromList, assert, assertSoft } from './util';
// import { SpaceHash } from '???';
import { Contact, collideShapes } from './collision';
import { Vect, vlengthsq, vneg, vzero } from './vect';
import {
    componentRoot, componentActive, floodFillComponent,
} from './space-components';
import { Shape, NearestPointQueryInfo, SegmentQueryInfo } from './shapes';


const defaultCollisionHandler = new CollisionHandler();

function assertSpaceUnlocked(space: Space): void {
    assert(!space.locked, "This addition/removal cannot be done safely during a call to cpSpaceStep() \
 or during a query. Put these calls into a post-step callback.");
}

// **** All Important cpSpaceStep() Function
function updateFunc(shape: Shape) {
    const body = shape.body;
    shape.update(body.p, body.rot);
}

/// Basic Unit of Simulation in Chipmunk
export class Space {

    stamp: number;
    curr_dt: number;

    bodies: Body[];
    rousedBodies: Body[];
    // TODO
    sleepingComponents: Body[];

    staticShapes: BBTree;
    activeShapes: BBTree;

    arbiters: Arbiter[];
    cachedArbiters: Map<string, Arbiter>;

    constraints: Constraint[];

    locked: number;

    collisionHandlers: Map<string, CollisionHandler>;
    defaultHandler: CollisionHandler;

    postStepCallbacks: Array<() => any>;

    /// Number of iterations to use in the impulse solver to solve contacts.
    iterations: number;

    /// Gravity to pass to rigid bodies when integrating velocity.
    gravity: Vect;

    /// Damping rate expressed as the fraction of velocity bodies retain each second.
    /// A value of 0.9 would mean that each body's velocity will drop 10% per second.
    /// The default value is 1.0, meaning no damping is applied.
    /// @note This damping value is different than those of cpDampedSpring and cpDampedRotarySpring.
    damping: number;

    /// Speed threshold for a body to be considered idle.
    /// The default value of 0 means to let the space guess a good threshold based on gravity.
    idleSpeedThreshold: number;

    /// Time a group of bodies must remain idle in order to fall asleep.
    /// Enabling sleeping also implicitly enables the the contact graph.
    /// The default value of Infinity disables the sleeping algorithm.
    sleepTimeThreshold: number;

    /// Amount of encouraged penetration between colliding shapes..
    /// Used to reduce oscillating contacts and keep the collision cache warm.
    /// Defaults to 0.1. If you have poor simulation quality,
    /// increase this number as much as possible without allowing visible amounts of overlap.
    collisionSlop: number;

    /// Determines how fast overlapping shapes are pushed apart.
    /// Expressed as a fraction of the error remaining after each second.
    /// Defaults to pow(1.0 - 0.1, 60.0) meaning that Chipmunk fixes 10% of overlap each frame at 60Hz.
    collisionBias: number;

    /// Number of frames that contact information should persist.
    /// Defaults to 3. There is probably never a reason to change this value.
    collisionPersistence: number;

    /// Rebuild the contact graph during each step. Must be enabled to use the cpBodyEachArbiter() function.
    /// Disabled by default for a small performance boost. Enabled implicitly when the sleeping feature is enabled.
    enableContactGraph: boolean;

    /// The designated static body for this space.
    /// You can modify this body, or replace it with your own static body.
    /// By default it points to a statically allocated cpBody in the cpSpace struct.
    staticBody: Body;

    // Cache the collideShapes callback function for the space.
    // TODO
    collideShapes: (a: Shape, b: Shape) => void;

    constructor() {
        this.stamp = 0;
        this.curr_dt = 0;

        this.bodies = [];
        this.rousedBodies = [];
        this.sleepingComponents = [];

        this.staticShapes = new BBTree(null);
        this.activeShapes = new BBTree(this.staticShapes);

        this.arbiters = [];
        this.cachedArbiters = new Map();
        //this.pooledArbiters = [];

        this.constraints = [];

        this.locked = 0;

        this.collisionHandlers = new Map();
        this.defaultHandler = defaultCollisionHandler;

        this.postStepCallbacks = [];

        /// Number of iterations to use in the impulse solver to solve contacts.
        this.iterations = 10;

        /// Gravity to pass to rigid bodies when integrating velocity.
        this.gravity = vzero;

        /// Damping rate expressed as the fraction of velocity bodies retain each second.
        /// A value of 0.9 would mean that each body's velocity will drop 10% per second.
        /// The default value is 1.0, meaning no damping is applied.
        /// @note This damping value is different than those of cpDampedSpring and cpDampedRotarySpring.
        this.damping = 1;

        /// Speed threshold for a body to be considered idle.
        /// The default value of 0 means to let the space guess a good threshold based on gravity.
        this.idleSpeedThreshold = 0;

        /// Time a group of bodies must remain idle in order to fall asleep.
        /// Enabling sleeping also implicitly enables the the contact graph.
        /// The default value of Infinity disables the sleeping algorithm.
        this.sleepTimeThreshold = Infinity;

        /// Amount of encouraged penetration between colliding shapes..
        /// Used to reduce oscillating contacts and keep the collision cache warm.
        /// Defaults to 0.1. If you have poor simulation quality,
        /// increase this number as much as possible without allowing visible amounts of overlap.
        this.collisionSlop = 0.1;

        /// Determines how fast overlapping shapes are pushed apart.
        /// Expressed as a fraction of the error remaining after each second.
        /// Defaults to pow(1.0 - 0.1, 60.0) meaning that Chipmunk fixes 10% of overlap each frame at 60Hz.
        this.collisionBias = (1 - 0.1) ** 60;

        /// Number of frames that contact information should persist.
        /// Defaults to 3. There is probably never a reason to change this value.
        this.collisionPersistence = 3;

        /// Rebuild the contact graph during each step. Must be enabled to use the cpBodyEachArbiter() function.
        /// Disabled by default for a small performance boost. Enabled implicitly when the sleeping feature is enabled.
        this.enableContactGraph = false;

        /// The designated static body for this space.
        /// You can modify this body, or replace it with your own static body.
        /// By default it points to a statically allocated cpBody in the cpSpace struct.
        this.staticBody = new Body(Infinity, Infinity);
        this.staticBody.nodeIdleTime = Infinity;

        // Cache the collideShapes callback function for the space.
        this.collideShapes = this.makeCollideShapes();
    }

    getCurrentTimeStep(): number {
        return this.curr_dt;
    }
    setIterations(iter: number): void {
        this.iterations = iter;
    }

    /// returns true from inside a callback and objects cannot be added/removed.
    isLocked(): boolean {
        return !!this.locked;
    }

    // **** Collision handler function management

    /// Set a collision handler to be used whenever the two shapes with the given collision types collide.
    /// You can pass null for any function you don't want to implement.
    addCollisionHandler(
        a: number, b: number,
        begin: (arg: Arbiter, space: Space) => any,
        preSolve: (arg: Arbiter, space: Space) => any,
        postSolve: (arg: Arbiter, space: Space) => any,
        separate: (arg: Arbiter, space: Space) => any,
    ) {
        assertSpaceUnlocked(this);

        // Remove any old function so the new one will get added.
        this.removeCollisionHandler(a, b);

        const handler = new CollisionHandler();
        handler.a = a;
        handler.b = b;
        if (begin) handler.begin = begin;
        if (preSolve) handler.preSolve = preSolve;
        if (postSolve) handler.postSolve = postSolve;
        if (separate) handler.separate = separate;

        this.collisionHandlers.set(hashPair(a, b), handler);
    }

    /// Unset a collision handler.
    removeCollisionHandler(a: number, b: number) {
        assertSpaceUnlocked(this);

        this.collisionHandlers.delete(hashPair(a, b));
    }

    /// Set a default collision handler for this space.
    /// The default collision handler is invoked for each colliding pair of shapes
    /// that isn't explicitly handled by a specific collision handler.
    /// You can pass null for any function you don't want to implement.
    setDefaultCollisionHandler(
        begin: (arg: Arbiter, space: Space) => any,
        preSolve: (arg: Arbiter, space: Space) => any,
        postSolve: (arg: Arbiter, space: Space) => any,
        separate: (arg: Arbiter, space: Space) => any,
    ) {
        assertSpaceUnlocked(this);

        const handler = new CollisionHandler();
        if (begin) handler.begin = begin;
        if (preSolve) handler.preSolve = preSolve;
        if (postSolve) handler.postSolve = postSolve;
        if (separate) handler.separate = separate;

        this.defaultHandler = handler;
    }

    lookupHandler(a: number, b: number): CollisionHandler {
        return this.collisionHandlers.get(hashPair(a, b)) || this.defaultHandler;
    }

    // **** Body, Shape, and Joint Management

    /// Add a collision shape to the simulation.
    /// If the shape is attached to a static body, it will be added as a static shape.
    addShape(shape: Shape): Shape {
        const body = shape.body;
        if (body.isStatic()) return this.addStaticShape(shape);

        assert(!shape.space, "This shape is already added to a space and cannot be added to another.");
        assertSpaceUnlocked(this);

        body.activate();
        body.addShape(shape);

        shape.update(body.p, body.rot);
        this.activeShapes.insert(shape);
        shape.space = this;

        return shape;
    }

    /// Explicity add a shape as a static shape to the simulation.
    addStaticShape(shape: Shape): Shape {
        assert(!shape.space, "This shape is already added to a space and cannot be added to another.");
        assertSpaceUnlocked(this);

        const body = shape.body;
        body.addShape(shape);

        shape.update(body.p, body.rot);
        this.staticShapes.insert(shape);
        shape.space = this;

        return shape;
    }

    /// Add a rigid body to the simulation.
    addBody(body: Body): Body {
        assert(!body.isStatic(), "Static bodies cannot be added to a space as they are not meant to be simulated.");
        assert(!body.space, "This body is already added to a space and cannot be added to another.");
        assertSpaceUnlocked(this);

        this.bodies.push(body);
        body.space = this;

        return body;
    }

    /// Add a constraint to the simulation.
    addConstraint(constraint: Constraint): Constraint {
        assert(!constraint.space, "This shape is already added to a space and cannot be added to another.");
        assertSpaceUnlocked(this);

        const a = constraint.a;
        const b = constraint.b;

        a.activate();
        b.activate();
        this.constraints.push(constraint);

        // Push onto the heads of the bodies' constraint lists
        constraint.next_a = a.constraintList; a.constraintList = constraint;
        constraint.next_b = b.constraintList; b.constraintList = constraint;
        constraint.space = this;

        return constraint;
    }

    filterArbiters(body: Body, filter: Shape): void {
        this.cachedArbiters.forEach((arb: Arbiter, hash: string) => {
            // Match on the filter shape, or if it's null the filter body
            if (
                (body === arb.body_a && (filter === arb.a || filter === null)) ||
                (body === arb.body_b && (filter === arb.b || filter === null))
            ) {
                // Call separate when removing shapes.
                if (filter && arb.state !== 'cached') arb.callSeparate(this);

                arb.unthread();

                deleteObjFromList(this.arbiters, arb);
                //this.pooledArbiters.push(arb);

                this.cachedArbiters.delete(hash);
            }
        });
    }

    /// Remove a collision shape from the simulation.
    removeShape(shape: Shape): void {
        const body = shape.body;
        if (body.isStatic()) {
            this.removeStaticShape(shape);
        } else {
            assert(this.containsShape(shape),
                "Cannot remove a shape that was not added to the space. (Removed twice maybe?)");
            assertSpaceUnlocked(this);

            body.activate();
            body.removeShape(shape);
            this.filterArbiters(body, shape);
            this.activeShapes.remove(shape);
            shape.space = null;
        }
    }

    /// Remove a collision shape added using addStaticShape() from the simulation.
    removeStaticShape(shape: Shape): void {
        assert(this.containsShape(shape),
            "Cannot remove a static or sleeping shape that was not added to the space. (Removed twice maybe?)");
        assertSpaceUnlocked(this);

        const body = shape.body;
        if (body.isStatic()) body.activateStatic(shape);
        body.removeShape(shape);
        this.filterArbiters(body, shape);
        this.staticShapes.remove(shape);
        shape.space = null;
    }

    /// Remove a rigid body from the simulation.
    removeBody(body: Body): void {
        assert(this.containsBody(body),
            "Cannot remove a body that was not added to the space. (Removed twice maybe?)");
        assertSpaceUnlocked(this);

        body.activate();
        //	this.filterArbiters(body, null);
        deleteObjFromList(this.bodies, body);
        body.space = null;
    }

    /// Remove a constraint from the simulation.
    removeConstraint(constraint: Constraint): void {
        assert(this.containsConstraint(constraint),
            "Cannot remove a constraint that was not added to the space. (Removed twice maybe?)");
        assertSpaceUnlocked(this);

        constraint.a.activate();
        constraint.b.activate();
        deleteObjFromList(this.constraints, constraint);

        constraint.a.removeConstraint(constraint);
        constraint.b.removeConstraint(constraint);
        constraint.space = null;
    }

    /// Test if a collision shape has been added to the space.
    containsShape(shape: Shape): boolean {
        return shape.space === this;
    }

    /// Test if a rigid body has been added to the space.
    containsBody(body: Body): boolean {
        return body.space == this;
    }

    /// Test if a constraint has been added to the space.
    containsConstraint(constraint: Constraint): boolean {
        return constraint.space == this;
    }

    uncacheArbiter(arb: Arbiter): void {
        this.cachedArbiters.delete(hashPair(arb.a.hashid, arb.b.hashid));
        deleteObjFromList(this.arbiters, arb);
    }

    // **** Iteration

    /// Call @c func for each body in the space.
    eachBody(func: (body: Body) => any): void {
        this.lock(); {
            const bodies = this.bodies;

            for (var i = 0; i < bodies.length; i++) {
                func(bodies[i]);
            }

            const components = this.sleepingComponents;
            for (var i = 0; i < components.length; i++) {
                const root = components[i];

                let body = root;
                while (body) {
                    const next = body.nodeNext;
                    func(body);
                    body = next;
                }
            }
        } this.unlock(true);
    }

    /// Call @c func for each shape in the space.
    eachShape(func: (shape: Shape) => any): void {
        this.lock(); {
            this.activeShapes.each(func);
            this.staticShapes.each(func);
        } this.unlock(true);
    }

    /// Call @c func for each shape in the space.
    eachConstraint(func: (constraint: Constraint) => any): void {
        this.lock(); {
            const constraints = this.constraints;

            for (let i = 0; i < constraints.length; i++) {
                func(constraints[i]);
            }
        } this.unlock(true);
    }

    // **** Spatial Index Management

    /// Update the collision detection info for the static shapes in the space.
    reindexStatic(): void {
        assert(!this.locked, "You cannot manually reindex objects while the space is locked. Wait until the current query or step is complete.");

        this.staticShapes.each((shape: Shape) => {
            const body = shape.body;
            shape.update(body.p, body.rot);
        });
        this.staticShapes.reindex();
    }

    /// Update the collision detection data for a specific shape in the space.
    reindexShape(shape: Shape): void {
        assert(!this.locked, "You cannot manually reindex objects while the space is locked. Wait until the current query or step is complete.");

        const body = shape.body;
        shape.update(body.p, body.rot);

        // attempt to rehash the shape in both hashes
        this.activeShapes.reindexObject(shape);
        this.staticShapes.reindexObject(shape);
    }

    /// Update the collision detection data for all shapes attached to a body.
    reindexShapesForBody(body: Body): void {
        body.eachShape((shape) => {
            this.reindexShape(shape);
        });
    }

    activateBody(body: Body): void {
        assert(!body.isRogue(), "Internal error: Attempting to activate a rogue body.");

        if (this.locked) {
            // cpSpaceActivateBody() is called again once the space is unlocked
            if (this.rousedBodies.indexOf(body) === -1) this.rousedBodies.push(body);
        } else {
            this.bodies.push(body);

            body.eachShape((shape) => {
                this.staticShapes.remove(shape);
                this.activeShapes.insert(shape);
            });

            body.eachArbiter((arbiter) => {
                var bodyA = arbiter.body_a;
                if (body === bodyA || bodyA.isStatic()) {
                    //var contacts = arb.contacts;

                    // Restore contact values back to the space's contact buffer memory
                    //arb.contacts = cpContactBufferGetArray(this);
                    //memcpy(arb.contacts, contacts, numContacts*sizeof(cpContact));
                    //cpSpacePushContacts(this, numContacts);

                    // Reinsert the arbiter into the arbiter cache
                    const a = arbiter.a;

                    const b = arbiter.b;
                    this.cachedArbiters.set(hashPair(a.hashid, b.hashid), arbiter);

                    // Update the arbiter's state
                    arbiter.stamp = this.stamp;
                    arbiter.handler = this.lookupHandler(a.collision_type, b.collision_type);
                    this.arbiters.push(arbiter);
                }
            });

            body.eachConstraint((constraint) => {
                var bodyA = constraint.a;
                if (body === bodyA || bodyA.isStatic()) {
                    this.constraints.push(constraint);
                }
            });
        }
    }

    deactivateBody(body: Body): void {
        assert(!body.isRogue(), "Internal error: Attempting to deactivate a rogue body.");

        deleteObjFromList(this.bodies, body);

        body.eachShape((shape) => {
            this.activeShapes.remove(shape);
            this.staticShapes.insert(shape);
        });

        body.eachArbiter((arbiter) => {
            var bodyA = arbiter.body_a;
            if (body === bodyA || bodyA.isStatic()) {
                this.uncacheArbiter(arbiter);

                // Save contact values to a new block of memory so they won't time out
                //size_t bytes = arb.numContacts*sizeof(cpContact);
                //cpContact *contacts = (cpContact *)cpcalloc(1, bytes);
                //memcpy(contacts, arb.contacts, bytes);
                //arb.contacts = contacts;
            }
        });

        body.eachConstraint((constraint) => {
            var bodyA = constraint.a;
            if (body === bodyA || bodyA.isStatic()) deleteObjFromList(this.constraints, constraint);
        });
    }

    processComponents(dt: number): void {
        var sleep = (this.sleepTimeThreshold !== Infinity);
        var bodies = this.bodies;

        // These checks can be removed at some stage (if DEBUG == undefined)
        for (var i = 0; i < bodies.length; i++) {
            var body = bodies[i];

            assertSoft(body.nodeNext === null, "Internal Error: Dangling next pointer detected in contact graph.");
            assertSoft(body.nodeRoot === null, "Internal Error: Dangling root pointer detected in contact graph.");
        }

        // Calculate the kinetic energy of all the bodies
        if (sleep) {
            var dv = this.idleSpeedThreshold;
            var dvsq = (dv ? dv * dv : vlengthsq(this.gravity) * dt * dt);

            for (var i = 0; i < bodies.length; i++) {
                var body = bodies[i];

                // Need to deal with infinite mass objects
                var keThreshold = (dvsq ? body.m * dvsq : 0);
                body.nodeIdleTime = (body.kineticEnergy() > keThreshold ? 0 : body.nodeIdleTime + dt);
            }
        }

        // Awaken any sleeping bodies found and then push arbiters to the bodies' lists.
        var arbiters = this.arbiters;
        for (var i = 0, count = arbiters.length; i < count; i++) {
            var arb = arbiters[i];
            var a = arb.body_a, b = arb.body_b;

            if (sleep) {
                if ((b.isRogue() && !b.isStatic()) || a.isSleeping()) a.activate();
                if ((a.isRogue() && !a.isStatic()) || b.isSleeping()) b.activate();
            }

            a.pushArbiter(arb);
            b.pushArbiter(arb);
        }

        if (sleep) {
            // Bodies should be held active if connected by a joint to a non-static rouge body.
            var constraints = this.constraints;
            for (var i = 0; i < constraints.length; i++) {
                var constraint = constraints[i];
                var a = constraint.a, b = constraint.b;

                if (b.isRogue() && !b.isStatic()) a.activate();
                if (a.isRogue() && !a.isStatic()) b.activate();
            }

            // Generate components and deactivate sleeping ones
            for (var i = 0; i < bodies.length;) {
                var body = bodies[i];

                if (componentRoot(body) === null) {
                    // Body not in a component yet. Perform a DFS to flood fill mark
                    // the component in the contact graph using this body as the root.
                    floodFillComponent(body, body);

                    // Check if the component should be put to sleep.
                    if (!componentActive(body, this.sleepTimeThreshold)) {
                        this.sleepingComponents.push(body);
                        for (var other = body; other; other = other.nodeNext) {
                            this.deactivateBody(other);
                        }

                        // deactivateBody() removed the current body from the list.
                        // Skip incrementing the index counter.
                        continue;
                    }
                }

                i++;

                // Only sleeping bodies retain their component node pointers.
                body.nodeRoot = null;
                body.nodeNext = null;
            }
        }
    }

    activateShapesTouchingShape(shape: Shape): void {
        if (this.sleepTimeThreshold !== Infinity) {
            this.shapeQuery(shape, function (shape, points) {
                shape.body.activate();
            });
        }
    }

    pointQuery(
        point: Vect, layers: number, group: number,
        func: (shape: Shape) => any,
    ) {
        const helper = (shape: Shape) => {
            if (
                !(shape.group && group === shape.group) && (layers & shape.layers) &&
                shape.pointQuery(point)
            ) {
                func(shape);
            }
        };

        const bb = new BB(point.x, point.y, point.x, point.y);
        this.lock(); {
            this.activeShapes.query(bb, helper);
            this.staticShapes.query(bb, helper);
        } this.unlock(true);
    }

    /// Query the space at a point and return the first shape found. Returns null if no shapes were found.
    pointQueryFirst(point: Vect, layers: number, group: number): Shape {
        let outShape = null;
        this.pointQuery(point, layers, group, shape => {
            if (!shape.sensor) outShape = shape;
        });

        return outShape;
    }

    // Nearest point query functions

    nearestPointQuery(
        point: Vect, maxDistance: number,
        layers: number, group: number,
        func: (shape: Shape, d: number, p: Vect) => any,
    ): void {
        const helper = (shape: Shape) => {
            if (!(shape.group && group === shape.group) && (layers & shape.layers)) {
                const info = shape.nearestPointQuery(point);

                if (info.d < maxDistance) func(shape, info.d, info.p);
            }
        };

        const bb = bbNewForCircle(point, maxDistance);

        this.lock(); {
            this.activeShapes.query(bb, helper);
            this.staticShapes.query(bb, helper);
        } this.unlock(true);
    }

    // Unlike the version in chipmunk, this returns a NearestPointQueryInfo object. Use its .shape
    // property to get the actual shape.
    nearestPointQueryNearest(
        point: Vect, maxDistance: number,
        layers: number, group: number,
    ): NearestPointQueryInfo {
        let out: NearestPointQueryInfo = null;

        const helper = (shape: Shape) => {
            if (!(shape.group && group === shape.group) && (layers & shape.layers) && !shape.sensor) {
                const info = shape.nearestPointQuery(point);

                if (info.d < maxDistance && (!out || info.d < out.d)) out = info;
            }
        };

        const bb = bbNewForCircle(point, maxDistance);
        this.activeShapes.query(bb, helper);
        this.staticShapes.query(bb, helper);

        return out;
    }

    /// Perform a directed line segment query (like a raycast) against the space calling @c func for each shape intersected.
    segmentQuery(
        start: Vect, end: Vect,
        layers: number, group: number,
        func: (shape: Shape, t: number, n: Vect) => any,
    ) {
        const helper = (shape: Shape) => {
            let info;

            if (
                !(shape.group && group === shape.group) && (layers & shape.layers) &&
                (info = shape.segmentQuery(start, end))
            ) {
                func(shape, info.t, info.n);
            }

            return 1;
        };

        this.lock(); {
            this.staticShapes.segmentQuery(start, end, 1, helper);
            this.activeShapes.segmentQuery(start, end, 1, helper);
        } this.unlock(true);
    }

    /// Perform a directed line segment query (like a raycast) against the space and return the first shape hit.
    /// Returns null if no shapes were hit.
    segmentQueryFirst(
        start: Vect, end: Vect,
        layers: number, group: number,
    ): SegmentQueryInfo {
        let out: SegmentQueryInfo = null;

        const helper = (shape: Shape) => {
            let info;

            if (
                !(shape.group && group === shape.group) && (layers & shape.layers) &&
                !shape.sensor &&
                (info = shape.segmentQuery(start, end)) &&
                (out === null || info.t < out.t)
            ) {
                out = info;
            }

            return out ? out.t : 1;
        };

        this.staticShapes.segmentQuery(start, end, 1, helper);
        this.activeShapes.segmentQuery(start, end, out ? out.t : 1, helper);

        return out;
    }

    /// Perform a fast rectangle query on the space calling @c func for each shape found.
    /// Only the shape's bounding boxes are checked for overlap, not their full shape.
    bbQuery(
        bb: BB,
        layers: number, group: number,
        func: (shape: Shape) => any,
    ): void {
        const helper = (shape: Shape) => {
            if (
                !(shape.group && group === shape.group) && (layers & shape.layers) &&
                bbIntersects2(bb, shape.bb_l, shape.bb_b, shape.bb_r, shape.bb_t)
            ) {
                func(shape);
            }
        };

        this.lock(); {
            this.activeShapes.query(bb, helper);
            this.staticShapes.query(bb, helper);
        } this.unlock(true);
    }

    /// Query a space for any shapes overlapping the given shape and call @c func for each shape found.
    shapeQuery(
        shape: Shape,
        func: (shape: Shape, contacts: Contact[]) => any,
    ): boolean {
        const body = shape.body;

        //var bb = (body ? shape.update(body.p, body.rot) : shape.bb);
        if (body) {
            shape.update(body.p, body.rot);
        }
        const bb = new BB(shape.bb_l, shape.bb_b, shape.bb_r, shape.bb_t);

        //shapeQueryContext context = {func, data, false};
        let anyCollision = false;

        const helper = (b: Shape) => {
            const a = shape;
            // Reject any of the simple cases
            if (
                (a.group && a.group === b.group) ||
                !(a.layers & b.layers) ||
                a === b
            ) return;

            let contacts;

            // Shape 'a' should have the lower shape type. (required by collideShapes() )
            if (a.collisionCode <= b.collisionCode) {
                contacts = collideShapes(a, b);
            } else {
                contacts = collideShapes(b, a);
                for (var i = 0; i < contacts.length; i++) contacts[i].n = vneg(contacts[i].n);
            }

            if (contacts.length) {
                anyCollision = !(a.sensor || b.sensor);

                if (func) {
                    const set = new Array(contacts.length);
                    for (var i = 0; i < contacts.length; i++) {
                        set[i] = new ContactPoint(contacts[i].p, contacts[i].n, contacts[i].dist);
                    }

                    func(b, set);
                }
            }
        };

        this.lock(); {
            this.activeShapes.query(bb, helper);
            this.staticShapes.query(bb, helper);
        } this.unlock(true);

        return anyCollision;
    }

    /// Schedule a post-step callback to be called when cpSpaceStep() finishes.
    addPostStepCallback(func: () => any): void {
        assertSoft(this.locked,
            "Adding a post-step callback when the space is not locked is unnecessary. " +
            "Post-step callbacks will not called until the end of the next call to cpSpaceStep() or the next query.");

        this.postStepCallbacks.push(func);
    }

    runPostStepCallbacks(): void {
        // Don't cache length because post step callbacks may add more post step callbacks
        // directly or indirectly.
        for (let i = 0; i < this.postStepCallbacks.length; i++) {
            this.postStepCallbacks[i]();
        }
        this.postStepCallbacks = [];
    }

    // **** Locking Functions

    lock(): void {
        this.locked++;
    }

    unlock(runPostStep: boolean): void {
        this.locked--;
        assert(this.locked >= 0, "Internal Error: Space lock underflow.");

        if (this.locked === 0 && runPostStep) {
            const waking = this.rousedBodies;
            for (let i = 0; i < waking.length; i++) {
                this.activateBody(waking[i]);
            }

            waking.length = 0;

            this.runPostStepCallbacks();
        }
    }

    // Callback from the spatial hash.
    makeCollideShapes(): (a: Shape, b: Shape) => void {
        // It would be nicer to use .bind() or something, but this is faster.
        const space_ = this;
        return (a: Shape, b: Shape) => {
            const space = space_;

            // Reject any of the simple cases
            if (
                // BBoxes must overlap
                //!bbIntersects(a.bb, b.bb)
                !(a.bb_l <= b.bb_r && b.bb_l <= a.bb_r && a.bb_b <= b.bb_t && b.bb_b <= a.bb_t)
                // Don't collide shapes attached to the same body.
                || a.body === b.body
                // Don't collide objects in the same non-zero group
                || (a.group && a.group === b.group)
                // Don't collide objects that don't share at least on layer.
                || !(a.layers & b.layers)
            ) return;

            const handler = space.lookupHandler(a.collision_type, b.collision_type);

            const sensor = a.sensor || b.sensor;
            if (sensor && handler === defaultCollisionHandler) return;

            // Shape 'a' should have the lower shape type. (required by cpCollideShapes() )
            if (a.collisionCode > b.collisionCode) {
                const temp = a;
                a = b;
                b = temp;
            }

            // Narrow-phase collision detection.
            //cpContact *contacts = cpContactBufferGetArray(space);
            //int numContacts = cpCollideShapes(a, b, contacts);
            const contacts = collideShapes(a, b);
            if (contacts.length === 0) return; // Shapes are not colliding.
            //cpSpacePushContacts(space, numContacts);

            // Get an arbiter from space.arbiterSet for the two shapes.
            // This is where the persistant contact magic comes from.
            const arbHash = hashPair(a.hashid, b.hashid);
            let arb = space.cachedArbiters.get(arbHash);
            if (!arb) {
                arb = new Arbiter(a, b)
                space.cachedArbiters.set(arbHash, arb);
            }

            arb.update(contacts, handler, a, b);

            // Call the begin function first if it's the first step
            if (arb.state == 'first-coll' && !handler.begin(arb, space)) {
                arb.ignore(); // permanently ignore the collision until separation
            }

            if (
                // Ignore the arbiter if it has been flagged
                (arb.state !== 'ignore') &&
                // Call preSolve
                handler.preSolve(arb, space) &&
                // Process, but don't add collisions for sensors.
                !sensor
            ) {
                space.arbiters.push(arb);
            } else {
                //cpSpacePopContacts(space, numContacts);

                arb.contacts = null;

                // Normally arbiters are set as used after calling the post-solve callback.
                // However, post-solve callbacks are not called for sensors or arbiters rejected from pre-solve.
                if (arb.state !== 'ignore') arb.state = 'normal';
            }

            // Time stamp the arbiter so we know it was used recently.
            arb.stamp = space.stamp;
        };
    }

    // Hashset filter func to throw away old arbiters.
    arbiterSetFilter(arb: Arbiter): boolean {
        const ticks = this.stamp - arb.stamp;

        const a = arb.body_a;
        const b = arb.body_b;

        // TODO should make an arbiter state for this so it doesn't require filtering arbiters for
        // dangling body pointers on body removal.
        // Preserve arbiters on sensors and rejected arbiters for sleeping objects.
        // This prevents errant separate callbacks from happenening.
        if (
            (a.isStatic() || a.isSleeping()) &&
            (b.isStatic() || b.isSleeping())
        ) {
            return true;
        }

        // Arbiter was used last frame, but not this one
        if (ticks >= 1 && arb.state != 'cached') {
            arb.callSeparate(this);
            arb.state = 'cached';
        }

        if (ticks >= this.collisionPersistence) {
            arb.contacts = null;

            //cpArrayPush(this.pooledArbiters, arb);
            return false;
        }

        return true;
    }

    /// Step the space forward in time by @c dt.
    step(dt: number): void {
        // don't step if the timestep is 0!
        if (dt === 0) return;

        assert(vzero.x === 0 && vzero.y === 0, "vzero is invalid");

        this.stamp++;

        const prev_dt = this.curr_dt;
        this.curr_dt = dt;

        let i;
        let j;
        let hash;
        const bodies = this.bodies;
        const constraints = this.constraints;
        const arbiters = this.arbiters;

        // Reset and empty the arbiter lists.
        for (i = 0; i < arbiters.length; i++) {
            const arb = arbiters[i];
            arb.state = 'normal';

            // If both bodies are awake, unthread the arbiter from the contact graph.
            if (!arb.body_a.isSleeping() && !arb.body_b.isSleeping()) {
                arb.unthread();
            }
        }
        arbiters.length = 0;

        this.lock(); {
            // Integrate positions
            for (i = 0; i < bodies.length; i++) {
                bodies[i].position_func(dt);
            }

            // Find colliding pairs.
            //this.pushFreshContactBuffer();
            this.activeShapes.each(updateFunc);
            this.activeShapes.reindexQuery(this.collideShapes);
        } this.unlock(false);

        // Rebuild the contact graph (and detect sleeping components if sleeping is enabled)
        this.processComponents(dt);

        this.lock(); {
            // Clear out old cached arbiters and call separate callbacks
            for (hash in this.cachedArbiters) {
                if (!this.arbiterSetFilter(this.cachedArbiters.get(hash))) {
                    this.cachedArbiters.delete(hash);
                }
            }

            // Prestep the arbiters and constraints.
            const slop = this.collisionSlop;
            const biasCoef = 1 - this.collisionBias ** dt;
            for (i = 0; i < arbiters.length; i++) {
                arbiters[i].preStep(dt, slop, biasCoef);
            }

            for (i = 0; i < constraints.length; i++) {
                const constraint = constraints[i];

                constraint.preSolve(this);
                constraint.preStep(dt);
            }

            // Integrate velocities.
            const damping = this.damping ** dt;
            const gravity = this.gravity;
            for (i = 0; i < bodies.length; i++) {
                bodies[i].velocity_func(gravity, damping, dt);
            }

            // Apply cached impulses
            const dt_coef = (prev_dt === 0 ? 0 : dt / prev_dt);
            for (i = 0; i < arbiters.length; i++) {
                arbiters[i].applyCachedImpulse(dt_coef);
            }

            for (i = 0; i < constraints.length; i++) {
                constraints[i].applyCachedImpulse(dt_coef);
            }

            // Run the impulse solver.
            for (i = 0; i < this.iterations; i++) {
                for (j = 0; j < arbiters.length; j++) {
                    arbiters[j].applyImpulse();
                }

                for (j = 0; j < constraints.length; j++) {
                    constraints[j].applyImpulse();
                }
            }

            // Run the constraint post-solve callbacks
            for (i = 0; i < constraints.length; i++) {
                constraints[i].postSolve(this);
            }

            // run the post-solve callbacks
            for (i = 0; i < arbiters.length; i++) {
                arbiters[i].handler.postSolve(arbiters[i], this);
            }
        } this.unlock(true);
    }
}
