declare namespace org.jbox2d.collision {
    /**
     * Creates an AABB object using the given bounding
     * vertices.
     * @param {org.jbox2d.common.Vec2} lowerVertex the bottom left vertex of the bounding box
     * @param maxVertex the top right vertex of the bounding box
     * @param {org.jbox2d.common.Vec2} upperVertex
     * @class
     */
    class AABB {
        /**
         * Bottom left vertex of bounding box.
         */
        lowerBound: org.jbox2d.common.Vec2;
        /**
         * Top right vertex of bounding box.
         */
        upperBound: org.jbox2d.common.Vec2;
        constructor(lowerVertex?: any, upperVertex?: any);
        /**
         * Sets this object from the given object
         * @param {org.jbox2d.collision.AABB} aabb the object to copy from
         * @return {org.jbox2d.collision.AABB}
         */
        set(aabb: AABB): AABB;
        /**
         * Verify that the bounds are sorted
         * @return {boolean}
         */
        isValid(): boolean;
        /**
         * Check if AABBs overlap. djm optimized
         * @param {org.jbox2d.collision.AABB} box
         * @return {boolean}
         */
        testOverlap(box: AABB): boolean;
        /**
         *
         * @return {string}
         */
        toString(): string;
    }
}
declare namespace org.jbox2d.collision {
    /**
     * Used in BroadPhase
     * @param {org.jbox2d.collision.Bound} b
     * @class
     */
    class Bound {
        value: number;
        proxyId: number;
        stabbingCount: number;
        constructor(b?: any);
        set(b: Bound): void;
        isLower(): boolean;
        isUpper(): boolean;
        /**
         * swaps the values of this bound with the other bound
         * @param {org.jbox2d.collision.Bound} other bound to swap values with
         */
        swap(other: Bound): void;
        /**
         *
         * @return {string}
         */
        toString(): string;
    }
}
declare namespace org.jbox2d.collision {
    class BoundValues {
        lowerValues: number[];
        upperValues: number[];
        constructor();
    }
}
declare namespace org.jbox2d.collision {
    /**
     * Used in pair manager.
     * @class
     */
    class BufferedPair {
        proxyId1: number;
        proxyId2: number;
        equals(other: BufferedPair): boolean;
        minor(other: BufferedPair): boolean;
        compareTo(p: BufferedPair): number;
        constructor();
    }
}
declare namespace org.jbox2d.collision {
    /**
     * A few static final variables that don't fit anywhere else (globals in C++ code).
     * @class
     */
    class Collision {
        static NULL_FEATURE: number;
        static NULL_FEATURE_$LI$(): number;
    }
}
declare namespace org.jbox2d.collision {
    /**
     * Contact ids to facilitate warm starting.
     * @param {org.jbox2d.collision.ContactID} c
     * @class
     */
    class ContactID {
        /**
         * Inactive in Java port (used for quick compares in C++ as part of a union)
         */
        key: number;
        /**
         * The features that intersect to form the contact point
         */
        features: ContactID.Features;
        isEqual(cid: ContactID): boolean;
        /**
         * zeros out the data
         */
        zero(): void;
        constructor(c?: any);
        set(c: ContactID): void;
    }
    namespace ContactID {
        /**
         * The features that intersect to form the contact point
         * @param {org.jbox2d.collision.ContactID.Features} f
         * @class
         */
        class Features {
            __parent: any;
            /**
             * The edge that defines the outward contact normal.
             */
            referenceEdge: number;
            /**
             * The edge most anti-parallel to the reference edge.
             */
            incidentEdge: number;
            /**
             * The vertex (0 or 1) on the incident edge that was clipped.
             */
            incidentVertex: number;
            /**
             * A value of 1 indicates that the reference edge is on shape2.
             */
            flip: number;
            constructor(__parent: any, f?: any);
            set(f: ContactID.Features): void;
            isEqual(f: ContactID.Features): boolean;
            /**
             *
             * @return {string}
             */
            toString(): string;
        }
    }
}
declare namespace org.jbox2d.collision {
    /**
     * Implements the GJK algorithm for computing distance between shapes.
     * @class
     */
    class Distance {
        g_GJK_Iterations: number;
        p1s: org.jbox2d.common.Vec2[];
        p2s: org.jbox2d.common.Vec2[];
        points: org.jbox2d.common.Vec2[];
        v: org.jbox2d.common.Vec2;
        vNeg: org.jbox2d.common.Vec2;
        w: org.jbox2d.common.Vec2;
        w1: org.jbox2d.common.Vec2;
        w2: org.jbox2d.common.Vec2;
        constructor();
        p2r: org.jbox2d.common.Vec2;
        p2d: org.jbox2d.common.Vec2;
        ProcessTwo(x1: org.jbox2d.common.Vec2, x2: org.jbox2d.common.Vec2, p1s: org.jbox2d.common.Vec2[], p2s: org.jbox2d.common.Vec2[], points: org.jbox2d.common.Vec2[]): number;
        ProcessThree(x1: org.jbox2d.common.Vec2, x2: org.jbox2d.common.Vec2, p1s: org.jbox2d.common.Vec2[], p2s: org.jbox2d.common.Vec2[], points: org.jbox2d.common.Vec2[]): number;
        InPoints(w: org.jbox2d.common.Vec2, points: org.jbox2d.common.Vec2[], pointCount: number): boolean;
        /**
         * Distance between any two objects that implement SupportsGeneric Note that
         * x1 and x2 are passed so that they may store results - they must be
         * instantiated before being passed, and the contents will be lost.
         *
         * @param {org.jbox2d.common.Vec2} x1
         * Set to closest point on shape1 (result parameter)
         * @param {org.jbox2d.common.Vec2} x2
         * Set to closest point on shape2 (result parameter)
         * @param {*} shape1
         * Shape to test
         * @param {org.jbox2d.common.XForm} xf1
         * Transform of shape1
         * @param {*} shape2
         * Shape to test
         * @param {org.jbox2d.common.XForm} xf2
         * Transform of shape2
         * @return {number} the distance
         */
        DistanceGeneric(x1: org.jbox2d.common.Vec2, x2: org.jbox2d.common.Vec2, shape1: org.jbox2d.collision.SupportsGenericDistance, xf1: org.jbox2d.common.XForm, shape2: org.jbox2d.collision.SupportsGenericDistance, xf2: org.jbox2d.common.XForm): number;
        distCCp1: org.jbox2d.common.Vec2;
        distCCp2: org.jbox2d.common.Vec2;
        distCCd: org.jbox2d.common.Vec2;
        /**
         * distance between two circle shapes
         *
         * @param {org.jbox2d.common.Vec2} x1
         * Closest point on shape1 is put here (result parameter)
         * @param {org.jbox2d.common.Vec2} x2
         * Closest point on shape2 is put here (result parameter)
         * @param {org.jbox2d.collision.shapes.CircleShape} circle1
         * @param {org.jbox2d.common.XForm} xf1
         * Transform of first shape
         * @param {org.jbox2d.collision.shapes.CircleShape} circle2
         * @param {org.jbox2d.common.XForm} xf2
         * Transform of second shape
         * @return {number} the distance
         */
        DistanceCC(x1: org.jbox2d.common.Vec2, x2: org.jbox2d.common.Vec2, circle1: org.jbox2d.collision.shapes.CircleShape, xf1: org.jbox2d.common.XForm, circle2: org.jbox2d.collision.shapes.CircleShape, xf2: org.jbox2d.common.XForm): number;
        cWorld: org.jbox2d.common.Vec2;
        ECcLocal: org.jbox2d.common.Vec2;
        ECvWorld: org.jbox2d.common.Vec2;
        ECd: org.jbox2d.common.Vec2;
        ECtemp: org.jbox2d.common.Vec2;
        /**
         * Distance bewteen an edge and a circle
         *
         * @param {org.jbox2d.common.Vec2} x1
         * Closest point on shape1 is put here (result parameter)
         * @param {org.jbox2d.common.Vec2} x2
         * Closest point on shape2 is put here (result parameter)
         * @param {org.jbox2d.collision.shapes.EdgeShape} edge
         * @param {org.jbox2d.common.XForm} xf1
         * xform of edge
         * @param {org.jbox2d.collision.shapes.CircleShape} circle
         * @param {org.jbox2d.common.XForm} xf2
         * xform of circle
         * @return {number} the distance
         */
        DistanceEdgeCircle(x1: org.jbox2d.common.Vec2, x2: org.jbox2d.common.Vec2, edge: org.jbox2d.collision.shapes.EdgeShape, xf1: org.jbox2d.common.XForm, circle: org.jbox2d.collision.shapes.CircleShape, xf2: org.jbox2d.common.XForm): number;
        point: org.jbox2d.collision.Point;
        /**
         * Distance between a polygon and a circle
         *
         * @param {org.jbox2d.common.Vec2} x1
         * Closest point on shape1 is put here (result parameter)
         * @param {org.jbox2d.common.Vec2} x2
         * Closest point on shape2 is put here (result parameter)
         * @param {org.jbox2d.collision.shapes.PolygonShape} polygon
         * @param {org.jbox2d.common.XForm} xf1
         * xform of polygon
         * @param {org.jbox2d.collision.shapes.CircleShape} circle
         * @param {org.jbox2d.common.XForm} xf2
         * xform of circle
         * @return {number} the distance
         */
        DistancePC(x1: org.jbox2d.common.Vec2, x2: org.jbox2d.common.Vec2, polygon: org.jbox2d.collision.shapes.PolygonShape, xf1: org.jbox2d.common.XForm, circle: org.jbox2d.collision.shapes.CircleShape, xf2: org.jbox2d.common.XForm): number;
        /**
         * Distance between a polygon and a point
         *
         * @param {org.jbox2d.common.Vec2} x1
         * Closest point on shape1 is put here (result parameter)
         * @param {org.jbox2d.common.Vec2} x2
         * Closest point on shape2 is put here (result parameter)
         * @param {org.jbox2d.collision.shapes.PolygonShape} polygon
         * @param {org.jbox2d.common.XForm} xf1
         * xform of polygon
         * @param {org.jbox2d.collision.shapes.PointShape} pt
         * @param {org.jbox2d.common.XForm} xf2
         * xform of point
         * @return {number} the distance
         */
        DistancePolygonPoint(x1: org.jbox2d.common.Vec2, x2: org.jbox2d.common.Vec2, polygon: org.jbox2d.collision.shapes.PolygonShape, xf1: org.jbox2d.common.XForm, pt: org.jbox2d.collision.shapes.PointShape, xf2: org.jbox2d.common.XForm): number;
        CPp1: org.jbox2d.common.Vec2;
        CPp2: org.jbox2d.common.Vec2;
        CPd: org.jbox2d.common.Vec2;
        /**
         * Distance between a circle and a point
         *
         * @param {org.jbox2d.common.Vec2} x1
         * Closest point on shape1 is put here (result parameter)
         * @param {org.jbox2d.common.Vec2} x2
         * Closest point on shape2 is put here (result parameter)
         * @param {org.jbox2d.collision.shapes.CircleShape} circle1
         * @param {org.jbox2d.common.XForm} xf1
         * xform of circle
         * @param {org.jbox2d.collision.shapes.PointShape} pt2
         * @param {org.jbox2d.common.XForm} xf2
         * xform of point
         * @return {number} the distance
         */
        DistanceCirclePoint(x1: org.jbox2d.common.Vec2, x2: org.jbox2d.common.Vec2, circle1: org.jbox2d.collision.shapes.CircleShape, xf1: org.jbox2d.common.XForm, pt2: org.jbox2d.collision.shapes.PointShape, xf2: org.jbox2d.common.XForm): number;
        /**
         * Find the closest distance between shapes shape1 and shape2, and load the
         * closest points into x1 and x2. Note that x1 and x2 are passed so that
         * they may store results - they must be instantiated before being passed,
         * and the contents will be lost.
         *
         * @param {org.jbox2d.common.Vec2} x1
         * Closest point on shape1 is put here (result parameter)
         * @param {org.jbox2d.common.Vec2} x2
         * Closest point on shape2 is put here (result parameter)
         * @param {org.jbox2d.collision.shapes.Shape} shape1
         * First shape to test
         * @param {org.jbox2d.common.XForm} xf1
         * Transform of first shape
         * @param {org.jbox2d.collision.shapes.Shape} shape2
         * Second shape to test
         * @param {org.jbox2d.common.XForm} xf2
         * Transform of second shape
         * @return {number} the distance
         */
        distance(x1: org.jbox2d.common.Vec2, x2: org.jbox2d.common.Vec2, shape1: org.jbox2d.collision.shapes.Shape, xf1: org.jbox2d.common.XForm, shape2: org.jbox2d.collision.shapes.Shape, xf2: org.jbox2d.common.XForm): number;
    }
    class Point implements org.jbox2d.collision.SupportsGenericDistance {
        p: org.jbox2d.common.Vec2;
        constructor(_p?: any);
        support(dest: org.jbox2d.common.Vec2, xf: org.jbox2d.common.XForm, v: org.jbox2d.common.Vec2): void;
        getFirstVertexToOut(xf: org.jbox2d.common.XForm, out: org.jbox2d.common.Vec2): void;
    }
}
declare namespace org.jbox2d.collision {
    /**
     * This holds contact filtering data.
     * @class
     */
    class FilterData {
        /**
         * The collision category bits. Normally you would just set one bit.
         */
        categoryBits: number;
        /**
         * The collision mask bits. This states the categories that this
         * shape would accept for collision.
         */
        maskBits: number;
        /**
         * Collision groups allow a certain group of objects to never collide (negative)
         * or always collide (positive). Zero means no collision group. Non-zero group
         * filtering always wins against the mask bits.
         */
        groupIndex: number;
        set(fd: FilterData): void;
        constructor();
    }
}
declare namespace org.jbox2d.collision {
    /**
     * Creates this manifold as a copy of the other
     * @param {org.jbox2d.collision.Manifold} other
     * @class
     */
    class Manifold {
        /**
         * The points of contact.
         */
        points: org.jbox2d.collision.ManifoldPoint[];
        /**
         * The shared unit normal vector.
         */
        normal: org.jbox2d.common.Vec2;
        /**
         * The number of manifold points.
         */
        pointCount: number;
        constructor(other?: any);
        /**
         * copies this manifold from the given one
         * @param {org.jbox2d.collision.Manifold} cp manifold to copy from
         * @return {org.jbox2d.collision.Manifold}
         */
        set(cp: Manifold): Manifold;
    }
}
declare namespace org.jbox2d.collision {
    /**
     * Creates a manifold point as a copy of the given point
     * @param {org.jbox2d.collision.ManifoldPoint} cp point to copy from
     * @class
     */
    class ManifoldPoint {
        /**
         * Local position of the contact point in body1
         */
        localPoint1: org.jbox2d.common.Vec2;
        /**
         * Local position of the contact point in body2
         */
        localPoint2: org.jbox2d.common.Vec2;
        /**
         * The separation of the shapes along the normal vector
         */
        separation: number;
        /**
         * The non-penetration force
         */
        normalImpulse: number;
        /**
         * The friction force
         */
        tangentImpulse: number;
        /**
         * Uniquely identifies a contact point between two shapes
         */
        id: org.jbox2d.collision.ContactID;
        constructor(cp?: any);
        /**
         * Sets this manifold point form the given one
         * @param {org.jbox2d.collision.ManifoldPoint} cp the point to copy from
         */
        set(cp: ManifoldPoint): void;
    }
}
declare namespace org.jbox2d.collision {
    /**
     * Copies from the given mass data
     * @param {org.jbox2d.collision.MassData} md mass data to copy from
     * @class
     */
    class MassData {
        /**
         * The mass of the shape, usually in kilograms.
         */
        mass: number;
        /**
         * The position of the shape's centroid relative to the shape's origin.
         */
        center: org.jbox2d.common.Vec2;
        /**
         * The rotational inertia of the shape.
         */
        I: number;
        set(argOther: MassData): MassData;
        constructor(md?: any);
        /**
         * Return a copy of this object.
         * @return {org.jbox2d.collision.MassData}
         */
        clone(): MassData;
    }
}
declare namespace org.jbox2d.collision {
    /**
     * creates OBB with the given data.  The arguments are cloned
     * before being placed in the object
     * @param {org.jbox2d.common.Mat22} _R
     * @param {org.jbox2d.common.Vec2} _center
     * @param {org.jbox2d.common.Vec2} _extents
     * @class
     */
    class OBB {
        /**
         * The rotation matrix.
         */
        R: org.jbox2d.common.Mat22;
        /**
         * The local centroid.
         */
        center: org.jbox2d.common.Vec2;
        /**
         * The half-widths.
         */
        extents: org.jbox2d.common.Vec2;
        constructor(_R?: any, _center?: any, _extents?: any);
        /**
         * @return {org.jbox2d.collision.OBB} a copy
         */
        clone(): OBB;
    }
}
declare namespace org.jbox2d.collision {
    /**
     * Copy constructor
     * @param {org.jbox2d.collision.Pair} other
     * @class
     */
    class Pair {
        static PAIR_BUFFERED: number;
        static PAIR_REMOVED: number;
        static PAIR_FINAL: number;
        userData: any;
        proxyId1: number;
        proxyId2: number;
        status: number;
        next: number;
        constructor(other?: any);
        setBuffered(): void;
        clearBuffered(): void;
        isBuffered(): boolean;
        clearRemoved(): void;
        setRemoved(): void;
        isRemoved(): boolean;
        setFinal(): void;
        isFinal(): boolean;
        compareTo(p: Pair): number;
    }
}
declare namespace org.jbox2d.collision {
    interface PairCallback {
        /**
         * This should return the new pair user data. It is okay if the
         * user data is null.
         * @param {*} proxyUserData1
         * @param {*} proxyUserData2
         * @return {*}
         */
        pairAdded(proxyUserData1: any, proxyUserData2: any): any;
        /**
         * This should free the pair's user data. In extreme circumstances, it is
         * possible this will be called with null pairUserData because the pair never
         * existed.
         * @param {*} proxyUserData1
         * @param {*} proxyUserData2
         * @param {*} pairUserData
         */
        pairRemoved(proxyUserData1: any, proxyUserData2: any, pairUserData: any): any;
    }
}
declare namespace org.jbox2d.collision {
    class Proxy {
        lowerBounds: number[];
        upperBounds: number[];
        overlapCount: number;
        timeStamp: number;
        categoryBits: number;
        maskBits: number;
        groupIndex: number;
        userData: any;
        constructor();
        getNext(): number;
        setNext(next: number): void;
        isValid(): boolean;
    }
}
declare namespace org.jbox2d.collision {
    enum SegmentCollide {
        STARTS_INSIDE_COLLIDE = 0,
        MISS_COLLIDE = 1,
        HIT_COLLIDE = 2,
    }
}
declare namespace org.jbox2d.collision.shapes {
    /**
     * Circle/circle and circle/polygon overlap solver -
     * for internal use only.
     * @class
     */
    class CollideCircle {
        colCCP1: org.jbox2d.common.Vec2;
        colCCP2: org.jbox2d.common.Vec2;
        colCCD: org.jbox2d.common.Vec2;
        colCCP: org.jbox2d.common.Vec2;
        /**
         * puts collision information of the two circles in the manifold
         * @param {org.jbox2d.collision.Manifold} manifold
         * @param {org.jbox2d.collision.shapes.CircleShape} circle1
         * @param {org.jbox2d.common.XForm} xf1
         * @param {org.jbox2d.collision.shapes.CircleShape} circle2
         * @param {org.jbox2d.common.XForm} xf2
         */
        collideCircles(manifold: org.jbox2d.collision.Manifold, circle1: org.jbox2d.collision.shapes.CircleShape, xf1: org.jbox2d.common.XForm, circle2: org.jbox2d.collision.shapes.CircleShape, xf2: org.jbox2d.common.XForm): void;
        colPCP1: org.jbox2d.common.Vec2;
        colPCP2: org.jbox2d.common.Vec2;
        colPCD: org.jbox2d.common.Vec2;
        colPCP: org.jbox2d.common.Vec2;
        /**
         * Puts collision information in the manifold about a collision between a point and a circle
         * @param {org.jbox2d.collision.Manifold} manifold
         * @param {org.jbox2d.collision.shapes.PointShape} point1
         * @param {org.jbox2d.common.XForm} xf1
         * @param {org.jbox2d.collision.shapes.CircleShape} circle2
         * @param {org.jbox2d.common.XForm} xf2
         */
        collidePointAndCircle(manifold: org.jbox2d.collision.Manifold, point1: org.jbox2d.collision.shapes.PointShape, xf1: org.jbox2d.common.XForm, circle2: org.jbox2d.collision.shapes.CircleShape, xf2: org.jbox2d.common.XForm): void;
        /**
         * puts collision information about the collision of a polygon and a circle
         * @param {org.jbox2d.collision.Manifold} manifold
         * @param {org.jbox2d.collision.shapes.PolygonShape} polygon
         * @param {org.jbox2d.common.XForm} xf1
         * @param {org.jbox2d.collision.shapes.CircleShape} circle
         * @param {org.jbox2d.common.XForm} xf2
         */
        collidePolygonAndCircle(manifold: org.jbox2d.collision.Manifold, polygon: org.jbox2d.collision.shapes.PolygonShape, xf1: org.jbox2d.common.XForm, circle: org.jbox2d.collision.shapes.CircleShape, xf2: org.jbox2d.common.XForm): void;
        ECd: org.jbox2d.common.Vec2;
        ECc: org.jbox2d.common.Vec2;
        ECcLocal: org.jbox2d.common.Vec2;
        /**
         * puts collision information into the manifold from a circle and edge collision
         * @param {org.jbox2d.collision.Manifold} manifold
         * @param {org.jbox2d.collision.shapes.EdgeShape} edge
         * @param {org.jbox2d.common.XForm} xf1
         * @param {org.jbox2d.collision.shapes.CircleShape} circle
         * @param {org.jbox2d.common.XForm} xf2
         */
        collideEdgeAndCircle(manifold: org.jbox2d.collision.Manifold, edge: org.jbox2d.collision.shapes.EdgeShape, xf1: org.jbox2d.common.XForm, circle: org.jbox2d.collision.shapes.CircleShape, xf2: org.jbox2d.common.XForm): void;
    }
}
declare namespace org.jbox2d.collision.shapes {
    /**
     * Polygon overlap solver - for internal use.
     * @class
     */
    class CollidePoly {
        clipSegmentToLine(vOut: CollidePoly.ClipVertex[], vIn: CollidePoly.ClipVertex[], normal: org.jbox2d.common.Vec2, offset: number): number;
        normal1World: org.jbox2d.common.Vec2;
        edgeSeparation(poly1: org.jbox2d.collision.shapes.PolygonShape, xf1: org.jbox2d.common.XForm, edge1: number, poly2: org.jbox2d.collision.shapes.PolygonShape, xf2: org.jbox2d.common.XForm): number;
        dLocal1: org.jbox2d.common.Vec2;
        /**
         * Find the max separation between poly1 and poly2 using face normals
         * from poly1.
         * @param {org.jbox2d.collision.shapes.PolygonShape} poly1
         * @param {org.jbox2d.common.XForm} xf1
         * @param {org.jbox2d.collision.shapes.PolygonShape} poly2
         * @param {org.jbox2d.common.XForm} xf2
         * @return
         * @return {org.jbox2d.collision.shapes.MaxSeparation}
         */
        findMaxSeparation(poly1: org.jbox2d.collision.shapes.PolygonShape, xf1: org.jbox2d.common.XForm, poly2: org.jbox2d.collision.shapes.PolygonShape, xf2: org.jbox2d.common.XForm): org.jbox2d.collision.shapes.MaxSeparation;
        mulTemp: org.jbox2d.common.Vec2;
        normal1: org.jbox2d.common.Vec2;
        findIncidentEdge(c: CollidePoly.ClipVertex[], poly1: org.jbox2d.collision.shapes.PolygonShape, xf1: org.jbox2d.common.XForm, edge1: number, poly2: org.jbox2d.collision.shapes.PolygonShape, xf2: org.jbox2d.common.XForm): void;
        collidePolygons(manif: org.jbox2d.collision.Manifold, polyA: org.jbox2d.collision.shapes.PolygonShape, xfA: org.jbox2d.common.XForm, polyB: org.jbox2d.collision.shapes.PolygonShape, xfB: org.jbox2d.common.XForm): void;
        colPPc: org.jbox2d.common.Vec2;
        colPPcLocal: org.jbox2d.common.Vec2;
        colPPsub: org.jbox2d.common.Vec2;
        colPPe: org.jbox2d.common.Vec2;
        colPPp: org.jbox2d.common.Vec2;
        colPPd: org.jbox2d.common.Vec2;
        /**
         * puts collision information into the manifold about the collision between a polygon and a point
         * @param {org.jbox2d.collision.Manifold} manifold
         * @param {org.jbox2d.collision.shapes.PolygonShape} polygon
         * @param {org.jbox2d.common.XForm} xf1
         * @param {org.jbox2d.collision.shapes.PointShape} point
         * @param {org.jbox2d.common.XForm} xf2
         */
        collidePolygonAndPoint(manifold: org.jbox2d.collision.Manifold, polygon: org.jbox2d.collision.shapes.PolygonShape, xf1: org.jbox2d.common.XForm, point: org.jbox2d.collision.shapes.PointShape, xf2: org.jbox2d.common.XForm): void;
        PEv1: org.jbox2d.common.Vec2;
        PEv2: org.jbox2d.common.Vec2;
        PEn: org.jbox2d.common.Vec2;
        PEv1Local: org.jbox2d.common.Vec2;
        PEv2Local: org.jbox2d.common.Vec2;
        PEnLocal: org.jbox2d.common.Vec2;
        temp: org.jbox2d.common.Vec2;
        temp2: org.jbox2d.common.Vec2;
        /**
         * puts collision information into the manifold about a collision between
         * a polygon and an edge
         * @param {org.jbox2d.collision.Manifold} manifold
         * @param {org.jbox2d.collision.shapes.PolygonShape} polygon
         * @param {org.jbox2d.common.XForm} xf1
         * @param {org.jbox2d.collision.shapes.EdgeShape} edge
         * @param {org.jbox2d.common.XForm} xf2
         */
        collidePolyAndEdge(manifold: org.jbox2d.collision.Manifold, polygon: org.jbox2d.collision.shapes.PolygonShape, xf1: org.jbox2d.common.XForm, edge: org.jbox2d.collision.shapes.EdgeShape, xf2: org.jbox2d.common.XForm): void;
        sideNormal: org.jbox2d.common.Vec2;
        frontNormal: org.jbox2d.common.Vec2;
        p_xf1: org.jbox2d.common.XForm;
        p_xf2: org.jbox2d.common.XForm;
    }
    namespace CollidePoly {
        class ClipVertex {
            v: org.jbox2d.common.Vec2;
            id: org.jbox2d.collision.ContactID;
            constructor();
        }
    }
    /**
     * Holder class used internally in CollidePoly
     * @class
     */
    class MaxSeparation {
        bestFaceIndex: number;
        bestSeparation: number;
        constructor();
    }
}
declare namespace org.jbox2d.collision.shapes {
    /**
     * Superclass for shape definitions.
     * You should usually use CircleDef or
     * PolygonDef to define concrete shapes.
     * @class
     */
    class ShapeDef {
        /**
         * Holds the shape type for down-casting.
         */
        type: org.jbox2d.collision.shapes.ShapeType;
        /**
         * Use this to store application specify shape data.
         */
        userData: any;
        /**
         * The shape's friction coefficient, usually in the range [0,1].
         */
        friction: number;
        /**
         * The shape's restitution (elasticity) usually in the range [0,1].
         */
        restitution: number;
        /**
         * The shape's density, usually in kg/m^2.
         */
        density: number;
        /**
         * Contact filtering data.
         */
        filter: org.jbox2d.collision.FilterData;
        /**
         * A sensor shape collects contact information but never generates a collision
         * response.
         */
        isSensor: boolean;
        constructor();
    }
}
declare namespace org.jbox2d.collision.shapes {
    enum ShapeType {
        UNKNOWN_SHAPE = 0,
        CIRCLE_SHAPE = 1,
        POLYGON_SHAPE = 2,
        POINT_SHAPE = 3,
        EDGE_SHAPE = 4,
        SHAPE_TYPE_COUNT = 5,
    }
}
declare namespace org.jbox2d.collision {
    /**
     * @author eric
     * @class
     */
    interface SortKeyFunc {
        apply(shape: any): number;
    }
}
declare namespace org.jbox2d.collision {
    /**
     * A shape that implements this interface can be used in distance calculations
     * for continuous collision detection.  This does not remove the necessity of
     * specialized penetration calculations when CCD is not in effect, however.
     * @class
     */
    interface SupportsGenericDistance {
        support(dest: org.jbox2d.common.Vec2, xf: org.jbox2d.common.XForm, v: org.jbox2d.common.Vec2): any;
        getFirstVertexToOut(xf: org.jbox2d.common.XForm, out: org.jbox2d.common.Vec2): any;
    }
}
declare namespace org.jbox2d.common {
    /**
     * Similar to javax.vecmath.Color3f holder
     * @author ewjordan
     * @param {number} r
     * @param {number} g
     * @param {number} b
     * @class
     */
    class Color3f {
        static WHITE: Color3f;
        static WHITE_$LI$(): Color3f;
        static BLACK: Color3f;
        static BLACK_$LI$(): Color3f;
        static BLUE: Color3f;
        static BLUE_$LI$(): Color3f;
        static GREEN: Color3f;
        static GREEN_$LI$(): Color3f;
        static RED: Color3f;
        static RED_$LI$(): Color3f;
        x: number;
        y: number;
        z: number;
        constructor(r: number, g: number, b: number);
    }
}
declare namespace org.jbox2d.common {
    /**
     * This is the viewport transform used from drawing.
     * Use yFlip if you are drawing from the top-left corner.
     * @author daniel
     * @class
     */
    interface IViewportTransform {
        /**
         * @return {boolean} if the transform flips the y axis
         */
        isYFlip(): boolean;
        /**
         * @param {boolean} yFlip if we flip the y axis when transforming
         */
        setYFlip(yFlip: boolean): any;
        /**
         * This is the half-width and half-height.
         * This should be the actual half-width and
         * half-height, not anything transformed or scaled.
         * Not a copy.
         * @return
         * @return {org.jbox2d.common.Vec2}
         */
        getExtents(): org.jbox2d.common.Vec2;
        /**
         * This sets the half-width and half-height of the
         * viewport. This should be the actual half-width and
         * half-height, not anything transformed or scaled.
         * @param {number} argHalfWidth
         * @param {number} argHalfHeight
         */
        setExtents(argHalfWidth?: any, argHalfHeight?: any): any;
        /**
         * center of the viewport.  Not a copy.
         * @return
         * @return {org.jbox2d.common.Vec2}
         */
        getCenter(): org.jbox2d.common.Vec2;
        /**
         * sets the center of the viewport.
         * @param {number} x
         * @param {number} y
         */
        setCenter(x?: any, y?: any): any;
        /**
         * Sets the transform's center to the given x and y coordinates,
         * and using the given scale.
         * @param {number} x
         * @param {number} y
         * @param {number} scale
         */
        setCamera(x: number, y: number, scale: number): any;
        /**
         * Transforms the given directional vector by the
         * viewport transform (not positional)
         * @param argVec
         * @param argOut
         * @param {org.jbox2d.common.Vec2} argWorld
         * @param {org.jbox2d.common.Vec2} argScreen
         */
        vectorTransform(argWorld: org.jbox2d.common.Vec2, argScreen: org.jbox2d.common.Vec2): any;
        /**
         * Transforms the given directional screen vector back to
         * the world direction.
         * @param argVec
         * @param argOut
         * @param {org.jbox2d.common.Vec2} argScreen
         * @param {org.jbox2d.common.Vec2} argWorld
         */
        vectorInverseTransform(argScreen: org.jbox2d.common.Vec2, argWorld: org.jbox2d.common.Vec2): any;
        /**
         * takes the world coordinate (argWorld) puts the corresponding
         * screen coordinate in argScreen.  It should be safe to give the
         * same object as both parameters.
         * @param {org.jbox2d.common.Vec2} argWorld
         * @param {org.jbox2d.common.Vec2} argScreen
         */
        getWorldToScreen(argWorld: org.jbox2d.common.Vec2, argScreen: org.jbox2d.common.Vec2): any;
        /**
         * takes the screen coordinates (argScreen) and puts the
         * corresponding world coordinates in argWorld. It should be safe
         * to give the same object as both parameters.
         * @param {org.jbox2d.common.Vec2} argScreen
         * @param {org.jbox2d.common.Vec2} argWorld
         */
        getScreenToWorld(argScreen: org.jbox2d.common.Vec2, argWorld: org.jbox2d.common.Vec2): any;
    }
}
declare namespace org.jbox2d.common {
    /**
     * Create a matrix from four floats.
     * @param {number} col1x
     * @param {number} col2x
     * @param {number} col1y
     * @param {number} col2y
     * @class
     * @author ewjordan
     */
    class Mat22 {
        col1: org.jbox2d.common.Vec2;
        col2: org.jbox2d.common.Vec2;
        /**
         * Convert the matrix to printable format.
         * @return {string}
         */
        toString(): string;
        constructor(col1x?: any, col2x?: any, col1y?: any, col2y?: any);
        set$org_jbox2d_common_Mat22(m: Mat22): Mat22;
        set$float$float$float$float(col1x: number, col2x: number, col1y: number, col2y: number): Mat22;
        set(col1x?: any, col2x?: any, col1y?: any, col2y?: any): any;
        /**
         * Return a clone of this matrix.
         * djm fixed double allocation
         * @return {org.jbox2d.common.Mat22}
         */
        clone(): Mat22;
        set$float(angle: number): void;
        /**
         * Set as the identity matrix.
         */
        setIdentity(): void;
        /**
         * Set as the zero matrix.
         */
        setZero(): void;
        set$org_jbox2d_common_Vec2$org_jbox2d_common_Vec2(c1: org.jbox2d.common.Vec2, c2: org.jbox2d.common.Vec2): void;
        /**
         * Returns the inverted Mat22 - does NOT invert the matrix locally!
         * @return {org.jbox2d.common.Mat22}
         */
        invert(): Mat22;
        invertLocal(): Mat22;
        invertToOut(out: Mat22): void;
        /**
         * Return the matrix composed of the absolute values of all elements.
         * djm: fixed double allocation
         * @return {org.jbox2d.common.Mat22} Absolute value matrix
         */
        abs(): Mat22;
        absLocal(): void;
        /**
         * Return the matrix composed of the absolute values of all elements.
         * @return {org.jbox2d.common.Mat22} Absolute value matrix
         * @param {org.jbox2d.common.Mat22} R
         */
        static abs(R: Mat22): Mat22;
        static absToOut(R: Mat22, out: Mat22): void;
        mul$org_jbox2d_common_Vec2(v: org.jbox2d.common.Vec2): org.jbox2d.common.Vec2;
        /**
         * Multiply a vector by this matrix.
         * @param {org.jbox2d.common.Vec2} v Vector to multiply by matrix.
         * @return {org.jbox2d.common.Vec2} Resulting vector
         */
        mul(v?: any): any;
        mulToOut$org_jbox2d_common_Vec2$org_jbox2d_common_Vec2(v: org.jbox2d.common.Vec2, out: org.jbox2d.common.Vec2): void;
        mulToOut(v?: any, out?: any): any;
        mul$org_jbox2d_common_Mat22(R: Mat22): Mat22;
        mulLocal(R: Mat22): Mat22;
        mulToOut$org_jbox2d_common_Mat22$org_jbox2d_common_Mat22(R: Mat22, out: Mat22): void;
        mulTrans$org_jbox2d_common_Mat22(B: Mat22): Mat22;
        /**
         * Multiply another matrix by the transpose of this one (transpose of this one on left).
         * djm: optimized
         * @param {org.jbox2d.common.Mat22} B
         * @return
         * @return {org.jbox2d.common.Mat22}
         */
        mulTrans(B?: any): any;
        mulTransLocal(B: Mat22): Mat22;
        mulTransToOut$org_jbox2d_common_Mat22$org_jbox2d_common_Mat22(B: Mat22, out: Mat22): void;
        mulTransToOut(B?: any, out?: any): any;
        mulTrans$org_jbox2d_common_Vec2(v: org.jbox2d.common.Vec2): org.jbox2d.common.Vec2;
        mulTransToOut$org_jbox2d_common_Vec2$org_jbox2d_common_Vec2(v: org.jbox2d.common.Vec2, out: org.jbox2d.common.Vec2): void;
        /**
         * Add this matrix to B, return the result.
         * @param {org.jbox2d.common.Mat22} B
         * @return
         * @return {org.jbox2d.common.Mat22}
         */
        add(B: Mat22): Mat22;
        /**
         * Add B to this matrix locally.
         * @param {org.jbox2d.common.Mat22} B
         * @return
         * @return {org.jbox2d.common.Mat22}
         */
        addLocal(B: Mat22): Mat22;
        /**
         * Solve A * x = b where A = this matrix.
         * @return {org.jbox2d.common.Vec2} The vector x that solves the above equation.
         * @param {org.jbox2d.common.Vec2} b
         */
        solve(b: org.jbox2d.common.Vec2): org.jbox2d.common.Vec2;
        solveToOut(b: org.jbox2d.common.Vec2, out: org.jbox2d.common.Vec2): void;
        static mul$org_jbox2d_common_Mat22$org_jbox2d_common_Vec2(R: Mat22, v: org.jbox2d.common.Vec2): org.jbox2d.common.Vec2;
        static mul(R?: any, v?: any): any;
        static mulToOut$org_jbox2d_common_Mat22$org_jbox2d_common_Vec2$org_jbox2d_common_Vec2(R: Mat22, v: org.jbox2d.common.Vec2, out: org.jbox2d.common.Vec2): void;
        static mulToOut(R?: any, v?: any, out?: any): any;
        static mul$org_jbox2d_common_Mat22$org_jbox2d_common_Mat22(A: Mat22, B: Mat22): Mat22;
        static mulToOut$org_jbox2d_common_Mat22$org_jbox2d_common_Mat22$org_jbox2d_common_Mat22(A: Mat22, B: Mat22, out: Mat22): void;
        static mulTrans$org_jbox2d_common_Mat22$org_jbox2d_common_Vec2(R: Mat22, v: org.jbox2d.common.Vec2): org.jbox2d.common.Vec2;
        static mulTrans(R?: any, v?: any): any;
        static mulTransToOut$org_jbox2d_common_Mat22$org_jbox2d_common_Vec2$org_jbox2d_common_Vec2(R: Mat22, v: org.jbox2d.common.Vec2, out: org.jbox2d.common.Vec2): void;
        static mulTransToOut(R?: any, v?: any, out?: any): any;
        static mulTrans$org_jbox2d_common_Mat22$org_jbox2d_common_Mat22(A: Mat22, B: Mat22): Mat22;
        static mulTransToOut$org_jbox2d_common_Mat22$org_jbox2d_common_Mat22$org_jbox2d_common_Mat22(A: Mat22, B: Mat22, out: Mat22): void;
        static createRotationalTransform$float(angle: number): Mat22;
        static createRotationalTransform$float$org_jbox2d_common_Mat22(angle: number, out: Mat22): void;
        static createRotationalTransform(angle?: any, out?: any): any;
        static createScaleTransform$float(scale: number): Mat22;
        static createScaleTransform$float$float(scaleX: number, scaleY: number): Mat22;
        static createScaleTransform$float$org_jbox2d_common_Mat22(scale: number, out: Mat22): void;
        static createScaleTransform$float$float$org_jbox2d_common_Mat22(scaleX: number, scaleY: number, out: Mat22): void;
        static createScaleTransform(scaleX?: any, scaleY?: any, out?: any): any;
    }
}
declare namespace org.jbox2d.common {
    class Mat33 {
        col1: org.jbox2d.common.Vec3;
        col2: org.jbox2d.common.Vec3;
        col3: org.jbox2d.common.Vec3;
        constructor(argCol1?: any, argCol2?: any, argCol3?: any);
        setZero(): void;
        static mul(A: Mat33, v: org.jbox2d.common.Vec3): org.jbox2d.common.Vec3;
        static mulToOut(A: Mat33, v: org.jbox2d.common.Vec3, out: org.jbox2d.common.Vec3): void;
        temp: org.jbox2d.common.Vec3;
        solve(b: org.jbox2d.common.Vec3): org.jbox2d.common.Vec3;
        solveToOut(b: org.jbox2d.common.Vec3, out: org.jbox2d.common.Vec3): void;
    }
}
declare namespace org.jbox2d.common {
    class RaycastResult {
        lambda: number;
        normal: org.jbox2d.common.Vec2;
        set(argOther: RaycastResult): RaycastResult;
    }
}
declare namespace org.jbox2d.common {
    /**
     * Global tuning constants based on MKS units and various integer maximums (vertices per shape, pairs, etc.).
     * @class
     */
    class Settings {
        /**
         * A "close to zero" float epsilon value for use
         */
        static EPSILON: number;
        /**
         * Pi.
         */
        static pi: number;
        static pi_$LI$(): number;
        /**
         * needs to be final, or will slow down math methods
         */
        static FAST_MATH: boolean;
        static SINCOS_LUT_ENABLED: boolean;
        /**
         * smaller the precision, the larger the table.  If
         * a small table is used (eg, precision is .006 or greater),
         * make sure you set the table to lerp it's results.  Accuracy chart
         * is in the MathUtils source.  Or, run the tests
         * yourself in {@link org.jbox2d.testbed.mathtests.SinCosTest}.</br>
         * </br>
         * Good lerp precision values:
         * <ul><li>.0092</li>
         * <li>.008201</li>
         * <li>.005904</li>
         * <li>.005204</li>
         * <li>.004305</li>
         * <li>.002807</li>
         * <li>.001508</li>
         * <li>9.32500E-4</li>
         * <li>7.48000E-4</li>
         * <li>8.47000E-4</li>
         * <li>.0005095</li>
         * <li>.0001098</li>
         * <li>9.50499E-5</li>
         * <li>6.08500E-5</li>
         * <li>3.07000E-5</li>
         * <li>1.53999E-5</li></ul>
         *
         */
        static SINCOS_LUT_PRECISION: number;
        static SINCOS_LUT_LENGTH: number;
        static SINCOS_LUT_LENGTH_$LI$(): number;
        /**
         * Use if the table's precision is large (eg .006 or greater).
         * Although it is more expensive, it greatly increases
         * accuracy.  Look in the MathUtils source for some test results
         * on the accuracy and speed of lerp vs non lerp.  Or, run the tests
         * yourself in {@link org.jbox2d.testbed.mathtests.SinCosTest}.
         */
        static SINCOS_LUT_LERP: boolean;
        static lengthUnitsPerMeter: number;
        static massUnitsPerKilogram: number;
        static timeUnitsPerSecond: number;
        static maxManifoldPoints: number;
        static maxShapesPerBody: number;
        static maxPolygonVertices: number;
        /**
         * Must be a power of two.
         */
        static maxProxies: number;
        /**
         * Must be a power of two.
         */
        static maxPairs: number;
        static maxPairs_$LI$(): number;
        /**
         * A small length used as a collision and constraint tolerance. Usually it is
         * chosen to be numerically significant, but visually insignificant.
         */
        static linearSlop: number;
        static linearSlop_$LI$(): number;
        /**
         * A small angle used as a collision and constraint tolerance. Usually it is
         * chosen to be numerically significant, but visually insignificant.
         */
        static angularSlop: number;
        static angularSlop_$LI$(): number;
        /**
         * A velocity threshold for elastic collisions. Any collision with a relative linear
         * velocity below this threshold will be treated as inelastic.
         */
        static velocityThreshold: number;
        static velocityThreshold_$LI$(): number;
        /**
         * The maximum linear position correction used when solving constraints. This helps to
         * prevent overshoot.
         */
        static maxLinearCorrection: number;
        static maxLinearCorrection_$LI$(): number;
        /**
         * The maximum angular position correction used when solving constraints. This helps to
         * prevent overshoot.
         */
        static maxAngularCorrection: number;
        static maxAngularCorrection_$LI$(): number;
        /**
         * This scale factor controls how fast overlap is resolved. Ideally this would be 1 so
         * that overlap is removed in one time step. However using values close to 1 often lead
         * to overshoot.
         */
        static contactBaumgarte: number;
        /**
         * The time that a body must be still before it will go to sleep.
         */
        static timeToSleep: number;
        static timeToSleep_$LI$(): number;
        /**
         * A body cannot sleep if its linear velocity is above this tolerance.
         */
        static linearSleepTolerance: number;
        static linearSleepTolerance_$LI$(): number;
        /**
         * A body cannot sleep if its angular velocity is above this tolerance.
         */
        static angularSleepTolerance: number;
        static angularSleepTolerance_$LI$(): number;
        /**
         * Continuous collision detection (CCD) works with core, shrunken shapes. This is the
         * amount by which shapes are automatically shrunk to work with CCD. This must be
         * larger than b2_linearSlop.
         */
        static toiSlop: number;
        static toiSlop_$LI$(): number;
        /**
         * The maximum linear velocity of a body. This limit is very large and is used
         * to prevent numerical problems. You shouldn't need to adjust this.
         */
        static maxLinearVelocity: number;
        static maxLinearVelocitySquared: number;
        static maxLinearVelocitySquared_$LI$(): number;
        /**
         * The maximum angular velocity of a body. This limit is very large and is used
         * to prevent numerical problems. You shouldn't need to adjust this.
         */
        static maxAngularVelocity: number;
        static maxAngularVelocitySquared: number;
        static maxAngularVelocitySquared_$LI$(): number;
        /**
         * Maximum number of contacts to be handled to solve a TOI island.
         */
        static maxTOIContactsPerIsland: number;
        /**
         * Maximum number of joints to be handled to solve a TOI island.
         */
        static maxTOIJointsPerIsland: number;
    }
}
declare namespace org.jbox2d.common {
    /**
     * Primarily for internal use.
     * <BR><BR>
     * Describes the motion of a body/shape for TOI computation.
     * Shapes are defined with respect to the body origin, which may
     * not coincide with the center of mass. However, to support dynamics
     * we must interpolate the center of mass position.
     *
     * @class
     */
    class Sweep {
        /**
         * Local center of mass position
         */
        localCenter: org.jbox2d.common.Vec2;
        /**
         * Center world positions
         */
        c0: org.jbox2d.common.Vec2;
        /**
         * Center world positions
         */
        c: org.jbox2d.common.Vec2;
        /**
         * World angles
         */
        a0: number;
        /**
         * World angles
         */
        a: number;
        /**
         * Time interval = [t0,1], where t0 is in [0,1]
         */
        t0: number;
        toString(): string;
        constructor();
        set(argCloneFrom: Sweep): Sweep;
        /**
         * Get the interpolated transform at a specific time.
         * @param {org.jbox2d.common.XForm} xf the result is placed here - must not be null
         * @param {number} t the normalized time in [0,1].
         */
        getXForm(xf: org.jbox2d.common.XForm, t: number): void;
        /**
         *
         * Advance the sweep forward, yielding a new initial state.
         * @param {number} t the new initial time.
         */
        advance(t: number): void;
    }
}
declare namespace org.jbox2d.common {
    /**
     * A 2-dimensional vector class.  Used heavily in JBox2d.
     * djm: added ToOut methods
     * @param {number} x
     * @param {number} y
     * @class
     */
    class Vec2 {
        /**
         * Should we count Vec2 creations?
         */
        static watchCreations: boolean;
        /**
         * Running count of Vec2 creations.  Must be zeroed out
         * manually (perhaps at start of time step).  Incremented
         * in Vec2 constructor if watchCreations flag is true.
         * <BR><BR>
         * Mainly used for optimization purposes, since temporary
         * Vec2 creation is often a bottleneck.
         */
        static creationCount: number;
        x: number;
        y: number;
        constructor(x?: any, y?: any);
        /**
         * Zero out this vector.
         */
        setZero(): void;
        set$float$float(x: number, y: number): void;
        /**
         * Set the vector component-wise.
         * @param {number} x
         * @param {number} y
         */
        set(x?: any, y?: any): any;
        set$org_jbox2d_common_Vec2(v: Vec2): Vec2;
        /**
         * Return the sum of this vector and another; does not alter either one.
         * @param {org.jbox2d.common.Vec2} v
         * @return {org.jbox2d.common.Vec2}
         */
        add(v: Vec2): Vec2;
        /**
         * Return the difference of this vector and another; does not alter either one.
         * @param {org.jbox2d.common.Vec2} v
         * @return {org.jbox2d.common.Vec2}
         */
        sub(v: Vec2): Vec2;
        /**
         * Return this vector multiplied by a scalar; does not alter this vector.
         * @param {number} a
         * @return {org.jbox2d.common.Vec2}
         */
        mul(a: number): Vec2;
        /**
         * Return the negation of this vector; does not alter this vector.
         * @return {org.jbox2d.common.Vec2}
         */
        negate(): Vec2;
        /**
         * Flip the vector and return it - alters this vector.
         * @return {org.jbox2d.common.Vec2}
         */
        negateLocal(): Vec2;
        addLocal$org_jbox2d_common_Vec2(v: Vec2): Vec2;
        addLocal$float$float(x: number, y: number): Vec2;
        /**
         * Adds values to this vector and returns result - alters this vector.
         * @param {number} x
         * @param {number} y
         * @return {org.jbox2d.common.Vec2}
         */
        addLocal(x?: any, y?: any): any;
        /**
         * Subtract another vector from this one and return result - alters this vector.
         * @param {org.jbox2d.common.Vec2} v
         * @return {org.jbox2d.common.Vec2}
         */
        subLocal(v: Vec2): Vec2;
        /**
         * Multiply this vector by a number and return result - alters this vector.
         * @param {number} a
         * @return {org.jbox2d.common.Vec2}
         */
        mulLocal(a: number): Vec2;
        /**
         * Return the length of this vector.
         * @return {number}
         */
        length(): number;
        /**
         * Return the squared length of this vector.
         * @return {number}
         */
        lengthSquared(): number;
        /**
         * Normalize this vector and return the length before normalization.  Alters this vector.
         * @return {number}
         */
        normalize(): number;
        /**
         * True if the vector represents a pair of valid, non-infinite floating point numbers.
         * @return {boolean}
         */
        isValid(): boolean;
        /**
         * Return a new vector that has positive components.
         * @return {org.jbox2d.common.Vec2}
         */
        abs(): Vec2;
        absLocal(): void;
        /**
         *
         * @return {org.jbox2d.common.Vec2}
         */
        clone(): Vec2;
        /**
         *
         * @return {string}
         */
        toString(): string;
        static abs(a: Vec2): Vec2;
        static absToOut(a: Vec2, out: Vec2): void;
        static dot(a: Vec2, b: Vec2): number;
        static cross$org_jbox2d_common_Vec2$org_jbox2d_common_Vec2(a: Vec2, b: Vec2): number;
        static cross(a?: any, b?: any): any;
        static cross$org_jbox2d_common_Vec2$float(a: Vec2, s: number): Vec2;
        static crossToOut$org_jbox2d_common_Vec2$float$org_jbox2d_common_Vec2(a: Vec2, s: number, out: Vec2): void;
        static crossToOut(a?: any, s?: any, out?: any): any;
        static cross$float$org_jbox2d_common_Vec2(s: number, a: Vec2): Vec2;
        static crossToOut$float$org_jbox2d_common_Vec2$org_jbox2d_common_Vec2(s: number, a: Vec2, out: Vec2): void;
        static negateToOut(a: Vec2, out: Vec2): void;
        static min(a: Vec2, b: Vec2): Vec2;
        static max(a: Vec2, b: Vec2): Vec2;
        static minToOut(a: Vec2, b: Vec2, out: Vec2): void;
        static maxToOut(a: Vec2, b: Vec2, out: Vec2): void;
        /**
         * @see java.lang.Object#hashCode()
         * @return {number}
         */
        hashCode(): number;
        /**
         * @see java.lang.Object#equals(java.lang.Object)
         * @param {*} obj
         * @return {boolean}
         */
        equals(obj: any): boolean;
    }
}
declare namespace org.jbox2d.common {
    class Vec3 {
        x: number;
        y: number;
        z: number;
        constructor(argX?: any, argY?: any, argZ?: any);
        set(argVec: Vec3): void;
        addLocal(argVec: Vec3): Vec3;
        add(argVec: Vec3): Vec3;
        subLocal(argVec: Vec3): Vec3;
        sub(argVec: Vec3): Vec3;
        mulLocal(argScalar: number): Vec3;
        mul(argScalar: number): Vec3;
        negate(): Vec3;
        negateLocal(): Vec3;
        setZero(): void;
        clone(): Vec3;
        /**
         * @see java.lang.Object#hashCode()
         * @return {number}
         */
        hashCode(): number;
        /**
         * @see java.lang.Object#equals(java.lang.Object)
         * @param {*} obj
         * @return {boolean}
         */
        equals(obj: any): boolean;
        static dot(a: Vec3, b: Vec3): number;
        static cross(a: Vec3, b: Vec3): Vec3;
        static crossToOut(a: Vec3, b: Vec3, out: Vec3): void;
    }
}
declare namespace org.jbox2d.common {
    /**
     * Initialize using a position vector and a rotation matrix.
     * @param {org.jbox2d.common.Vec2} _position
     * @param {org.jbox2d.common.Mat22} _R
     * @class
     */
    class XForm {
        static __static_initialized: boolean;
        static __static_initialize(): void;
        /**
         * The translation caused by the transform
         */
        position: org.jbox2d.common.Vec2;
        /**
         * A matrix representing a rotation
         */
        R: org.jbox2d.common.Mat22;
        static identity: XForm;
        static identity_$LI$(): XForm;
        static __static_initializer_0(): void;
        constructor(_position?: any, _R?: any);
        /**
         * Set this to equal another transform.
         * @param {org.jbox2d.common.XForm} xf
         * @return {org.jbox2d.common.XForm}
         */
        set(xf: XForm): XForm;
        /**
         * Set this to the identity transform.
         */
        setIdentity(): void;
        static mul(T: XForm, v: org.jbox2d.common.Vec2): org.jbox2d.common.Vec2;
        static mulToOut(T: XForm, v: org.jbox2d.common.Vec2, out: org.jbox2d.common.Vec2): void;
        static mulTrans(T: XForm, v: org.jbox2d.common.Vec2): org.jbox2d.common.Vec2;
        static mulTransToOut(T: XForm, v: org.jbox2d.common.Vec2, out: org.jbox2d.common.Vec2): void;
        /**
         *
         * @return {string}
         */
        toString(): string;
    }
}
declare namespace org.jbox2d.dynamics {
    /**
     * Default body definition.  Defines a body with the following characteristics:
     * <UL>
     * <LI>Zero mass/inertia (you can set this later on the Body with setMassFromShapes)
     * <LI>null userData
     * <LI>Position of (0.0f, 0.0f)
     * <LI>Zero rotation
     * <LI>Zero damping (linear + angular)
     * <LI>Able to sleep, but not sleeping to start
     * <LI>Able to rotate
     * <LI>Not a bullet
     * </UL>
     * @class
     */
    class BodyDef {
        /**
         * You can use this to initialized the mass properties of the body.
         * If you prefer, you can set the mass properties after the shapes
         * have been added using Body::setMassFromShapes.
         */
        massData: org.jbox2d.collision.MassData;
        /**
         * Use this to store application specific body data
         */
        userData: any;
        /**
         * The world position of the body.  Avoid creating bodies at the origin
         * since this can lead to many overlapping shapes.
         */
        position: org.jbox2d.common.Vec2;
        /**
         * The world angle of the body in radians.
         */
        angle: number;
        /**
         * Linear damping is use to reduce the linear velocity. The damping parameter
         * can be larger than 1.0f but the damping effect becomes sensitive to the
         * time step when the damping parameter is large.
         */
        linearDamping: number;
        /**
         * Angular damping is use to reduce the angular velocity. The damping parameter
         * can be larger than 1.0f but the damping effect becomes sensitive to the
         * time step when the damping parameter is large.
         */
        angularDamping: number;
        /**
         * Set this flag to false if this body should never fall asleep.  Note that
         * this increases CPU usage.
         */
        allowSleep: boolean;
        /**
         * Is this body initially sleeping?
         */
        isSleeping: boolean;
        /**
         * Should this body be prevented from rotating?  Useful for characters.
         */
        fixedRotation: boolean;
        /**
         * Is this a fast moving body that should be prevented from tunneling through
         * other moving bodies? Note that all bodies are prevented from tunneling through
         * static bodies.
         * <BR><BR><em>Warning</em>: You should use this flag sparingly since it increases processing time.
         */
        isBullet: boolean;
        constructor();
    }
}
declare namespace org.jbox2d.dynamics {
    /**
     * Create a BodyGroup from a group of bodies.
     * @param {Array} bodies
     * @class
     * @author eric
     */
    class BodyGroup {
        children: Array<BodyGroup>;
        bodies: Array<org.jbox2d.dynamics.Body>;
        world: org.jbox2d.dynamics.World;
        constructor(...bodies: any[]);
        /**
         * Get child BodyGroups. Does not return children-of-children.
         * @return {org.jbox2d.dynamics.BodyGroup[]}
         */
        getChildren(): Array<BodyGroup>;
        /**
         * Get bodies. Does not return bodies that are members of child BodyGroups.
         *
         * @see #getBodiesDeep()
         * @return {org.jbox2d.dynamics.Body[]}
         */
        getBodies(): Array<org.jbox2d.dynamics.Body>;
        /**
         * Get all bodies in this group and all children, recursively.
         *
         * @see #getBodies() for non-recursive version
         * @return {org.jbox2d.dynamics.Body[]}
         */
        getBodiesDeep(): Array<org.jbox2d.dynamics.Body>;
        /**
         * @return {org.jbox2d.dynamics.World} the world this BodyGroup lives in
         */
        getWorld(): org.jbox2d.dynamics.World;
        /**
         * Create a BodyGroup by traversing the joint tree from a seed body.
         *
         * @param {org.jbox2d.dynamics.Body} seed
         * @return {org.jbox2d.dynamics.BodyGroup}
         */
        static fromSeedConnectedByJoints(seed: org.jbox2d.dynamics.Body): BodyGroup;
    }
}
declare namespace org.jbox2d.dynamics {
    /**
     * This is called when a body's shape passes outside of the world boundary.
     * @class
     */
    interface BoundaryListener {
        /**
         * This is called for each body that leaves the world boundary.
         * <BR><BR><em>Warning</em>: you can't modify the world inside this callback.
         * @param {org.jbox2d.dynamics.Body} body
         */
        violation(body: org.jbox2d.dynamics.Body): any;
    }
}
declare namespace org.jbox2d.dynamics {
    enum BoundaryResponse {
        FREEZE_BODY = 0,
        DESTROY_BODY = 1,
    }
}
declare namespace org.jbox2d.dynamics {
    /**
     * Implement this class to get collision results. You can use these results for
     * things like sounds and game logic. You can also get contact results by
     * traversing the contact lists after the time step. However, you might miss
     * some contacts because continuous physics leads to sub-stepping.
     * Additionally you may receive multiple callbacks for the same contact in a
     * single time step.
     * You should strive to make your callbacks efficient because there may be
     * many callbacks per time step.
     * <BR><BR><em>Warning</em>: The contact separation is the last computed value.
     * <BR><BR><em>Warning</em>: You cannot create/destroy Box2D entities inside these callbacks.
     * Buffer any such events and apply them at the end of the time step.
     * @class
     */
    interface ContactListener {
        /**
         * Called when a contact point is added. This includes the geometry
         * and the forces.
         * @param {org.jbox2d.dynamics.contacts.ContactPoint} point
         */
        add(point: org.jbox2d.dynamics.contacts.ContactPoint): any;
        /**
         * Called when a contact point persists. This includes the geometry
         * and the forces.
         * @param {org.jbox2d.dynamics.contacts.ContactPoint} point
         */
        persist(point: org.jbox2d.dynamics.contacts.ContactPoint): any;
        /**
         * Called when a contact point is removed. This includes the last
         * computed geometry and forces.
         * @param {org.jbox2d.dynamics.contacts.ContactPoint} point
         */
        remove(point: org.jbox2d.dynamics.contacts.ContactPoint): any;
        result(point: org.jbox2d.dynamics.contacts.ContactResult): any;
    }
}
declare namespace org.jbox2d.dynamics.contacts {
    /**
     * Base class for contacts between shapes.
     * @author ewjordan
     * @param {org.jbox2d.collision.shapes.Shape} s1
     * @param {org.jbox2d.collision.shapes.Shape} s2
     * @class
     */
    abstract class Contact {
        static e_nonSolidFlag: number;
        static e_slowFlag: number;
        static e_islandFlag: number;
        static e_toiFlag: number;
        static s_registers: Array<org.jbox2d.dynamics.contacts.ContactRegister>;
        static s_initialized: boolean;
        /**
         * The parent world.
         */
        m_world: org.jbox2d.dynamics.World;
        m_prev: Contact;
        m_next: Contact;
        /**
         * Node for connecting bodies.
         */
        m_node1: org.jbox2d.dynamics.contacts.ContactEdge;
        /**
         * Node for connecting bodies.
         */
        m_node2: org.jbox2d.dynamics.contacts.ContactEdge;
        m_shape1: org.jbox2d.collision.shapes.Shape;
        m_shape2: org.jbox2d.collision.shapes.Shape;
        /**
         * Combined friction
         */
        m_friction: number;
        /**
         * Combined restitution
         */
        m_restitution: number;
        m_flags: number;
        m_manifoldCount: number;
        m_toi: number;
        abstract evaluate(listener: org.jbox2d.dynamics.ContactListener): any;
        /**
         * Get the manifold array.
         * @return {org.jbox2d.collision.Manifold[]}
         */
        abstract getManifolds(): Array<org.jbox2d.collision.Manifold>;
        /**
         * Get the number of manifolds. This is 0 or 1 between convex shapes.
         * This may be greater than 1 for convex-vs-concave shapes. Each
         * manifold holds up to two contact points with a shared contact normal.
         * @return {number}
         */
        getManifoldCount(): number;
        isSolid(): boolean;
        constructor(s1?: any, s2?: any);
        getNext(): Contact;
        getShape1(): org.jbox2d.collision.shapes.Shape;
        getShape2(): org.jbox2d.collision.shapes.Shape;
        update(listener: org.jbox2d.dynamics.ContactListener): void;
        /**
         * returns a clone of this contact.  rev 166: not used in the engine
         * @return {org.jbox2d.dynamics.contacts.Contact}
         */
        abstract clone(): Contact;
        static initializeRegisters(): void;
        static addType(createFcn: org.jbox2d.dynamics.contacts.ContactCreateFcn, type1: org.jbox2d.collision.shapes.ShapeType, type2: org.jbox2d.collision.shapes.ShapeType): void;
        static createContact(shape1: org.jbox2d.collision.shapes.Shape, shape2: org.jbox2d.collision.shapes.Shape): Contact;
        static getContactRegister(type1: org.jbox2d.collision.shapes.ShapeType, type2: org.jbox2d.collision.shapes.ShapeType): org.jbox2d.dynamics.contacts.ContactRegister;
        static destroy(contact: Contact): void;
    }
}
declare namespace org.jbox2d.dynamics.contacts {
    class ContactConstraint {
        points: org.jbox2d.dynamics.contacts.ContactConstraintPoint[];
        normal: org.jbox2d.common.Vec2;
        manifold: org.jbox2d.collision.Manifold;
        body1: org.jbox2d.dynamics.Body;
        body2: org.jbox2d.dynamics.Body;
        friction: number;
        restitution: number;
        pointCount: number;
        constructor();
        set(cp: ContactConstraint): void;
    }
}
declare namespace org.jbox2d.dynamics.contacts {
    class ContactConstraintPoint {
        localAnchor1: org.jbox2d.common.Vec2;
        localAnchor2: org.jbox2d.common.Vec2;
        r1: org.jbox2d.common.Vec2;
        r2: org.jbox2d.common.Vec2;
        normalImpulse: number;
        tangentImpulse: number;
        positionImpulse: number;
        normalMass: number;
        tangentMass: number;
        equalizedMass: number;
        separation: number;
        velocityBias: number;
        constructor();
        set(cp: ContactConstraintPoint): void;
    }
}
declare namespace org.jbox2d.dynamics.contacts {
    interface ContactCreateFcn {
        create(s1: org.jbox2d.collision.shapes.Shape, s2: org.jbox2d.collision.shapes.Shape): org.jbox2d.dynamics.contacts.Contact;
    }
}
declare namespace org.jbox2d.dynamics.contacts {
    /**
     * A contact edge is used to connect bodies and contacts together
     * in a contact graph where each body is a node and each contact
     * is an edge. A contact edge belongs to a doubly linked list
     * maintained in each attached body. Each contact has two contact
     * nodes, one for each attached body.
     * @class
     */
    class ContactEdge {
        /**
         * Provides quick access to the other body attached.
         */
        other: org.jbox2d.dynamics.Body;
        /**
         * The contact.
         */
        contact: org.jbox2d.dynamics.contacts.Contact;
        /**
         * The previous contact edge in the body's contact list.
         */
        prev: ContactEdge;
        /**
         * The next contact edge in the body's contact list.
         */
        next: ContactEdge;
        set(argToCopy: ContactEdge): void;
        constructor();
    }
}
declare namespace org.jbox2d.dynamics.contacts {
    class ContactPoint {
        /**
         * The first shape
         */
        shape1: org.jbox2d.collision.shapes.Shape;
        /**
         * The second shape
         */
        shape2: org.jbox2d.collision.shapes.Shape;
        /**
         * Position in world coordinates
         */
        position: org.jbox2d.common.Vec2;
        /**
         * Velocity of point on body2 relative to point on body1 (pre-solver)
         */
        velocity: org.jbox2d.common.Vec2;
        /**
         * Points from shape1 to shape2
         */
        normal: org.jbox2d.common.Vec2;
        /**
         * The separation is negative when shapes are touching
         */
        separation: number;
        /**
         * The combined friction coefficient
         */
        friction: number;
        /**
         * The combined restitution coefficient
         */
        restitution: number;
        /**
         * The contact id identifies the features in contact
         */
        id: org.jbox2d.collision.ContactID;
        constructor();
    }
}
declare namespace org.jbox2d.dynamics.contacts {
    class ContactRegister {
        s1: org.jbox2d.collision.shapes.ShapeType;
        s2: org.jbox2d.collision.shapes.ShapeType;
        createFcn: org.jbox2d.dynamics.contacts.ContactCreateFcn;
        primary: boolean;
        constructor();
    }
}
declare namespace org.jbox2d.dynamics.contacts {
    /**
     * This structure is used to report contact point results.
     * @class
     */
    class ContactResult {
        /**
         * The first shape
         */
        shape1: org.jbox2d.collision.shapes.Shape;
        /**
         * The second shape
         */
        shape2: org.jbox2d.collision.shapes.Shape;
        /**
         * Position in world coordinates
         */
        position: org.jbox2d.common.Vec2;
        /**
         * Points from shape1 to shape2
         */
        normal: org.jbox2d.common.Vec2;
        /**
         * The normal impulse applied to body2
         */
        normalImpulse: number;
        /**
         * The tangent impulse applied to body2
         */
        tangentImpulse: number;
        /**
         * The contact id identifies the features in contact
         */
        id: org.jbox2d.collision.ContactID;
        constructor();
    }
}
declare namespace org.jbox2d.dynamics.contacts {
    class ContactSolver {
        m_step: org.jbox2d.dynamics.TimeStep;
        m_constraints: org.jbox2d.dynamics.contacts.ContactConstraint[];
        m_constraintCount: number;
        constructor();
        init(step: org.jbox2d.dynamics.TimeStep, contacts: org.jbox2d.dynamics.contacts.Contact[], contactCount: number): void;
        initVelocityConstraints(step: org.jbox2d.dynamics.TimeStep): void;
        solveVelocityConstraints(): void;
        finalizeVelocityConstraints(): void;
        solvePositionConstraints(baumgarte: number): boolean;
    }
}
declare namespace org.jbox2d.dynamics.controllers {
    /**
     *
     * Base class for controllers. Controllers are a convience for encapsulating common
     * per-step functionality.
     * @class
     */
    class Controller {
        /**
         * Controllers override this to implement per-step functionality.
         * @param {org.jbox2d.dynamics.TimeStep} step
         */
        step(step: org.jbox2d.dynamics.TimeStep): void;
        /**
         * Controllers override this to provide debug drawing.
         * @param {org.jbox2d.dynamics.DebugDraw} debugDraw
         */
        draw(debugDraw: org.jbox2d.dynamics.DebugDraw): void;
        /**
         * Adds a body to the controller list.
         * @param {org.jbox2d.dynamics.Body} body
         */
        addBody(body: org.jbox2d.dynamics.Body): void;
        /**
         * Removes a body from the controller list.
         * @param {org.jbox2d.dynamics.Body} body
         */
        removeBody(body: org.jbox2d.dynamics.Body): void;
        /**
         * Removes all bodies from the controller list.
         */
        clear(): void;
        /**
         * Get the next controller in the world's body list.
         * @return {org.jbox2d.dynamics.controllers.Controller}
         */
        getNext(): Controller;
        /**
         * Get the parent world of this body.
         * @return {org.jbox2d.dynamics.World}
         */
        getWorld(): org.jbox2d.dynamics.World;
        /**
         * Get the attached body list
         * @return {org.jbox2d.dynamics.controllers.ControllerEdge}
         */
        getBodyList(): org.jbox2d.dynamics.controllers.ControllerEdge;
        m_world: org.jbox2d.dynamics.World;
        m_bodyList: org.jbox2d.dynamics.controllers.ControllerEdge;
        m_bodyCount: number;
        constructor(def: org.jbox2d.dynamics.controllers.ControllerDef);
        m_prev: Controller;
        m_next: Controller;
    }
}
declare namespace org.jbox2d.dynamics.controllers {
    /**
     * @author eric
     * @class
     */
    abstract class ControllerDef {
        abstract create(): org.jbox2d.dynamics.controllers.Controller;
    }
}
declare namespace org.jbox2d.dynamics.controllers {
    /**
     * @author eric
     * @class
     */
    class ControllerEdge {
        /**
         * provides quick access to other end of this edge.
         */
        controller: org.jbox2d.dynamics.controllers.Controller;
        /**
         * the body
         */
        body: org.jbox2d.dynamics.Body;
        /**
         * the previous controller edge in the controllers's joint list
         */
        prevBody: ControllerEdge;
        /**
         * the next controller edge in the controllers's joint list
         */
        nextBody: ControllerEdge;
        /**
         * the previous controller edge in the body's joint list
         */
        prevController: ControllerEdge;
        /**
         * the next controller edge in the body's joint list
         */
        nextController: ControllerEdge;
        constructor();
    }
}
declare namespace org.jbox2d.dynamics {
    /**
     * Implement this abstract class to allow JBox2d to
     * automatically draw your physics for debugging purposes.
     * Not intended to replace your own custom rendering
     * routines!
     * @param {*} viewport
     * @class
     */
    abstract class DebugDraw {
        static e_shapeBit: number;
        static e_jointBit: number;
        static e_coreShapeBit: number;
        static e_aabbBit: number;
        static e_obbBit: number;
        static e_pairBit: number;
        static e_centerOfMassBit: number;
        static e_controllerBit: number;
        m_drawFlags: number;
        viewportTransform: org.jbox2d.common.IViewportTransform;
        constructor(viewport: org.jbox2d.common.IViewportTransform);
        setFlags(flags: number): void;
        getFlags(): number;
        appendFlags(flags: number): void;
        clearFlags(flags: number): void;
        abstract drawPolygon(vertices: org.jbox2d.common.Vec2[], vertexCount: number, color: org.jbox2d.common.Color3f): any;
        abstract drawSolidPolygon(vertices: org.jbox2d.common.Vec2[], vertexCount: number, color: org.jbox2d.common.Color3f): any;
        abstract drawCircle(center: org.jbox2d.common.Vec2, radius: number, color: org.jbox2d.common.Color3f): any;
        abstract drawSolidCircle(center: org.jbox2d.common.Vec2, radius: number, axis: org.jbox2d.common.Vec2, color: org.jbox2d.common.Color3f): any;
        abstract drawPoint(position: org.jbox2d.common.Vec2, f: number, color3f: org.jbox2d.common.Color3f): any;
        abstract drawSegment(p1: org.jbox2d.common.Vec2, p2: org.jbox2d.common.Vec2, color: org.jbox2d.common.Color3f): any;
        abstract drawXForm(xf: org.jbox2d.common.XForm): any;
        abstract drawString(x: number, y: number, s: string, color: org.jbox2d.common.Color3f): any;
        getViewportTranform(): org.jbox2d.common.IViewportTransform;
        /**
         * @param {number} x
         * @param {number} y
         * @param {number} scale
         * @see IViewportTransform#setCamera(float, float, float)
         */
        setCamera(x: number, y: number, scale: number): void;
        getScreenToWorldToOut$org_jbox2d_common_Vec2$org_jbox2d_common_Vec2(argScreen: org.jbox2d.common.Vec2, argWorld: org.jbox2d.common.Vec2): void;
        getWorldToScreenToOut$org_jbox2d_common_Vec2$org_jbox2d_common_Vec2(argWorld: org.jbox2d.common.Vec2, argScreen: org.jbox2d.common.Vec2): void;
        getWorldToScreenToOut$float$float$org_jbox2d_common_Vec2(worldX: number, worldY: number, argScreen: org.jbox2d.common.Vec2): void;
        /**
         * Takes the world coordinates and puts the corresponding screen
         * coordinates in argScreen.
         * @param {number} worldX
         * @param {number} worldY
         * @param {org.jbox2d.common.Vec2} argScreen
         */
        getWorldToScreenToOut(worldX?: any, worldY?: any, argScreen?: any): any;
        getWorldToScreen$org_jbox2d_common_Vec2(argWorld: org.jbox2d.common.Vec2): org.jbox2d.common.Vec2;
        getWorldToScreen$float$float(worldX: number, worldY: number): org.jbox2d.common.Vec2;
        /**
         * Takes the world coordinates and returns the screen
         * coordinates.
         * @param {number} worldX
         * @param {number} worldY
         * @return {org.jbox2d.common.Vec2}
         */
        getWorldToScreen(worldX?: any, worldY?: any): any;
        getScreenToWorldToOut$float$float$org_jbox2d_common_Vec2(screenX: number, screenY: number, argWorld: org.jbox2d.common.Vec2): void;
        /**
         * takes the screen coordinates and puts the corresponding
         * world coordinates in argWorld.
         * @param {number} screenX
         * @param {number} screenY
         * @param {org.jbox2d.common.Vec2} argWorld
         */
        getScreenToWorldToOut(screenX?: any, screenY?: any, argWorld?: any): any;
        getScreenToWorld$org_jbox2d_common_Vec2(argScreen: org.jbox2d.common.Vec2): org.jbox2d.common.Vec2;
        getScreenToWorld$float$float(screenX: number, screenY: number): org.jbox2d.common.Vec2;
        /**
         * takes the screen coordinates and returns the
         * world coordinates.
         * @param {number} screenX
         * @param {number} screenY
         * @return {org.jbox2d.common.Vec2}
         */
        getScreenToWorld(screenX?: any, screenY?: any): any;
    }
}
declare namespace org.jbox2d.dynamics {
    /**
     * Default sample implementation of ContactFilter.
     * @class
     */
    class DefaultContactFilter implements org.jbox2d.dynamics.ContactFilter {
        /**
         * Return true if contact calculations should be performed between these two shapes.
         * If you implement your own collision filter you may want to build from this implementation.
         * @param {org.jbox2d.collision.shapes.Shape} shape1
         * @param {org.jbox2d.collision.shapes.Shape} shape2
         * @return {boolean}
         */
        shouldCollide(shape1: org.jbox2d.collision.shapes.Shape, shape2: org.jbox2d.collision.shapes.Shape): boolean;
        rayCollide(userData: any, shape: org.jbox2d.collision.shapes.Shape): boolean;
        constructor();
    }
}
declare namespace org.jbox2d.dynamics {
    /**
     * Joints and shapes are destroyed when their associated
     * body is destroyed. Implement this listener so that you
     * may nullify references to these joints and shapes.
     * @class
     */
    interface DestructionListener {
        /**
         * Called when any joint is about to be destroyed due
         * to the destruction of one of its attached bodies.
         * @param {org.jbox2d.dynamics.joints.Joint} joint
         */
        sayGoodbye(joint?: any): any;
    }
}
declare namespace org.jbox2d.dynamics.joints {
    class Jacobian {
        linear1: org.jbox2d.common.Vec2;
        angular1: number;
        linear2: org.jbox2d.common.Vec2;
        angular2: number;
        constructor();
        setZero(): void;
        set(x1: org.jbox2d.common.Vec2, a1: number, x2: org.jbox2d.common.Vec2, a2: number): void;
        compute(x1: org.jbox2d.common.Vec2, a1: number, x2: org.jbox2d.common.Vec2, a2: number): number;
    }
}
declare namespace org.jbox2d.dynamics.joints {
    /**
     * Base class for all Joints
     * @param {org.jbox2d.dynamics.joints.JointDef} description
     * @class
     */
    abstract class Joint {
        m_type: org.jbox2d.dynamics.joints.JointType;
        m_prev: Joint;
        m_next: Joint;
        m_node1: org.jbox2d.dynamics.joints.JointEdge;
        m_node2: org.jbox2d.dynamics.joints.JointEdge;
        m_body1: org.jbox2d.dynamics.Body;
        m_body2: org.jbox2d.dynamics.Body;
        m_islandFlag: boolean;
        m_collideConnected: boolean;
        m_userData: any;
        m_inv_dt: number;
        constructor(description: org.jbox2d.dynamics.joints.JointDef);
        static destroy(j: Joint): void;
        destructor(): void;
        static create(description: org.jbox2d.dynamics.joints.JointDef): Joint;
        /**
         * Get the type of the concrete joint.
         * @return {org.jbox2d.dynamics.joints.JointType}
         */
        getType(): org.jbox2d.dynamics.joints.JointType;
        /**
         * Get the first body attached to this joint.
         * @return {org.jbox2d.dynamics.Body}
         */
        getBody1(): org.jbox2d.dynamics.Body;
        /**
         * Get the second body attached to this joint.
         * @return {org.jbox2d.dynamics.Body}
         */
        getBody2(): org.jbox2d.dynamics.Body;
        /**
         * Get the anchor point on body1 in world coordinates.
         * @return {org.jbox2d.common.Vec2}
         */
        abstract getAnchor1(): org.jbox2d.common.Vec2;
        /**
         * Get the anchor point on body2 in world coordinates.
         * @return {org.jbox2d.common.Vec2}
         */
        abstract getAnchor2(): org.jbox2d.common.Vec2;
        /**
         * Get the reaction force on body2 at the joint anchor.
         * @return {org.jbox2d.common.Vec2}
         */
        abstract getReactionForce(): org.jbox2d.common.Vec2;
        /**
         * Get the reaction torque on body2.
         * @return {number}
         */
        abstract getReactionTorque(): number;
        /**
         * Get the next joint the world joint list.
         * @return {org.jbox2d.dynamics.joints.Joint}
         */
        getNext(): Joint;
        /**
         * Get the user data pointer.
         * @return {*}
         */
        getUserData(): any;
        /**
         * Set the user data pointer.
         * @param {*} o
         */
        setUserData(o: any): void;
        abstract initVelocityConstraints(step: org.jbox2d.dynamics.TimeStep): any;
        abstract solveVelocityConstraints(step: org.jbox2d.dynamics.TimeStep): any;
        initPositionConstraints(): void;
        /**
         * This returns true if the position errors are within tolerance.
         * @return {boolean}
         */
        abstract solvePositionConstraints(): boolean;
    }
}
declare namespace org.jbox2d.dynamics.joints {
    /**
     * Base class for all joint definitions
     * @class
     */
    abstract class JointDef {
        type: org.jbox2d.dynamics.joints.JointType;
        body1: org.jbox2d.dynamics.Body;
        body2: org.jbox2d.dynamics.Body;
        userData: any;
        collideConnected: boolean;
        constructor();
    }
}
declare namespace org.jbox2d.dynamics.joints {
    class JointEdge {
        other: org.jbox2d.dynamics.Body;
        joint: org.jbox2d.dynamics.joints.Joint;
        prev: JointEdge;
        next: JointEdge;
        constructor();
    }
}
declare namespace org.jbox2d.dynamics.joints {
    enum JointType {
        UNKNOWN_JOINT = 0,
        REVOLUTE_JOINT = 1,
        PRISMATIC_JOINT = 2,
        DISTANCE_JOINT = 3,
        PULLEY_JOINT = 4,
        MOUSE_JOINT = 5,
        GEAR_JOINT = 6,
        CONSTANT_VOLUME_JOINT = 7,
    }
}
declare namespace org.jbox2d.dynamics.joints {
    enum LimitState {
        INACTIVE_LIMIT = 0,
        AT_LOWER_LIMIT = 1,
        AT_UPPER_LIMIT = 2,
        EQUAL_LIMITS = 3,
    }
}
declare namespace org.jbox2d.dynamics {
    /**
     * This interface allows registration within a JBox2d World
     * to be run immediately after the physics step.  This is
     * useful if you need to do something every step, but would
     * prefer not to have to manually code your step routine
     * differently, instead letting the engine handle the calling.
     * @class
     */
    interface Steppable {
        step(dt: number, iterations: number): any;
    }
}
declare namespace org.jbox2d.dynamics {
    /**
     * A holder for time step information.
     * @class
     */
    class TimeStep {
        dt: number;
        inv_dt: number;
        dtRatio: number;
        warmStarting: boolean;
        positionCorrection: boolean;
        maxIterations: number;
        constructor();
    }
}
declare namespace org.jbox2d.p5 {
    /**
     * Set of static methods for creating more complex bodies easily.
     * @class
     */
    class BodyUtils {
    }
}
declare namespace org.jbox2d.p5 {
    /**
     * A class to hold static methods for applying forces to bodies / shapes /
     * objects in various ways. If you find yourself duplicating code to do things
     * like this, then add it here!
     *
     * @author Greg
     *
     * @class
     */
    class ForceUtils {
        /**
         * Pushes a body towards the specified location.
         * @param {org.jbox2d.dynamics.Body} b
         * @param {org.jbox2d.common.Vec2} worldTarget
         * @param {number} force
         */
        pushTowards(b: org.jbox2d.dynamics.Body, worldTarget: org.jbox2d.common.Vec2, force: number): void;
        /**
         * Pushes a body in the specified direction.
         * @param {org.jbox2d.dynamics.Body} b
         * @param {org.jbox2d.common.Vec2} dir
         * @param {number} force
         */
        push(b: org.jbox2d.dynamics.Body, dir: org.jbox2d.common.Vec2, force: number): void;
    }
}
declare namespace org.jbox2d.p5 {
    /**
     * Tracks the reaction force of a given joint, and destroys the joint if the reaction force is above
     * a given threshold AND is between the minimum and maximum allowable angles. This is useful for creating
     * mouse-controlled joints where the "release" should only happen in one direction, and when a certain
     * force threshold is reached.
     * @author Greg
     * @param {org.jbox2d.dynamics.joints.Joint} joint
     * @param {number} reactionThreshold
     * @class
     */
    class JointBreaker implements org.jbox2d.dynamics.Steppable {
        joint: org.jbox2d.dynamics.joints.Joint;
        reactionThreshold: number;
        angleLo: number;
        angleHi: number;
        constructor(joint: org.jbox2d.dynamics.joints.Joint, reactionThreshold: number);
        step(dt: number, iterations: number): void;
        /**
         * Sets the range of angles between which this jointbreaker will break the joint. Angles are in radians,
         * relative to due east.
         * @param {number} lo
         * @param {number} hi
         */
        setAngleLimits(lo: number, hi: number): void;
    }
}
declare namespace org.jbox2d.p5 {
    class JointUtils {
        static createDistanceJoint$org_jbox2d_dynamics_World$org_jbox2d_common_Vec2$org_jbox2d_common_Vec2(w: org.jbox2d.dynamics.World, pA: org.jbox2d.common.Vec2, pB: org.jbox2d.common.Vec2): org.jbox2d.dynamics.joints.DistanceJoint;
        /**
         * Creates a distance joint between bodies that touch the given points,
         * anchored at those points. Returns null if there are not two different
         * bodies at the points given. Behavior is undetermined if more than one
         * body touches a point.
         *
         * @param {org.jbox2d.dynamics.World} w
         * World to add joint to and read bodies from
         * @param {org.jbox2d.common.Vec2} pA
         * First point to attach to
         * @param {org.jbox2d.common.Vec2} pB
         * Second point to attach to
         * @return {org.jbox2d.dynamics.joints.DistanceJoint} Joint created, or null if impossible to create the joint
         */
        static createDistanceJoint(w?: any, pA?: any, pB?: any): any;
        static createDistanceJoint$org_jbox2d_dynamics_Body$org_jbox2d_dynamics_Body(a: org.jbox2d.dynamics.Body, b: org.jbox2d.dynamics.Body): org.jbox2d.dynamics.joints.DistanceJoint;
        /**
         * Creates a RevoluteJoint between two bodies that rotates around a given
         * point in World coordinates.
         *
         * @param {org.jbox2d.dynamics.Body} a
         * @param {org.jbox2d.dynamics.Body} b
         * @param {org.jbox2d.common.Vec2} worldCenter
         * @return
         * @return {org.jbox2d.dynamics.joints.RevoluteJoint}
         */
        static createRevoluteJoint(a: org.jbox2d.dynamics.Body, b: org.jbox2d.dynamics.Body, worldCenter: org.jbox2d.common.Vec2): org.jbox2d.dynamics.joints.RevoluteJoint;
        /**
         * <p>
         * Finds a pair of non-identical bodies that have shapes touching points pA
         * and pB, loaded resp. into the 0th and 1st elements of the returned array.
         * Returns null array if no such pair exists.
         * </p>
         * <p>
         * This method is useful for creating joints based on location, where the
         * two bodies in a joint must be distinct.
         * </p>
         *
         * @param {org.jbox2d.dynamics.World} w
         * @param {org.jbox2d.common.Vec2} pA
         * @param {org.jbox2d.common.Vec2} pB
         * @return {Array} a Body array containing two distinct bodies, or NULL if no
         * distinct bodies exist at the given points.
         * @private
         */
        static getTwoDistinctBodies(w: org.jbox2d.dynamics.World, pA: org.jbox2d.common.Vec2, pB: org.jbox2d.common.Vec2): org.jbox2d.dynamics.Body[];
    }
}
declare namespace org.jbox2d.p5 {
    /**
     * Set up a physics world.
     *
     * @param {*} parent The PApplet this physics world should use
     * @param {number} gravX The x component of gravity, in meters/sec^2
     * @param {number} gravY The y component of gravity, in meters/sec^2
     * @param {number} screenAABBWidth The world's width, in pixels - should be significantly larger than the area you intend to use
     * @param {number} screenAABBHeight The world's height, in pixels - should be significantly larger than the area you intend to use
     * @param {number} borderBoxWidth The containing box's width - should be smaller than the world width, so that no object can escape
     * @param {number} borderBoxHeight The containing box's height - should be smaller than the world height, so that no object can escape
     * @param {number} pixelsPerMeter Pixels per physical meter
     * @param {number} screenW
     * @param {number} screenH
     * @class
     * @author ewjordan
     */
    class Physics {
        m_world: org.jbox2d.dynamics.World;
        m_parent: any;
        /**
         * m_draw is used by default for both drawing
         * and coordinate conversions; if a custom
         * rendering method is set, m_draw is <em>still</em>
         * used for coordinate conversions, so bear this
         * in mind.  In particular, however, none of the
         * camera functionality of ProcessingDebugDraw is
         * used, so don't expect anything like that.  All
         * the conversion does is applies a simple (read: unchanging)
         * scaling and translation from the default screen space to
         * the world space.  Any monkey business with cameras
         * and transformations is up to the user, and it's up
         * to the user to keep track of it.
         */
        m_draw: org.jbox2d.testbed.ProcessingDebugDraw;
        m_gravity: org.jbox2d.common.Vec2;
        m_worldAABB: org.jbox2d.collision.AABB;
        m_settings: org.jbox2d.testbed.TestSettings;
        m_border: org.jbox2d.dynamics.Body[];
        m_density: number;
        m_restitution: number;
        m_friction: number;
        m_bullet: boolean;
        m_sensor: boolean;
        m_customRenderingMethod: {
            owner: any;
            name: string;
            fn: Function;
        };
        m_customRenderingObject: any;
        constructor(parent?: any, screenW?: any, screenH?: any, gravX?: any, gravY?: any, screenAABBWidth?: any, screenAABBHeight?: any, borderBoxWidth?: any, borderBoxHeight?: any, pixelsPerMeter?: any);
        /**
         * Called automatically by Processing.
         */
        draw(): void;
        /**
         * For advanced users only.
         * <BR><BR>
         * Set a custom rendering method to be called.  Use this
         * if the default renderer is not drawing things the way
         * you like, or if you need more flexibility.  This
         * will allow a very modular approach to rendering, whereby
         * you can change the entire graphical style on the fly by
         * switching the render function.  Hopefully some people a
         * lot more graphically skilled than I am will provide some
         * cool looking functions for this purpose!
         * <BR><BR>
         * The method is set through Java's reflection API, so you
         * may call any method that takes a World object as a parameter.
         * The usual Java way is to force implementation of an interace,
         * but this way you can write things in a simpler manner, within
         * the PDE and without multiple tabs or pure Java.
         * Just pass the object that has the method along with the name
         * of the method.
         * <BR><BR>
         * e.g. if you have the following method defined in your
         * sketch:
         * <pre>
         * void myDrawMethod(World world) {
         * // Do a bunch of stuff
         * }
         * </pre>
         * then inside either the draw or setup functions you could write:
         * <pre>
         * setCustomRenderingMethod(this, "myDrawMethod");</pre>
         * to register that function.  Use unsetCustomRenderingMethod() to
         * go back to the default renderer.
         * <BR><BR>
         * If you're interested in writing your own renderer, you'll likely need
         * to look at the source code, which you can get to from http://www.jbox2d.org.
         * In particular, the org.jbox2d.dynamics.World file has a drawDebugData()
         * function, which, after some preprocessing, makes some calls out to
         * org.jbox2d.testbed.ProcessingDebugDraw to do the actual drawing.  That
         * should give you a place to start from, at least.
         * <BR><BR>
         * Note also that this rendering method has nothing to do with
         * Java2d vs. P3D vs. OpenGL - that choice is made at the
         * beginning of your sketch, and this library won't let you change it!
         * This functionality merely relates to the way bodies and shapes
         * are translated into drawing calls; the drawing calls themselves
         * rely on whatever renderer you chose in your size() function.
         * <BR><BR>
         * @param {*} object The object in which your method is defined ('this' should work if
         * the object is defined in a .pde file and not within a class)
         * @param {string} methodName The name of the method (without the parenthesis) to call
         */
        setCustomRenderingMethod(object: any, methodName: string): void;
        /**
         * Clear any custom rendering method that has been set, and
         * revert to the default Box2d debug renderer.
         */
        unsetCustomRenderingMethod(): void;
        /**
         * Draws the scene using the default render options.
         * Automatically called by the engine unless you have
         * specified your own rendering routine.
         * @param {org.jbox2d.dynamics.World} world
         */
        defaultDraw(world: org.jbox2d.dynamics.World): void;
        /**
         * Get an editable copy of the current TestSettings so
         * that you may change certain aspects of the simulation
         * and display.  You do not need to re-set anything after
         * editing these settings, the changes take effect immediately.
         * <BR><BR>
         * The list of useful fields in the TestSettings objects follows:
         * <pre>
         * public int hz; // "frame" rate of physics simulation - best to leave at 60
         * public int iterationCount; // number of constraint iterations - set to 10 normally
         * public boolean enableWarmStarting; // makes constraints work better by reusing last results
         * public boolean enablePositionCorrection; // leave this on...without it, things turn to mush
         * public boolean enableTOI; // enable/disable continuous collision detection
         * public boolean drawShapes;
         * public boolean drawJoints;
         * public boolean drawCoreShapes;
         * public boolean drawOBBs;
         * public boolean drawCOMs;
         * public boolean drawImpulses;
         * public boolean drawAABBs;
         * public boolean drawPairs;
         * public boolean drawContactPoints;
         * public boolean drawContactNormals;
         * public boolean drawContactForces;
         * public boolean drawFrictionForces;
         * </pre>
         * Note: the drawing settings only affect the default debug renderer.
         * If you have specified your own renderer, you will have to manually
         * read off and apply these settings if you wish to use them.
         * @return {org.jbox2d.testbed.TestSettings} A reference to the active TestSettings object
         */
        getSettings(): org.jbox2d.testbed.TestSettings;
        /**
         * Create a hollow box of the given screen dimensions.
         * @param {number} centerX Center of box x coordinate (in screen coordinates)
         * @param {number} centerY Center of box y coordinate (in screen coordinates)
         * @param {number} width Width of box (screen scale)
         * @param {number} height Height of box (screen scale)
         * @param {number} thickness Thickness of box edge (screen scale)
         * @return
         * @return {Array}
         */
        createHollowBox(centerX: number, centerY: number, width: number, height: number, thickness: number): org.jbox2d.dynamics.Body[];
        /**
         * Create a rectangle given by screen coordinates of corners.
         * @param {number} x0
         * @param {number} y0
         * @param {number} x1
         * @param {number} y1
         * @return
         * @return {org.jbox2d.dynamics.Body}
         */
        createRect(x0: number, y0: number, x1: number, y1: number): org.jbox2d.dynamics.Body;
        /**
         * Create a circle in screen coordinates
         * @param {number} x
         * @param {number} y
         * @param {number} r
         * @return
         * @return {org.jbox2d.dynamics.Body}
         */
        createCircle(x: number, y: number, r: number): org.jbox2d.dynamics.Body;
        /**
         * Create a polygon based on vertices.
         * <BR><BR>
         * Polygons must be:
         * <ul>
         * <li>Ordered clockwise in screen coordinates (which
         * becomes counterclockwise in world coordinates).
         * <li>Non self-intersecting.
         * <li>Convex
         * </ul>
         * Failure to adhere to any of these restrictions may cause
         * simulation crashes or problems.  In particular, if your
         * objects are showing up as static objects instead of dynamic
         * ones, and are not colliding correctly, you have probably
         * not met the clockwise ordering requirement.
         * <BR><BR>
         * This can be called with any number of vertices passed as
         * pairs of interleaved floats, for instance:
         * <pre>
         * createPolygon(x0,y0,x1,y1,x2,y2,x3,y3);</pre>
         * or
         * <pre>
         * createPolygon(x0,y0,x1,y1,x2,y2,x3,y3,x4,y4,x5,y5);</pre>
         * or
         * <pre>
         * float[] xyInterleaved = {x0,y0,x1,y1,x2,y2,x3,y3,x4,y4};
         * createPolygon(xyInterleaved);</pre>
         * are all fine.
         * @param {Array} vertices Any number of pairs of x,y floats, or an array of the same (screen coordinates)
         * @return
         * @return {org.jbox2d.dynamics.Body}
         */
        createPolygon(...vertices: number[]): org.jbox2d.dynamics.Body;
        /**
         * Create a distance (stick) joint between two bodies
         * that holds the specified points at a constant distance.
         * <BR><BR>
         * Once the distance joint is created, it may be turned into
         * a "soft" distance joint by using DistanceJoint::setFrequencyHz(float)
         * to set the frequency to a non-zero value, and using
         * DistanceJoint::setDampingRatio(float) to tune the damping constant.
         * <BR><BR>
         * Distance joints do not support joint limits or motors.
         * @param {org.jbox2d.dynamics.Body} a First body
         * @param {org.jbox2d.dynamics.Body} b Second body
         * @param {number} xa x component of anchor point on first body (screen coordinates)
         * @param {number} ya y component of anchor point on first body (screen coordinates)
         * @param {number} xb x component of anchor point on second body (screen coordinates)
         * @param {number} yb y component of anchor point on second body (screen coordinates)
         * @return {org.jbox2d.dynamics.joints.DistanceJoint} Newly created DistanceJoint
         */
        createDistanceJoint(a: org.jbox2d.dynamics.Body, b: org.jbox2d.dynamics.Body, xa: number, ya: number, xb: number, yb: number): org.jbox2d.dynamics.joints.DistanceJoint;
        /**
         * Create a revolute (pin) joint between the two bodies
         * at the given position.
         * <BR><BR>
         * Joint limits and motors may be set once the joint is created.
         * @param {org.jbox2d.dynamics.Body} a First body
         * @param {org.jbox2d.dynamics.Body} b Second body
         * @param {number} x x coordinate of pin joint location (screen coordinates)
         * @param {number} y y coordinate of pin joint location (screen coordinates)
         * @return {org.jbox2d.dynamics.joints.RevoluteJoint} Newly created RevoluteJoint
         */
        createRevoluteJoint(a: org.jbox2d.dynamics.Body, b: org.jbox2d.dynamics.Body, x: number, y: number): org.jbox2d.dynamics.joints.RevoluteJoint;
        /**
         * Create a prismatic (piston) joint between two bodies
         * that allows movement in the given direction.
         * <BR><BR>
         * dirX and dirY can be given in screen coordinates or
         * world coordinates, scaling does not matter.
         * <BR><BR>
         * Joint limits and motors may be set once the joint is created.
         * @param {org.jbox2d.dynamics.Body} a First body
         * @param {org.jbox2d.dynamics.Body} b Second body
         * @param {number} dirX x component of allowed movement direction
         * @param {number} dirY y component of allowed movement direction
         * @return {org.jbox2d.dynamics.joints.PrismaticJoint} Newly created PrismaticJoint
         */
        createPrismaticJoint(a: org.jbox2d.dynamics.Body, b: org.jbox2d.dynamics.Body, dirX: number, dirY: number): org.jbox2d.dynamics.joints.PrismaticJoint;
        /**
         * Create a pulley joint between the
         * The pulley joint is connected to two bodies and two fixed ground points.
         * The pulley supports a ratio such that:
         * length1 + ratio * length2 = constant
         * Yes, the force transmitted is scaled by the ratio.
         * <BR><BR>
         * The ground anchors are the points where the "rope" touches the pulley,
         * and the anchors are the points on the bodies where the rope is attached.
         * <BR><BR>
         * Joint limits may be set after the joint is created.
         * @param {org.jbox2d.dynamics.Body} a First body
         * @param {org.jbox2d.dynamics.Body} b Second body
         * @param {number} groundAnchorAx x coordinate of (fixed) ground anchor for body a, in screen coordinates
         * @param {number} groundAnchorAy y coordinate of (fixed) ground anchor for body a, in screen coordinates
         * @param {number} groundAnchorBx x coordinate of (fixed) ground anchor for body b, in screen coordinates
         * @param {number} groundAnchorBy y coordinate of (fixed) ground anchor for body b, in screen coordinates
         * @param {number} anchorAx x coordinate of body anchor for body a, in screen coordinates
         * @param {number} anchorAy y coordinate of body anchor for body a, in screen coordinates
         * @param {number} anchorBx x coordinate of body anchor for body b, in screen coordinates
         * @param {number} anchorBy y coordinate of body anchor for body b, in screen coordinates
         * @param {number} ratio "Block and tackle" ratio
         * @return {org.jbox2d.dynamics.joints.PulleyJoint} Newly created PulleyJoint
         */
        createPulleyJoint(a: org.jbox2d.dynamics.Body, b: org.jbox2d.dynamics.Body, groundAnchorAx: number, groundAnchorAy: number, groundAnchorBx: number, groundAnchorBy: number, anchorAx: number, anchorAy: number, anchorBx: number, anchorBy: number, ratio: number): org.jbox2d.dynamics.joints.PulleyJoint;
        /**
         * Create a gear joint, which binds together two existing
         * revolute or prismatic joints (any combination will work).
         * The provided joints must attach a dynamic body to a static body.
         * <BR><BR>
         * A gear joint is used to connect two joints together. Either joint
         * can be a revolute or prismatic joint. You specify a gear ratio
         * to bind the motions together:
         * coordinate1 + ratio * coordinate2 = constant
         * The ratio can be negative or positive. If one joint is a revolute joint
         * and the other joint is a prismatic joint, then the ratio will have units
         * of length or units of 1/length.
         * <BR><em>Warning</em>: The revolute and prismatic joints must be attached to
         * fixed bodies (which must be body1 on those joints).
         * @param {org.jbox2d.dynamics.joints.Joint} pj1 First joint (revolute or prismatic)
         * @param {org.jbox2d.dynamics.joints.Joint} pj2 Second joint (revolute or prismatic)
         * @param {number} ratio Gear ratio
         * @return {org.jbox2d.dynamics.joints.GearJoint} Newly created GearJoint
         */
        createGearJoint(pj1: org.jbox2d.dynamics.joints.Joint, pj2: org.jbox2d.dynamics.joints.Joint, ratio: number): org.jbox2d.dynamics.joints.GearJoint;
        /**
         * Sets the body def properties based on the current state
         * of the physics handler.
         *
         * @param {org.jbox2d.dynamics.BodyDef} bd
         * @private
         */
        setBodyDefProperties(bd: org.jbox2d.dynamics.BodyDef): void;
        /**
         * Sets the shape def properties based on the current state
         * of the physics handler.
         *
         * @param {org.jbox2d.collision.shapes.ShapeDef} sd Shape def to set
         * @private
         */
        setShapeDefProperties(sd: org.jbox2d.collision.shapes.ShapeDef): void;
        /**
         * Set the density used for newly created shapes.
         * @param {number} d
         */
        setDensity(d: number): void;
        /**
         * Get the density being used for newly created shapes.
         * @return
         * @return {number}
         */
        getDensity(): number;
        /**
         * Set the restitution used for newly created shapes.
         * @param {number} r
         */
        setRestitution(r: number): void;
        /**
         * Get the restitution being used for newly created shapes.
         * @return
         * @return {number}
         */
        getRestitution(): number;
        /**
         * Set the friction used for newly created shapes.
         * @param {number} f
         */
        setFriction(f: number): void;
        /**
         * Get the friction being used for newly created shapes.
         * @return
         * @return {number}
         */
        getFriction(): number;
        /**
         * Get the viewport transform, which transforms (moves, scales, and rotates)
         * the world to show us what we see in our window.
         * @return
         * @return {*}
         */
        getViewportTransform(): org.jbox2d.common.IViewportTransform;
        /**
         * Set to true to create new bodies as "bullets,"
         * which use (slower) continuous collision detection
         * against other moving bodies.
         * <BR><BR>
         * <em>Warning:</em> continuous collision detection between
         * moving bodies is slow, and should be used sparingly.  All
         * bodies use continuous collision detection against static
         * scenery, so for most purposes your bodies should not be
         * marked as bullets.
         * @param {boolean} bullet
         */
        setBullet(bullet: boolean): void;
        /**
         * Are newly created bodies being created as bullets?
         * @return {boolean}
         */
        getBullet(): boolean;
        /**
         * Set to true to create new shapes as sensors.  Sensors
         * do not respond to collisions physically, but they
         * generate contact events.  This can be useful if you
         * need to check whether a body is in a certain geometrical
         * area.
         * @param {boolean} sensor
         */
        setSensor(sensor: boolean): void;
        /**
         * Are newly created shapes being created as sensors?
         * @return {boolean}
         */
        getSensor(): boolean;
        /**
         * Destroy this world, unregistering it from the PApplet.
         * If this is not called, the world will still be active
         * and simulating, as upon creation it is registered with
         * the PApplet's draw events.
         */
        destroy(): void;
        /**
         * Get the current physics world.
         * <BR><BR>
         * <em>Warning:</em> anything involving a World object directly
         * is not strictly supported as part of this Processing library.
         * It <em>is</em> supported as part of JBox2d, however, so there
         * is quite a bit you can do, and you can always ask for help if
         * you run into trouble.  Note that all coordinates and vectors
         * in JBox2d proper are in world coordinates, not screen coordinates,
         * so you will likely need to use the screenToWorld and worldToScreen
         * functions to convert back and forth as necessary.
         * @return {org.jbox2d.dynamics.World} The active physics world
         */
        getWorld(): org.jbox2d.dynamics.World;
        /**
         *
         * Get the border Body[] array, or null
         * if the border has been removed.
         * @return {Array}
         */
        getBorder(): org.jbox2d.dynamics.Body[];
        /**
         * Remove the solid border if it exists.
         */
        removeBorder(): void;
        /**
         * Remove a body from the world.
         * @param {org.jbox2d.dynamics.Body} b
         */
        removeBody(b: org.jbox2d.dynamics.Body): void;
        /**
         * Remove a joint from the world.
         * @param {org.jbox2d.dynamics.joints.Joint} j
         */
        removeJoint(j: org.jbox2d.dynamics.joints.Joint): void;
        applyForce$org_jbox2d_dynamics_Body$float$float(b: org.jbox2d.dynamics.Body, fx: number, fy: number): void;
        applyForce$org_jbox2d_dynamics_Body$org_jbox2d_common_Vec2(b: org.jbox2d.dynamics.Body, f: org.jbox2d.common.Vec2): void;
        applyForce$org_jbox2d_dynamics_Body$float$float$float$float(b: org.jbox2d.dynamics.Body, fx: number, fy: number, pointX: number, pointY: number): void;
        /**
         * Apply a force to a body at a point.
         * @param {org.jbox2d.dynamics.Body} b Body you wish to apply force to
         * @param {number} fx x component of force (in pixel units)
         * @param {number} fy y component of force (in pixel units)
         * @param {number} pointX x coordinate of application point (in screen/pixel coordinates)
         * @param {number} pointY y coordinate of application point (in screen/pixel coordinates)
         */
        applyForce(b?: any, fx?: any, fy?: any, pointX?: any, pointY?: any): any;
        applyForce$org_jbox2d_dynamics_Body$org_jbox2d_common_Vec2$org_jbox2d_common_Vec2(b: org.jbox2d.dynamics.Body, f: org.jbox2d.common.Vec2, point: org.jbox2d.common.Vec2): void;
        /**
         *
         * Get the location of the body's origin (screen coordinates) -
         * note that this does <em>not</em> usually correspond to the
         * center of mass position, which may be obtained by calling
         * {@link #getCMPosition(Body)}.
         * @param {org.jbox2d.dynamics.Body} b
         * @return
         * @return {org.jbox2d.common.Vec2}
         */
        getPosition(b: org.jbox2d.dynamics.Body): org.jbox2d.common.Vec2;
        /**
         * Get the center of mass position (screen coordinates)
         * @param {org.jbox2d.dynamics.Body} b
         * @return {org.jbox2d.common.Vec2}
         */
        getCMPosition(b: org.jbox2d.dynamics.Body): org.jbox2d.common.Vec2;
        /**
         * Get the angle (in radians)
         * @param {org.jbox2d.dynamics.Body} b
         * @return {number}
         */
        getAngle(b: org.jbox2d.dynamics.Body): number;
        /**
         * Screen space to world space conversion for position.
         * @param {number} x
         * @param {number} y
         * @return {number}
         */
        screenToWorldX(x: number, y: number): number;
        /**
         * Screen space to world space conversion for position.
         * @param {number} x
         * @param {number} y
         * @return {number}
         */
        screenToWorldY(x: number, y: number): number;
        screenToWorld$float$float(x: number, y: number): org.jbox2d.common.Vec2;
        screenToWorld$org_jbox2d_common_Vec2(v: org.jbox2d.common.Vec2): org.jbox2d.common.Vec2;
        screenToWorld$float$org_jbox2d_common_Vec2(length: number, vector: org.jbox2d.common.Vec2): number;
        /**
         * Screen length to world length, on the given vector direction
         * @param {number} length
         * @param {org.jbox2d.common.Vec2} vector
         * @return {number}
         */
        screenToWorld(length?: any, vector?: any): any;
        /**
         * World space to screen space conversion for position.
         * @param {number} x
         * @param {number} y
         * @return {number}
         */
        worldToScreenX(x: number, y: number): number;
        /**
         * World space to screen space conversion for position.
         * @param {number} x
         * @param {number} y
         * @return {number}
         */
        worldToScreenY(x: number, y: number): number;
        worldToScreen$float$float(x: number, y: number): org.jbox2d.common.Vec2;
        worldToScreen$org_jbox2d_common_Vec2(v: org.jbox2d.common.Vec2): org.jbox2d.common.Vec2;
        worldToScreen$float$org_jbox2d_common_Vec2(length: number, vector: org.jbox2d.common.Vec2): number;
        /**
         * World length to screen length, on the given vector direction
         * @param {number} length
         * @param {org.jbox2d.common.Vec2} vector
         * @return {number}
         */
        worldToScreen(length?: any, vector?: any): any;
        screenToWorldVector$org_jbox2d_common_Vec2(screenV: org.jbox2d.common.Vec2): org.jbox2d.common.Vec2;
        screenToWorldVector$float$float(sx: number, sy: number): org.jbox2d.common.Vec2;
        screenToWorldVector(sx?: any, sy?: any): any;
        worldToScreenVector$org_jbox2d_common_Vec2(worldV: org.jbox2d.common.Vec2): org.jbox2d.common.Vec2;
        worldToScreenVector$float$float(wx: number, wy: number): org.jbox2d.common.Vec2;
        worldToScreenVector(wx?: any, wy?: any): any;
    }
}
declare namespace org.jbox2d.p5 {
    class PhysicsUtils {
        /**
         * In case you want to do something to a body during each creation (such as add an Actor object),
         * you can override the PhysicsUtils class and override this method.
         * @return
         * @return {org.jbox2d.dynamics.BodyDef}
         */
        static newBodyDef(): org.jbox2d.dynamics.BodyDef;
        static angle$org_jbox2d_common_Vec2$org_jbox2d_common_Vec2(a: org.jbox2d.common.Vec2, b: org.jbox2d.common.Vec2): number;
        /**
         * Returns the clockwise angle from vector A to vector B.
         * @param {org.jbox2d.common.Vec2} a
         * @param {org.jbox2d.common.Vec2} b
         * @return
         * @return {number}
         */
        static angle(a?: any, b?: any): any;
        static angle$org_jbox2d_common_Vec2(a: org.jbox2d.common.Vec2): number;
    }
}
declare namespace org.jbox2d.p5 {
    /**
     * A general class for holding static methods that do things with or to
     * polygons. Things like convex decomposition, polygon merging, &etc...
     *
     * @author Greg
     *
     * @class
     */
    class PolygonUtils {
    }
}
declare namespace org.jbox2d.pooling.arrays {
    abstract class DynamicTLArray<I> {
        tlMap: DynamicTLArray.TLHashMap<number, I[]>;
        get(argLength: number): I[];
        recycle(argArray: I[]): void;
        abstract getInitializedArray(argLength: number): I[];
    }
    namespace DynamicTLArray {
        class TLHashMap<K, V> {
            initialValue(): any;
            constructor();
        }
    }
}
declare namespace org.jbox2d.pooling {
    class SingletonPool {
        static pool: SingletonPool.Pool;
        static pool_$LI$(): SingletonPool.Pool;
        static getCollideCircle(): org.jbox2d.collision.shapes.CollideCircle;
        static getCollidePoly(): org.jbox2d.collision.shapes.CollidePoly;
        static getDistance(): org.jbox2d.collision.Distance;
    }
    namespace SingletonPool {
        class Singletons {
            collideCircle: org.jbox2d.collision.shapes.CollideCircle;
            collidePoly: org.jbox2d.collision.shapes.CollidePoly;
            distance: org.jbox2d.collision.Distance;
            constructor();
        }
        class Pool {
            initialValue(): SingletonPool.Singletons;
            constructor();
        }
    }
}
declare namespace org.jbox2d.pooling.stacks {
    abstract class DynamicTLStack<T> {
        tlStack: org.jbox2d.pooling.TLStack<T>;
        get(): T;
        recycle(argObject: T): void;
        abstract newObjectInstance(): T;
    }
}
declare namespace org.jbox2d.pooling {
    class TLAABB {
        initialValue(): org.jbox2d.collision.AABB;
    }
}
declare namespace org.jbox2d.pooling {
    class TLBoundValues {
        initialValue(): org.jbox2d.collision.BoundValues;
    }
}
declare namespace org.jbox2d.pooling {
    class TLContactPoint {
        initialValue(): org.jbox2d.dynamics.contacts.ContactPoint;
    }
}
declare namespace org.jbox2d.pooling {
    class TLContactSolver {
        initialValue(): org.jbox2d.dynamics.contacts.ContactSolver;
    }
}
declare namespace org.jbox2d.pooling {
    class TLManifold {
        initialValue(): org.jbox2d.collision.Manifold;
    }
}
declare namespace org.jbox2d.pooling {
    class TLMassData {
        initialValue(): org.jbox2d.collision.MassData;
    }
}
declare namespace org.jbox2d.pooling {
    class TLMat22 {
        initialValue(): org.jbox2d.common.Mat22;
    }
}
declare namespace org.jbox2d.pooling {
    class TLStack<T> {
        initialValue(): Array<T>;
    }
}
declare namespace org.jbox2d.pooling {
    class TLTimeStep {
        /**
         *
         * @return {org.jbox2d.dynamics.TimeStep}
         */
        initialValue(): org.jbox2d.dynamics.TimeStep;
    }
}
declare namespace org.jbox2d.pooling {
    class TLVec2 {
        initialValue(): org.jbox2d.common.Vec2;
    }
}
declare namespace org.jbox2d.pooling {
    class TLXForm {
        initialValue(): org.jbox2d.common.XForm;
    }
}
declare namespace org.jbox2d.testbed {
    /**
     * Holder for storing contact information.
     * @class
     */
    class ExampleContactPoint {
        shape1: org.jbox2d.collision.shapes.Shape;
        shape2: org.jbox2d.collision.shapes.Shape;
        normal: org.jbox2d.common.Vec2;
        position: org.jbox2d.common.Vec2;
        velocity: org.jbox2d.common.Vec2;
        id: org.jbox2d.collision.ContactID;
        state: number;
        constructor();
    }
}
declare namespace org.jbox2d.testbed.mathtests {
    class SinCosTable {
        static TWOPI: number;
        static TWOPI_$LI$(): number;
        static LERP_LOOKUP: boolean;
        precision: number;
        tableLength: number;
        sinLUT: number[];
        constructor(argPrecision: number);
        sin(x: number): number;
    }
}
declare namespace org.jbox2d.testbed.mathtests {
}
declare namespace org.jbox2d.testbed {
    /**
     * Settings for the current test.  Mostly self-explanatory.
     * <BR><BR>
     * The settings from here are applied during AbstractExample::step().
     * @class
     */
    class TestSettings {
        hz: number;
        iterationCount: number;
        enableWarmStarting: boolean;
        enablePositionCorrection: boolean;
        enableTOI: boolean;
        pause: boolean;
        singleStep: boolean;
        drawShapes: boolean;
        drawJoints: boolean;
        drawCoreShapes: boolean;
        drawOBBs: boolean;
        drawCOMs: boolean;
        drawStats: boolean;
        drawImpulses: boolean;
        drawAABBs: boolean;
        drawPairs: boolean;
        drawContactPoints: boolean;
        drawContactNormals: boolean;
        drawContactForces: boolean;
        drawFrictionForces: boolean;
        drawControllers: boolean;
        constructor();
    }
}
declare namespace org.jbox2d.testbed.timingTests {
    /**
     * Benchmark - piston example (constantly bumping a bunch of
     * circles and boxes).  Should be a decent mix of circle and
     * polygon collisions/contacts, though very little joint work.
     *
     * Rev 129 performance summary (details below class definition in source code):
     *
     * No bullets:
     * 1.6 Average FPS: 			390.21332
     * 1.6 -server Average FPS: 	470.05365
     *
     * (131+: with 1024M heap, 1.6 -server: 578.7675 FPS!)
     *
     * All bullets:
     * 1.6 Average FPS: 			185.98808
     * 1.6 -server Average FPS: 	221.55266
     *
     *
     * (C++ performance for no bullets is ~708 FPS, for comparison's sake)
     * @author eric
     * @class
     */
    class PistonBenchmark implements org.jbox2d.testbed.timingTests.SimpleTest {
        static BULLETS: boolean;
        m_joint1: org.jbox2d.dynamics.joints.RevoluteJoint;
        m_joint2: org.jbox2d.dynamics.joints.PrismaticJoint;
        create(world: org.jbox2d.dynamics.World): void;
        toString(): string;
        constructor();
    }
}
declare namespace org.jbox2d.testbed.timingTests {
    interface SimpleTest {
        create(world: org.jbox2d.dynamics.World): any;
    }
}
declare namespace org.jbox2d.util.blob {
    interface BlobContainer {
        /**
         * Is the Vec2 within the desired geometry?
         * @param {org.jbox2d.common.Vec2} p The point to test
         * @return {boolean} True if the geometry contains the point
         */
        containsPoint(p: org.jbox2d.common.Vec2): boolean;
        /**
         * Get the world AABB of the container.
         * @return {org.jbox2d.collision.AABB}
         */
        getAABB(): org.jbox2d.collision.AABB;
    }
}
declare namespace org.jbox2d.util.blob {
    /**
     * BlobMaker offers a static API for the creation of blobs.
     * @class
     */
    class BlobMaker {
        static pointRadius: number;
        static pointDensity: number;
        static pointFriction: number;
        static createBlob$org_jbox2d_util_blob_BlobStructure$org_jbox2d_util_blob_BlobContainer$org_jbox2d_dynamics_World(s: org.jbox2d.util.blob.BlobStructure, c: org.jbox2d.util.blob.BlobContainer, w: org.jbox2d.dynamics.World): void;
        static createBlob$org_jbox2d_util_blob_BlobStructure$org_jbox2d_util_blob_BlobContainer$org_jbox2d_dynamics_World$float$float(s: org.jbox2d.util.blob.BlobStructure, c: org.jbox2d.util.blob.BlobContainer, w: org.jbox2d.dynamics.World, scaleX: number, scaleY: number): void;
        static createBlob$org_jbox2d_util_blob_BlobStructure$org_jbox2d_util_blob_BlobContainer$org_jbox2d_dynamics_World$float$float$float$float(s: org.jbox2d.util.blob.BlobStructure, c: org.jbox2d.util.blob.BlobContainer, w: org.jbox2d.dynamics.World, scaleX: number, scaleY: number, transX: number, transY: number): void;
        /**
         * Creates a blob in a given physics world.
         * Applies the specified x/y scaling to the structure before fill.
         * <BR><BR>
         * The fill procedure aligns the structure with the upper
         * left corner of the container AABB, applies the scaling,
         * shifts the cell by the requested translation amounts,
         * then repeats the structure until the AABB is filled,
         * testing at each point whether the container is supposed
         * to have geometry there.
         * @param {org.jbox2d.util.blob.BlobStructure} s The BlobStructure definition
         * @param {*} c The BlobContainer that specifies the geometry to fill
         * @param {org.jbox2d.dynamics.World} w The World to create the blob in
         * @param {number} scaleX The world width of one repeating cell of the structure
         * @param {number} scaleY The world height of one repeating cell of the structure
         * @param {number} transX The world x offset of the cells from the AABB edge
         * @param {number} transY The world y offset of the cells from the AABB edge
         */
        static createBlob(s?: any, c?: any, w?: any, scaleX?: any, scaleY?: any, transX?: any, transY?: any): any;
        static createConnection(bodies: org.jbox2d.dynamics.Body[], a: number, b: number, frequency: number, damping: number, w: org.jbox2d.dynamics.World): org.jbox2d.dynamics.joints.Joint;
    }
}
declare namespace org.jbox2d.util.blob {
    class BlobPoint {
        position: org.jbox2d.common.Vec2;
        mass: number;
        constructor(x: number, y: number);
    }
}
declare namespace org.jbox2d.util.blob {
    /**
     * <p>Class for a toroidal repeating blob structure.
     * Should be subclassed with code to initialize
     * the structure in appropriate ways, such as for
     * a hexagonal lattice or a uniform grid.</p>
     * <p>Blobs are defined within an AABB from
     * (0,0)->(1,1) that is then repeated to fill the
     * full space after scaling (using the BlobMaker methods).
     * The connections list keeps track of pairs of BlobPoints
     * by index that are connected, and the connections* lists keep
     * track of connections outside the AABB (to the
     * corresponding points in the next regions).
     * </p>
     * <p>
     * Connections should only be defined once per pair.
     * </p>
     * <p>
     * This class does not allow for arbitrary repeated structures,
     * but most structures of interest will be expressible
     * as toroidally repeating in this way.
     * </p>
     * @class
     */
    class BlobStructure {
        points: Array<org.jbox2d.util.blob.BlobPoint>;
        connections: Array<BlobStructure.IntIntFloatFloat>;
        connectionsR: Array<BlobStructure.IntIntFloatFloat>;
        connectionsDR: Array<BlobStructure.IntIntFloatFloat>;
        connectionsD: Array<BlobStructure.IntIntFloatFloat>;
        connectionsUR: Array<BlobStructure.IntIntFloatFloat>;
        currentFrequency: number;
        currentDamping: number;
        updateSprings(): void;
        setSpringFrequency(freq: number): void;
        getSpringFrequency(): number;
        setSpringDamping(damp: number): void;
        getSpringDamping(): number;
        constructor();
        addPoint(p: org.jbox2d.util.blob.BlobPoint): number;
        addConnection$int$int(a: number, b: number): void;
        addConnection$int$int$org_jbox2d_util_blob_BlobStructure_Region(a: number, b: number, r: BlobStructure.Region): void;
        /**
         *
         * Add a connection between point at index a in the fundamental domain
         * and point at index b in region r.
         * <BR><BR>
         * Point indices can be obtained when points are added by storing the
         * return value of the addPoint method.
         * @param {number} a
         * @param {number} b
         * @param {org.jbox2d.util.blob.BlobStructure.Region} r
         */
        addConnection(a?: any, b?: any, r?: any): any;
    }
    namespace BlobStructure {
        class IntIntFloatFloat {
            __parent: any;
            a: number;
            b: number;
            c: number;
            d: number;
            constructor(__parent: any, _a: number, _b: number, _c: number, _d: number);
        }
        /**
         * Determines what region in toroidal space we're referring to.
         * @enum
         * @property {org.jbox2d.util.blob.BlobStructure.Region} DOWN
         * @property {org.jbox2d.util.blob.BlobStructure.Region} RIGHT
         * @property {org.jbox2d.util.blob.BlobStructure.Region} DOWN_RIGHT
         * @property {org.jbox2d.util.blob.BlobStructure.Region} UP_RIGHT
         * @property {org.jbox2d.util.blob.BlobStructure.Region} CENTER
         * @class
         */
        enum Region {
            DOWN = 0,
            RIGHT = 1,
            DOWN_RIGHT = 2,
            UP_RIGHT = 3,
            CENTER = 4,
        }
    }
}
declare namespace org.jbox2d.util.blob {
    /**
     * A circular blob container specified by radius and center.
     * @param {org.jbox2d.common.Vec2} _center
     * @param {number} _radius
     * @class
     */
    class CircularBlobContainer implements org.jbox2d.util.blob.BlobContainer {
        centerX: number;
        centerY: number;
        radius: number;
        radiusSqr: number;
        constructor(_center: org.jbox2d.common.Vec2, _radius: number);
        getRadius(): number;
        setRadius(r: number): void;
        getCenter(): org.jbox2d.common.Vec2;
        setCenter(c: org.jbox2d.common.Vec2): void;
        containsPoint(p: org.jbox2d.common.Vec2): boolean;
        getAABB(): org.jbox2d.collision.AABB;
    }
}
declare namespace org.jbox2d.util.blob {
    /**
     * A donut blob container specified by two radii and a center.
     * @param {org.jbox2d.common.Vec2} _center
     * @param {number} _radiusSmall
     * @param {number} _radiusLarge
     * @class
     */
    class DonutBlobContainer implements org.jbox2d.util.blob.BlobContainer {
        centerX: number;
        centerY: number;
        radiusLarge: number;
        radiusSmallSqr: number;
        radiusLargeSqr: number;
        constructor(_center: org.jbox2d.common.Vec2, _radiusSmall: number, _radiusLarge: number);
        containsPoint(p: org.jbox2d.common.Vec2): boolean;
        getAABB(): org.jbox2d.collision.AABB;
    }
}
declare namespace org.jbox2d.util.nonconvex {
    class Triangle {
        x: number[];
        y: number[];
        constructor(x1?: any, y1?: any, x2?: any, y2?: any, x3?: any, y3?: any);
        set(t: Triangle): void;
        containsPoint(_x: number, _y: number): boolean;
    }
}
declare namespace org.jbox2d.util.sph {
    class Parameter {
        numPts: number;
        velocityScale: number;
        initVel: org.jbox2d.common.Vec2;
        initMass: number;
        initDensity: number;
        initPressure: number;
        initPtSpacing: number;
        h: number;
        c: number;
        machNum: number;
        betaMax: number;
        nu: number;
        bodyFX: number;
        bodyFY: number;
        densityVariation: number;
        lengthScale: number;
        firstOutput: number;
        outputEvery: number;
        numSteps: number;
        deltaT: number;
        constructor(p?: any);
    }
}
declare namespace org.jbox2d.util.sph {
    class Particle {
        pos: org.jbox2d.common.Vec2;
        vel: org.jbox2d.common.Vec2;
        mass: number;
        deleted: number;
        constructor(p?: any, v?: any, m?: any);
        deleteParticle(): void;
        undeleteParticle(): void;
        isEmpty(): boolean;
    }
}
declare namespace org.jbox2d.util.sph {
    class SPHGroup {
        t: org.jbox2d.util.sph.SmoothParticle[];
        param: org.jbox2d.util.sph.Parameter;
        constructor(nParts: number, aabb: org.jbox2d.collision.AABB, parent: any, tStep: number);
        /**
         * This method calculates the pressure of each particle
         */
        calcPressure(): void;
        /**
         * Calculates SPH density/velocity contributions to p1 as a result of p2.
         * @param {org.jbox2d.util.sph.SmoothParticle} p1
         * @param {org.jbox2d.util.sph.SmoothParticle} p2
         */
        calcSPHEquations(p1: org.jbox2d.util.sph.SmoothParticle, p2: org.jbox2d.util.sph.SmoothParticle): void;
        /**
         * Calculates the correct timestep given the current setup of the system, prints
         * a message if the current time step is too large.
         */
        calcCorrectDeltaT(): void;
        /**
         * Handle a time step, including SPH forces
         */
        updateParticles(): void;
    }
}
declare namespace org.jbox2d.collision.shapes {
    /**
     * creates the circle definition at the given point with
     * a radius of 1
     * @param {org.jbox2d.common.Vec2} argVec
     * @class
     * @extends org.jbox2d.collision.shapes.ShapeDef
     */
    class CircleDef extends org.jbox2d.collision.shapes.ShapeDef {
        radius: number;
        localPosition: org.jbox2d.common.Vec2;
        constructor(argVec?: any);
    }
}
declare namespace org.jbox2d.collision.shapes {
    class EdgeChainDef extends org.jbox2d.collision.shapes.ShapeDef {
        /**
         * The vertices in local coordinates.
         */
        vertices: Array<org.jbox2d.common.Vec2>;
        /**
         * Whether to create an extra edge between the first and last vertices.
         */
        isALoop: boolean;
        constructor();
        /**
         * Add a vertex to the chain.
         * @param {org.jbox2d.common.Vec2} newV
         */
        addVertex(newV: org.jbox2d.common.Vec2): void;
        /**
         * Get the number of vertices in the chain.
         * @return
         * @return {number}
         */
        getVertexCount(): number;
        /**
         * Is the chain a closed loop?  If so,
         * an extra edge will be created between the
         * first and last vertices.
         * @return {boolean}
         */
        isLoop(): boolean;
        /**
         * Set whether an extra edge should be
         * created between first and last vertices.
         * @param {boolean} isLoop True if the chain should be a closed loop
         */
        setIsLoop(isLoop: boolean): void;
        /**
         * Return the raw vertex list.  Modifications
         * will effect the edge chain.
         * @return
         * @return {org.jbox2d.common.Vec2[]}
         */
        getVertices(): Array<org.jbox2d.common.Vec2>;
    }
}
declare namespace org.jbox2d.collision.shapes {
    /**
     * Point shape definition.
     * @class
     * @extends org.jbox2d.collision.shapes.ShapeDef
     */
    class PointDef extends org.jbox2d.collision.shapes.ShapeDef {
        localPosition: org.jbox2d.common.Vec2;
        mass: number;
        constructor();
    }
}
declare namespace org.jbox2d.collision.shapes {
    /**
     * Convex polygon. The vertices must be in CCW order for a right-handed
     * coordinate system with the z-axis coming out of the screen.
     * Add vertices using PolygonDef.add(Vec2),
     * and create the polygon shape using Body::createShape(ShapeDef).
     * @class
     * @extends org.jbox2d.collision.shapes.ShapeDef
     */
    class PolygonDef extends org.jbox2d.collision.shapes.ShapeDef {
        /**
         * The polygon vertices in local coordinates.
         * <BR><BR>
         * Accessing this field is discouraged - it remains
         * public for the moment, but that is likely to change.
         * Please use addVertex(Vec2) and getVertexList/Array
         * instead to add to or inspect the current vertices.
         */
        vertices: Array<org.jbox2d.common.Vec2>;
        set(copyMe: PolygonDef): void;
        constructor();
        /**
         * Add a vertex to the polygon.
         * @param {org.jbox2d.common.Vec2} v
         */
        addVertex(v: org.jbox2d.common.Vec2): void;
        /**
         * Removes all vertices.
         */
        clearVertices(): void;
        /**
         * Return the vertex list as an array.
         * @return {Array}
         */
        getVertexArray(): org.jbox2d.common.Vec2[];
        /**
         * Return the vertex list as a List<Vec2>.
         * @return {org.jbox2d.common.Vec2[]}
         */
        getVertexList(): Array<org.jbox2d.common.Vec2>;
        setAsBox$float$float(hx: number, hy: number): void;
        setAsBox$float$float$org_jbox2d_common_Vec2$float(hx: number, hy: number, center: org.jbox2d.common.Vec2, angle: number): void;
        /**
         * Build vertices to represent an oriented box.
         * @param {number} hx the half-width.
         * @param {number} hy the half-height.
         * @param {org.jbox2d.common.Vec2} center the center of the box in local coordinates.
         * @param {number} angle the rotation of the box in local coordinates.
         */
        setAsBox(hx?: any, hy?: any, center?: any, angle?: any): any;
        /**
         * Return the number of vertices.
         * @return {number}
         */
        getVertexCount(): number;
    }
}
declare namespace org.jbox2d.collision {
    class PairManager {
        static NULL_PAIR: number;
        static NULL_PAIR_$LI$(): number;
        static NULL_PROXY: number;
        static NULL_PROXY_$LI$(): number;
        static TABLE_CAPACITY: number;
        static TABLE_CAPACITY_$LI$(): number;
        static TABLE_MASK: number;
        static TABLE_MASK_$LI$(): number;
        m_pairs: org.jbox2d.collision.Pair[];
        m_pairCount: number;
        m_hashTable: number[];
        m_broadPhase: org.jbox2d.collision.BroadPhase;
        m_callback: org.jbox2d.collision.PairCallback;
        m_freePair: number;
        m_pairBuffer: org.jbox2d.collision.BufferedPair[];
        m_pairBufferCount: number;
        constructor();
        initialize(broadPhase: org.jbox2d.collision.BroadPhase, callback: org.jbox2d.collision.PairCallback): void;
        addPair(proxyId1: number, proxyId2: number): org.jbox2d.collision.Pair;
        removePair(proxyId1: number, proxyId2: number): any;
        /**
         * Buffer a pair for addition.
         * We may add a pair that is not in the pair manager or pair buffer.
         * We may add a pair that is already in the pair manager and pair buffer.
         * If the added pair is not a new pair, then it must be in the pair buffer (because RemovePair was called).
         * @param {number} id1
         * @param {number} id2
         */
        addBufferedPair(id1: number, id2: number): void;
        /**
         * Buffer a pair for removal.
         * @param {number} id1
         * @param {number} id2
         */
        removeBufferedPair(id1: number, id2: number): void;
        /**
         * commits the proxies
         */
        commit(): void;
        /**
         * Unimplemented - for debugging purposes only in C++ version
         */
        validateBuffer(): void;
        /**
         * For debugging
         */
        validateTable(): void;
        find$int$int$int(proxyId1: number, proxyId2: number, hash: number): org.jbox2d.collision.Pair;
        /**
         * finds the pair with the given hash
         * @param {number} proxyId1
         * @param {number} proxyId2
         * @param {number} hash
         * @return
         * @return {org.jbox2d.collision.Pair}
         */
        find(proxyId1?: any, proxyId2?: any, hash?: any): any;
        find$int$int(proxyId1: number, proxyId2: number): org.jbox2d.collision.Pair;
        hash(proxyId1: number, proxyId2: number): number;
        equals$org_jbox2d_collision_Pair$int$int(pair: org.jbox2d.collision.Pair, proxyId1: number, proxyId2: number): boolean;
        /**
         * returns if the pair has the two proxy id's
         * @param {org.jbox2d.collision.Pair} pair
         * @param {number} proxyId1
         * @param {number} proxyId2
         * @return
         * @return {boolean}
         */
        equals(pair?: any, proxyId1?: any, proxyId2?: any): any;
        equals$org_jbox2d_collision_BufferedPair$org_jbox2d_collision_BufferedPair(pair1: org.jbox2d.collision.BufferedPair, pair2: org.jbox2d.collision.BufferedPair): boolean;
        /**
         * For sorting.  Returns if the first pair's proxyid's are less than the
         * second pair, starting with proxyId1
         * @param {org.jbox2d.collision.BufferedPair} pair1
         * @param {org.jbox2d.collision.BufferedPair} pair2
         * @return
         * @return {boolean}
         */
        minor(pair1: org.jbox2d.collision.BufferedPair, pair2: org.jbox2d.collision.BufferedPair): boolean;
    }
}
declare namespace org.jbox2d.common {
    /**
     * A few math methods that don't fit very well anywhere else.
     * @class
     */
    class MathUtils {
        static __static_initialized: boolean;
        static __static_initialize(): void;
        static PI: number;
        static PI_$LI$(): number;
        static TWOPI: number;
        static TWOPI_$LI$(): number;
        static sinLUT: number[];
        static sinLUT_$LI$(): number[];
        static cosLUT: number[];
        static cosLUT_$LI$(): number[];
        static __static_initializer_0(): void;
        static sin(x: number): number;
        static cos(x: number): number;
        static abs(x: number): number;
        static floor(x: number): number;
        static ceil(x: number): number;
        static round(x: number): number;
        static max$float$float(a: number, b: number): number;
        static max$int$int(a: number, b: number): number;
        static max(a?: any, b?: any): any;
        static min(a: number, b: number): number;
        static map(val: number, fromMin: number, fromMax: number, toMin: number, toMax: number): number;
        static clamp$float$float$float(a: number, low: number, high: number): number;
        static clamp$org_jbox2d_common_Vec2$org_jbox2d_common_Vec2$org_jbox2d_common_Vec2(a: org.jbox2d.common.Vec2, low: org.jbox2d.common.Vec2, high: org.jbox2d.common.Vec2): org.jbox2d.common.Vec2;
        static clamp(a?: any, low?: any, high?: any): any;
        static clampToOut(a: org.jbox2d.common.Vec2, low: org.jbox2d.common.Vec2, high: org.jbox2d.common.Vec2, dest: org.jbox2d.common.Vec2): void;
        /**
         * Next Largest Power of 2: Given a binary integer value x, the next largest
         * power of 2 can be computed by a SWAR algorithm that recursively "folds"
         * the upper bits into the lower bits. This process yields a bit vector with
         * the same most significant 1 as x, but all 1's below it. Adding 1 to that
         * value yields the next largest power of 2.
         * @param {number} x
         * @return {number}
         */
        static nextPowerOfTwo(x: number): number;
        static isPowerOfTwo(x: number): boolean;
        static atan2(y: number, x: number): number;
        /**
         * Computes a fast approximation to <code>Math.pow(a, b)</code>.
         * Adapted from <url>http://www.dctsystems.co.uk/Software/power.html</url>.
         *
         * @param {number} a
         * a positive number
         * @param {number} b
         * a number
         * @return {number} a^b
         */
        static pow(a: number, b: number): number;
        static sqrt(x: number): number;
        static invSqrt(x: number): number;
    }
}
declare namespace org.jbox2d.util.nonconvex {
    /**
     *
     * @author ewjordan
     * @param {Array} _x
     * @param {Array} _y
     * @param {number} nVert
     * @class
     */
    class Polygon {
        static toiSlop: number;
        static toiSlop_$LI$(): number;
        static B2_POLYGON_REPORT_ERRORS: boolean;
        static COLLAPSE_DIST_SQR: number;
        static COLLAPSE_DIST_SQR_$LI$(): number;
        static maxPolygonVertices: number;
        static maxPolygonVertices_$LI$(): number;
        nVertices: number;
        x: number[];
        y: number[];
        areaIsSet: boolean;
        area: number;
        /**
         * Check if the lines a0->a1 and b0->b1 cross.
         * If they do, intersectionPoint will be filled
         * with the point of crossing.
         *
         * Grazing lines should not return true, though
         * this is not very robust.
         *
         * @param {org.jbox2d.common.Vec2} a0
         * @param {org.jbox2d.common.Vec2} a1
         * @param {org.jbox2d.common.Vec2} b0
         * @param {org.jbox2d.common.Vec2} b1
         * @param {org.jbox2d.common.Vec2} intersectionPoint
         * @return {boolean}
         */
        static intersect(a0: org.jbox2d.common.Vec2, a1: org.jbox2d.common.Vec2, b0: org.jbox2d.common.Vec2, b1: org.jbox2d.common.Vec2, intersectionPoint: org.jbox2d.common.Vec2): boolean;
        intersect(a0: org.jbox2d.common.Vec2, a1: org.jbox2d.common.Vec2, b0: org.jbox2d.common.Vec2, b1: org.jbox2d.common.Vec2): boolean;
        constructor(_x?: any, _y?: any, nVert?: any);
        getArea(): number;
        isCCW(): boolean;
        mergeParallelEdges(tolerance: number): void;
        getVertexVecs(): org.jbox2d.common.Vec2[];
        set(p: Polygon): void;
        /**
         * Assuming the polygon is simple, checks if it is convex.
         * @return {boolean}
         */
        isConvex(): boolean;
        static polyCentroid(vs: org.jbox2d.common.Vec2[], count: number): org.jbox2d.common.Vec2;
        isUsable$boolean(printErrors: boolean): boolean;
        /**
         * Checks if polygon is valid for use in Box2d engine.
         * Last ditch effort to ensure no invalid polygons are
         * added to world geometry.
         *
         * Performs a full check, for simplicity, convexity,
         * orientation, minimum angle, and volume.  This won't
         * be very efficient, and a lot of it is redundant when
         * other tools in this section are used.
         * @param {boolean} printErrors
         * @return {boolean}
         */
        isUsable(printErrors?: any): any;
        isUsable$(): boolean;
        isSimple(): boolean;
        /**
         * Tries to add a triangle to the polygon. Returns null if it can't connect
         * properly, otherwise returns a pointer to the new Polygon. Assumes bitwise
         * equality of joined vertex positions.
         *
         * Remember to delete the pointer afterwards.
         * TODO: Make this return a Polygon instead
         * of a pointer to a heap-allocated one.
         *
         * For internal use.
         * @param {org.jbox2d.util.nonconvex.Triangle} t
         * @return {org.jbox2d.util.nonconvex.Polygon}
         */
        add(t: org.jbox2d.util.nonconvex.Triangle): Polygon;
        /**
         * Adds this polygon to a PolyDef.
         * @param {org.jbox2d.collision.shapes.PolygonDef} pd
         */
        addTo(pd: org.jbox2d.collision.shapes.PolygonDef): void;
        /**
         * Finds and fixes "pinch points," points where two polygon
         * vertices are at the same point.
         *
         * If a pinch point is found, pin is broken up into poutA and poutB
         * and true is returned; otherwise, returns false.
         *
         * Mostly for internal use.
         *
         * O(N^2) time, which sucks...
         * @param {org.jbox2d.util.nonconvex.Polygon} pin
         * @param {org.jbox2d.util.nonconvex.Polygon} poutA
         * @param {org.jbox2d.util.nonconvex.Polygon} poutB
         * @return {boolean}
         * @private
         */
        static resolvePinchPoint(pin: Polygon, poutA: Polygon, poutB: Polygon): boolean;
        /**
         * Triangulates a polygon using simple ear-clipping algorithm. Returns
         * size of Triangle array unless the polygon can't be triangulated.
         * This should only happen if the polygon self-intersects,
         * though it will not _always_ return null for a bad polygon - it is the
         * caller's responsibility to check for self-intersection, and if it
         * doesn't, it should at least check that the return value is non-null
         * before using. You're warned!
         *
         * Triangles may be degenerate, especially if you have identical points
         * in the input to the algorithm.  Check this before you use them.
         *
         * This is totally unoptimized, so for large polygons it should not be part
         * of the simulation loop.
         *
         * Returns:
         * -1 if algorithm fails (self-intersection most likely)
         * 0 if there are not enough vertices to triangulate anything.
         * Number of triangles if triangulation was successful.
         *
         * results will be filled with results - ear clipping always creates vNum - 2
         * or fewer (due to pinch point polygon snipping), so allocate an array of
         * this size.
         * @param {Array} xv
         * @param {Array} yv
         * @param {number} vNum
         * @param {Array} results
         * @return {number}
         */
        static triangulatePolygon(xv: number[], yv: number[], vNum: number, results: org.jbox2d.util.nonconvex.Triangle[]): number;
        /**
         * Turns a list of triangles into a list of convex polygons. Very simple
         * method - start with a seed triangle, keep adding triangles to it until
         * you can't add any more without making the polygon non-convex.
         *
         * Returns an integer telling how many polygons were created.  Will fill
         * polys array up to polysLength entries, which may be smaller or larger
         * than the return value.
         *
         * Takes O(N*P) where P is the number of resultant polygons, N is triangle
         * count.
         *
         * The final polygon list will not necessarily be minimal, though in
         * practice it works fairly well.
         * @param {Array} triangulated
         * @param {number} triangulatedLength
         * @param {Array} polys
         * @param {number} polysLength
         * @return {number}
         */
        static polygonizeTriangles(triangulated: org.jbox2d.util.nonconvex.Triangle[], triangulatedLength: number, polys: Polygon[], polysLength: number): number;
        /**
         * Checks if vertex i is the tip of an ear in polygon defined by xv[] and
         * yv[].
         *
         * Assumes clockwise orientation of polygon...ick
         * @param {number} i
         * @param {Array} xv
         * @param {Array} yv
         * @param {number} xvLength
         * @return {boolean}
         * @private
         */
        static isEar(i: number, xv: number[], yv: number[], xvLength: number): boolean;
        static reversePolygon$org_jbox2d_util_nonconvex_Polygon(p: Polygon): void;
        static reversePolygon$float_A$float_A$int(x: number[], y: number[], n: number): void;
        static reversePolygon(x?: any, y?: any, n?: any): any;
        /**
         * Decomposes a non-convex polygon into a number of convex polygons, up
         * to maxPolys (remaining pieces are thrown out, but the total number
         * is returned, so the return value can be greater than maxPolys).
         *
         * Each resulting polygon will have no more than maxVerticesPerPolygon
         * vertices (set to b2MaxPolyVertices by default, though you can change
         * this).
         *
         * Returns -1 if operation fails (usually due to self-intersection of
         * polygon).
         * @param {org.jbox2d.util.nonconvex.Polygon} p
         * @param {Array} results
         * @param {number} maxPolys
         * @return {number}
         */
        static decomposeConvex(p: Polygon, results: Polygon[], maxPolys: number): number;
        /**
         * Decomposes a polygon into convex polygons and adds all pieces to a b2BodyDef
         * using a prototype b2PolyDef. All fields of the prototype are used for every
         * shape except the vertices (friction, restitution, density, filter, etc).
         *
         * If you want finer control, you'll have to add everything by hand.
         *
         * This is the simplest method to add a complicated polygon to a body.
         * @param {org.jbox2d.util.nonconvex.Polygon} p
         * @param {org.jbox2d.dynamics.Body} bd
         * @param {org.jbox2d.collision.shapes.PolygonDef} prototype
         */
        static decomposeConvexAndAddTo(p: Polygon, bd: org.jbox2d.dynamics.Body, prototype: org.jbox2d.collision.shapes.PolygonDef): void;
        static convexHull$org_jbox2d_common_Vec2_A$int(v: org.jbox2d.common.Vec2[], nVert: number): Polygon;
        static convexHull$float_A$float_A$int(cloudX: number[], cloudY: number[], nVert: number): Polygon;
        static convexHull(cloudX?: any, cloudY?: any, nVert?: any): any;
        static isRighter(sinA: number, cosA: number, sinB: number, cosB: number): boolean;
        static remainder(x: number, modulus: number): number;
        static traceEdge(p: Polygon): Polygon;
        print(): void;
        printFormatted(): void;
    }
    class PolyNode {
        static MAX_CONNECTED: number;
        nConnected: number;
        visited: boolean;
        position: org.jbox2d.common.Vec2;
        connected: PolyNode[];
        constructor(pos?: any);
        addConnection(toMe: PolyNode): void;
        removeConnection(fromMe: PolyNode): void;
        removeConnectionByIndex(index: number): void;
        isConnectedTo(me: PolyNode): boolean;
        getRightestConnection$org_jbox2d_util_nonconvex_PolyNode(incoming: PolyNode): PolyNode;
        getRightestConnection(incoming?: any): any;
        getRightestConnection$org_jbox2d_common_Vec2(incomingDir: org.jbox2d.common.Vec2): PolyNode;
    }
}
declare namespace org.jbox2d.dynamics.contacts {
    class NullContact extends org.jbox2d.dynamics.contacts.Contact {
        /**
         *
         * @param {*} cl
         */
        evaluate(cl: org.jbox2d.dynamics.ContactListener): void;
        constructor();
        clone(): org.jbox2d.dynamics.contacts.Contact;
        /**
         *
         * @return {org.jbox2d.collision.Manifold[]}
         */
        getManifolds(): Array<org.jbox2d.collision.Manifold>;
        getFirstManifold(): org.jbox2d.collision.Manifold;
    }
}
declare namespace org.jbox2d.dynamics.controllers {
    /**
     * @author eric
     * @extends org.jbox2d.dynamics.controllers.Controller
     * @class
     */
    class BuoyancyController extends org.jbox2d.dynamics.controllers.Controller {
        constructor(def: org.jbox2d.dynamics.controllers.BuoyancyControllerDef);
        /**
         * The outer surface normal
         */
        normal: org.jbox2d.common.Vec2;
        /**
         * The height of the fluid surface along the normal
         */
        offset: number;
        /**
         * The fluid density
         */
        density: number;
        /**
         * Fluid velocity, for drag calculations
         */
        velocity: org.jbox2d.common.Vec2;
        /**
         * Linear drag co-efficient
         */
        linearDrag: number;
        /**
         * Linear drag co-efficient
         */
        angularDrag: number;
        /**
         * If false, bodies are assumed to be uniformly dense, otherwise use the shapes densities
         */
        useDensity: boolean;
        /**
         * If true, gravity is taken from the world instead of the gravity parameter.
         */
        useWorldGravity: boolean;
        /**
         * Gravity vector, if the world's gravity is not used
         */
        gravity: org.jbox2d.common.Vec2;
        /**
         *
         * @param {org.jbox2d.dynamics.TimeStep} step
         */
        step(step: org.jbox2d.dynamics.TimeStep): void;
        /**
         *
         * @param {org.jbox2d.dynamics.DebugDraw} debugDraw
         */
        draw(debugDraw: org.jbox2d.dynamics.DebugDraw): void;
    }
}
declare namespace org.jbox2d.dynamics.controllers {
    /**
     * @author eric
     * @class
     * @extends org.jbox2d.dynamics.controllers.ControllerDef
     */
    class BuoyancyControllerDef extends org.jbox2d.dynamics.controllers.ControllerDef {
        normal: org.jbox2d.common.Vec2;
        offset: number;
        density: number;
        velocity: org.jbox2d.common.Vec2;
        linearDrag: number;
        angularDrag: number;
        useDensity: boolean;
        useWorldGravity: boolean;
        gravity: org.jbox2d.common.Vec2;
        constructor();
        /**
         * @see org.jbox2d.dynamics.controllers.ControllerDef#create()
         * @return {org.jbox2d.dynamics.controllers.Controller}
         */
        create(): org.jbox2d.dynamics.controllers.Controller;
    }
}
declare namespace org.jbox2d.testbed {
    /**
     * Implementation of DebugDraw using Processing (http://www.processing.org)
     *
     * @author ewjordan
     * @param {*} pApplet
     * @class
     * @extends org.jbox2d.dynamics.DebugDraw
     */
    class ProcessingDebugDraw extends org.jbox2d.dynamics.DebugDraw {
        static screen: ProcessingDebugDraw;
        firstTime: boolean;
        g: any;
        pg: PGraphics;
        m_font: PFont;
        fontHeight: number;
        transform: org.jbox2d.common.OBBViewportTransform;
        constructor(pApplet: any);
        static circlePt: org.jbox2d.common.Vec2;
        static circlePt_$LI$(): org.jbox2d.common.Vec2;
        /**
         *
         * @param {org.jbox2d.common.Vec2} argCenter
         * @param {number} radius
         * @param {org.jbox2d.common.Color3f} color
         */
        drawCircle(argCenter: org.jbox2d.common.Vec2, radius: number, color: org.jbox2d.common.Color3f): void;
        static p: org.jbox2d.common.Vec2;
        static p_$LI$(): org.jbox2d.common.Vec2;
        static center: org.jbox2d.common.Vec2;
        static center_$LI$(): org.jbox2d.common.Vec2;
        /**
         *
         * @param {org.jbox2d.common.Vec2} argCenter
         * @param {number} radius
         * @param {org.jbox2d.common.Vec2} axis
         * @param {org.jbox2d.common.Color3f} color
         */
        drawSolidCircle(argCenter: org.jbox2d.common.Vec2, radius: number, axis: org.jbox2d.common.Vec2, color: org.jbox2d.common.Color3f): void;
        static v1: org.jbox2d.common.Vec2;
        static v1_$LI$(): org.jbox2d.common.Vec2;
        static v2: org.jbox2d.common.Vec2;
        static v2_$LI$(): org.jbox2d.common.Vec2;
        /**
         *
         * @param {Array} vertices
         * @param {number} vertexCount
         * @param {org.jbox2d.common.Color3f} color
         */
        drawPolygon(vertices: org.jbox2d.common.Vec2[], vertexCount: number, color: org.jbox2d.common.Color3f): void;
        static v: org.jbox2d.common.Vec2;
        static v_$LI$(): org.jbox2d.common.Vec2;
        /**
         *
         * @param {Array} vertices
         * @param {number} vertexCount
         * @param {org.jbox2d.common.Color3f} color
         */
        drawSolidPolygon(vertices: org.jbox2d.common.Vec2[], vertexCount: number, color: org.jbox2d.common.Color3f): void;
        /**
         *
         * @param {org.jbox2d.common.Vec2} argP1
         * @param {org.jbox2d.common.Vec2} argP2
         * @param {org.jbox2d.common.Color3f} color
         */
        drawSegment(argP1: org.jbox2d.common.Vec2, argP2: org.jbox2d.common.Vec2, color: org.jbox2d.common.Color3f): void;
        static p1: org.jbox2d.common.Vec2;
        static p1_$LI$(): org.jbox2d.common.Vec2;
        static p2: org.jbox2d.common.Vec2;
        static p2_$LI$(): org.jbox2d.common.Vec2;
        static p1world: org.jbox2d.common.Vec2;
        static p1world_$LI$(): org.jbox2d.common.Vec2;
        static p2world: org.jbox2d.common.Vec2;
        static p2world_$LI$(): org.jbox2d.common.Vec2;
        /**
         *
         * @param {org.jbox2d.common.XForm} xf
         */
        drawXForm(xf: org.jbox2d.common.XForm): void;
        /**
         *
         * @param {number} x
         * @param {number} y
         * @param {string} s
         * @param {org.jbox2d.common.Color3f} color
         */
        drawString(x: number, y: number, s: string, color: org.jbox2d.common.Color3f): void;
        static position: org.jbox2d.common.Vec2;
        static position_$LI$(): org.jbox2d.common.Vec2;
        /**
         *
         * @param {org.jbox2d.common.Vec2} argPosition
         * @param {number} f
         * @param {org.jbox2d.common.Color3f} color
         */
        drawPoint(argPosition: org.jbox2d.common.Vec2, f: number, color: org.jbox2d.common.Color3f): void;
        static localOffset: org.jbox2d.common.Vec2;
        static localOffset_$LI$(): org.jbox2d.common.Vec2;
        /**
         * First image is centered on position, then
         * rotation, then localOffset is applied, and
         * lastly localScale.
         * <BR><BR>
         * Thus localOffset should be specified in world
         * units before scaling is applied.
         * For instance, if you want a MxN image to have its corner
         * at body center and be scaled by S, use a localOffset
         * of (M*S/2, N*S/2) and a localScale of S.
         * <BR><BR>
         *
         * @param {PImage} image
         * @param {org.jbox2d.common.Vec2} argPosition
         * @param {number} rotation
         * @param {number} localScale
         * @param {org.jbox2d.common.Vec2} argLocalOffset
         * @param {number} halfImageWidth
         * @param {number} halfImageHeight
         */
        drawImage(image: PImage, argPosition: org.jbox2d.common.Vec2, rotation: number, localScale: number, argLocalOffset: org.jbox2d.common.Vec2, halfImageWidth: number, halfImageHeight: number): void;
    }
}
declare namespace org.jbox2d.dynamics {
    /**
     * Implement this class to provide collision filtering. In other words, you can implement
     * this class if you want finer control over contact creation.
     * @class
     */
    interface ContactFilter {
        /**
         * Return true if contact calculations should be performed between these two shapes.
         * <BR><BR><em>Warning</em>: for performance reasons this is only called when the AABBs begin to overlap.
         * @param {org.jbox2d.collision.shapes.Shape} shape1
         * @param {org.jbox2d.collision.shapes.Shape} shape2
         * @return {boolean}
         */
        shouldCollide(shape1: org.jbox2d.collision.shapes.Shape, shape2: org.jbox2d.collision.shapes.Shape): boolean;
        /**
         * Return true if the given shape should be considered for ray intersection
         * @param {*} userData
         * @param {org.jbox2d.collision.shapes.Shape} shape
         * @return {boolean}
         */
        rayCollide(userData: any, shape: org.jbox2d.collision.shapes.Shape): boolean;
    }
    namespace ContactFilter {
        /**
         * Default contact filter, using groupIndex, maskBits and categoryBits as detailed
         * in Box2d manual.
         */
        let DEFAULT_FILTER: ContactFilter;
        function DEFAULT_FILTER_$LI$(): ContactFilter;
    }
}
declare namespace org.jbox2d.dynamics.joints {
    /**
     * A gear joint is used to connect two joints together. Either joint
     * can be a revolute or prismatic joint. You specify a gear ratio
     * to bind the motions together:
     * coordinate1 + ratio * coordinate2 = constant
     * The ratio can be negative or positive. If one joint is a revolute joint
     * and the other joint is a prismatic joint, then the ratio will have units
     * of length or units of 1/length.
     * <BR><em>Warning</em>: The revolute and prismatic joints must be attached to
     * fixed bodies (which must be body1 on those joints).
     * @param {org.jbox2d.dynamics.joints.GearJointDef} def
     * @class
     * @extends org.jbox2d.dynamics.joints.Joint
     */
    class GearJoint extends org.jbox2d.dynamics.joints.Joint {
        m_ground1: org.jbox2d.dynamics.Body;
        m_ground2: org.jbox2d.dynamics.Body;
        m_revolute1: org.jbox2d.dynamics.joints.RevoluteJoint;
        m_prismatic1: org.jbox2d.dynamics.joints.PrismaticJoint;
        m_revolute2: org.jbox2d.dynamics.joints.RevoluteJoint;
        m_prismatic2: org.jbox2d.dynamics.joints.PrismaticJoint;
        m_groundAnchor1: org.jbox2d.common.Vec2;
        m_groundAnchor2: org.jbox2d.common.Vec2;
        m_localAnchor1: org.jbox2d.common.Vec2;
        m_localAnchor2: org.jbox2d.common.Vec2;
        m_J: org.jbox2d.dynamics.joints.Jacobian;
        m_constant: number;
        m_ratio: number;
        /**
         * Effective mass
         */
        m_mass: number;
        /**
         * Force for accumulation/warm starting.
         */
        m_force: number;
        constructor(def: org.jbox2d.dynamics.joints.GearJointDef);
        tlug: org.jbox2d.pooling.TLVec2;
        tlr: org.jbox2d.pooling.TLVec2;
        /**
         *
         * @param {org.jbox2d.dynamics.TimeStep} step
         */
        initVelocityConstraints(step: org.jbox2d.dynamics.TimeStep): void;
        /**
         *
         * @param {org.jbox2d.dynamics.TimeStep} step
         */
        solveVelocityConstraints(step: org.jbox2d.dynamics.TimeStep): void;
        /**
         *
         * @return {boolean}
         */
        solvePositionConstraints(): boolean;
        /**
         *
         * @return {org.jbox2d.common.Vec2}
         */
        getAnchor1(): org.jbox2d.common.Vec2;
        getAnchor1ToOut(out: org.jbox2d.common.Vec2): void;
        /**
         *
         * @return {org.jbox2d.common.Vec2}
         */
        getAnchor2(): org.jbox2d.common.Vec2;
        getAnchor2ToOut(out: org.jbox2d.common.Vec2): void;
        /**
         *
         * @return {org.jbox2d.common.Vec2}
         */
        getReactionForce(): org.jbox2d.common.Vec2;
        getReactionForceToOut(out: org.jbox2d.common.Vec2): void;
        /**
         *
         * @return {number}
         */
        getReactionTorque(): number;
        getRatio(): number;
    }
}
declare namespace org.jbox2d.dynamics.joints {
    /**
     * Definition for a {@link ConstantVolumeJoint}, which connects a group a bodies together
     * so they maintain a constant volume within them.
     * @class
     * @extends org.jbox2d.dynamics.joints.JointDef
     */
    class ConstantVolumeJointDef extends org.jbox2d.dynamics.joints.JointDef {
        bodies: org.jbox2d.dynamics.Body[];
        frequencyHz: number;
        dampingRatio: number;
        constructor();
        addBody(b: org.jbox2d.dynamics.Body): void;
    }
}
declare namespace org.jbox2d.dynamics.joints {
    /**
     * Definition for a distance joint.  A distance joint
     * keeps two points on two bodies at a constant distance
     * from each other.
     * @class
     * @extends org.jbox2d.dynamics.joints.JointDef
     */
    class DistanceJointDef extends org.jbox2d.dynamics.joints.JointDef {
        /**
         * The local anchor point relative to body1's origin.
         */
        localAnchor1: org.jbox2d.common.Vec2;
        /**
         * The local anchor point relative to body2's origin.
         */
        localAnchor2: org.jbox2d.common.Vec2;
        /**
         * The equilibrium length between the anchor points.
         */
        length: number;
        frequencyHz: number;
        dampingRatio: number;
        constructor();
        /**
         * Initialize the bodies, anchors, and length using the world
         * anchors.
         * @param {org.jbox2d.dynamics.Body} b1 First body
         * @param {org.jbox2d.dynamics.Body} b2 Second body
         * @param {org.jbox2d.common.Vec2} anchor1 World anchor on first body
         * @param {org.jbox2d.common.Vec2} anchor2 World anchor on second body
         */
        initialize(b1: org.jbox2d.dynamics.Body, b2: org.jbox2d.dynamics.Body, anchor1: org.jbox2d.common.Vec2, anchor2: org.jbox2d.common.Vec2): void;
    }
}
declare namespace org.jbox2d.dynamics.joints {
    /**
     *
     * Gear joint definition. This definition requires two existing
     * revolute or prismatic joints (any combination will work).
     * The provided joints must attach a dynamic body to a static body.<BR><BR>
     * A gear joint is used to connect two joints together. Either joint
     * can be a revolute or prismatic joint. You specify a gear ratio
     * to bind the motions together:
     * coordinate1 + ratio * coordinate2 = constant
     * The ratio can be negative or positive. If one joint is a revolute joint
     * and the other joint is a prismatic joint, then the ratio will have units
     * of length or units of 1/length.
     * <BR><em>Warning</em>: The revolute and prismatic joints must be attached to
     * fixed bodies (which must be body1 on those joints).
     * @class
     * @extends org.jbox2d.dynamics.joints.JointDef
     */
    class GearJointDef extends org.jbox2d.dynamics.joints.JointDef {
        /**
         * The first revolute/prismatic joint attached to the gear joint.
         */
        joint1: org.jbox2d.dynamics.joints.Joint;
        /**
         * The second revolute/prismatic joint attached to the gear joint.
         */
        joint2: org.jbox2d.dynamics.joints.Joint;
        /**
         * The gear ratio.
         * @see org.jbox2d.dynamics.joints.GearJoint for explanation.
         */
        ratio: number;
        constructor();
    }
}
declare namespace org.jbox2d.dynamics.joints {
    /**
     * This joint is used to add force to an object to accelerate to a position,
     * normally used with the mouse.
     * @class
     * @extends org.jbox2d.dynamics.joints.JointDef
     */
    class MouseJointDef extends org.jbox2d.dynamics.joints.JointDef {
        /**
         * The initial world target point. This is assumed
         * to coincide with the body anchor initially.
         */
        target: org.jbox2d.common.Vec2;
        /**
         * The maximum constraint force that can be exerted
         * to move the candidate body. Usually you will express
         * as some multiple of the weight (multiplier * mass * gravity).
         */
        maxForce: number;
        /**
         * The response speed.
         */
        frequencyHz: number;
        /**
         * The damping ratio. 0 = no damping, 1 = critical damping.
         */
        dampingRatio: number;
        /**
         * The time step used in the simulation.
         */
        timeStep: number;
        constructor();
    }
}
declare namespace org.jbox2d.dynamics.joints {
    /**
     * A prismatic joint definition. This joint provides one degree of freedom: translation
     * along an axis fixed in body1. Relative rotation is prevented. You can
     * use a joint limit to restrict the range of motion and a joint motor to
     * drive the motion or to model joint friction.
     * @class
     * @extends org.jbox2d.dynamics.joints.JointDef
     */
    class PrismaticJointDef extends org.jbox2d.dynamics.joints.JointDef {
        constructor();
        initialize(b1: org.jbox2d.dynamics.Body, b2: org.jbox2d.dynamics.Body, anchor: org.jbox2d.common.Vec2, axis: org.jbox2d.common.Vec2): void;
        localAnchor1: org.jbox2d.common.Vec2;
        localAnchor2: org.jbox2d.common.Vec2;
        localAxis1: org.jbox2d.common.Vec2;
        referenceAngle: number;
        enableLimit: boolean;
        lowerTranslation: number;
        upperTranslation: number;
        enableMotor: boolean;
        maxMotorForce: number;
        motorSpeed: number;
    }
}
declare namespace org.jbox2d.dynamics.joints {
    /**
     * The pulley joint is connected to two bodies and two fixed ground points.
     * The pulley supports a ratio such that:
     * length1 + ratio * length2 = constant
     * Yes, the force transmitted is scaled by the ratio.
     * The pulley also enforces a maximum length limit on both sides. This is
     * useful to prevent one side of the pulley hitting the top.
     * @class
     * @extends org.jbox2d.dynamics.joints.JointDef
     */
    class PulleyJointDef extends org.jbox2d.dynamics.joints.JointDef {
        constructor();
        initialize(b1: org.jbox2d.dynamics.Body, b2: org.jbox2d.dynamics.Body, ga1: org.jbox2d.common.Vec2, ga2: org.jbox2d.common.Vec2, anchor1: org.jbox2d.common.Vec2, anchor2: org.jbox2d.common.Vec2, r: number): void;
        groundAnchor1: org.jbox2d.common.Vec2;
        groundAnchor2: org.jbox2d.common.Vec2;
        localAnchor1: org.jbox2d.common.Vec2;
        localAnchor2: org.jbox2d.common.Vec2;
        length1: number;
        maxLength1: number;
        length2: number;
        maxLength2: number;
        ratio: number;
    }
}
declare namespace org.jbox2d.dynamics.joints {
    /**
     * Revolute joint definition. This requires defining an
     * anchor point where the bodies are joined. The definition
     * uses local anchor points so that the initial configuration
     * can violate the constraint slightly. You also need to
     * specify the initial relative angle for joint limits. This
     * helps when saving and loading a game.
     * The local anchor points are measured from the body's origin
     * rather than the center of mass because:<br/>
     * <ul>
     * <li>you might not know where the center of mass will be.</li>
     * <li>if you add/remove shapes from a body and recompute the mass,
     * the joints will be broken.</li></ul>
     * @class
     * @extends org.jbox2d.dynamics.joints.JointDef
     */
    class RevoluteJointDef extends org.jbox2d.dynamics.joints.JointDef {
        constructor();
        initialize(b1: org.jbox2d.dynamics.Body, b2: org.jbox2d.dynamics.Body, anchor: org.jbox2d.common.Vec2): void;
        /**
         * The local anchor point relative to body1's origin.
         */
        localAnchor1: org.jbox2d.common.Vec2;
        /**
         * The local anchor point relative to body2's origin.
         */
        localAnchor2: org.jbox2d.common.Vec2;
        /**
         * The body2 angle minus body1 angle in the reference state (radians).
         */
        referenceAngle: number;
        /**
         * A flag to enable joint limits.
         */
        enableLimit: boolean;
        /**
         * The lower angle for the joint limit (radians).
         */
        lowerAngle: number;
        /**
         * The upper angle for the joint limit (radians).
         */
        upperAngle: number;
        /**
         * A flag to enable the joint motor.
         */
        enableMotor: boolean;
        /**
         * The desired motor speed. Usually in radians per second.
         */
        motorSpeed: number;
        /**
         * The maximum motor torque used to achieve the desired motor speed.
         * Usually in N-m.
         */
        maxMotorTorque: number;
    }
}
declare namespace org.jbox2d.pooling.arrays {
    class BooleanArray extends org.jbox2d.pooling.arrays.DynamicTLArray<boolean> {
        /**
         *
         * @param {number} argLength
         * @return {Array}
         */
        getInitializedArray(argLength: number): boolean[];
    }
}
declare namespace org.jbox2d.pooling.arrays {
    class IntegerArray extends org.jbox2d.pooling.arrays.DynamicTLArray<number> {
        /**
         *
         * @param {number} argLength
         * @return {Array}
         */
        getInitializedArray(argLength: number): number[];
    }
}
declare namespace org.jbox2d.pooling.arrays {
    class Vec2Array extends org.jbox2d.pooling.arrays.DynamicTLArray<org.jbox2d.common.Vec2> {
        /**
         *
         * @param {number} argLength
         * @return {Array}
         */
        getInitializedArray(argLength: number): org.jbox2d.common.Vec2[];
    }
}
declare namespace org.jbox2d.pooling.stacks {
    class ContactSolverStack extends org.jbox2d.pooling.stacks.DynamicTLStack<org.jbox2d.dynamics.contacts.ContactSolver> {
        /**
         *
         * @return {org.jbox2d.dynamics.contacts.ContactSolver}
         */
        newObjectInstance(): org.jbox2d.dynamics.contacts.ContactSolver;
    }
}
declare namespace org.jbox2d.pooling.stacks {
    class IslandStack extends org.jbox2d.pooling.stacks.DynamicTLStack<org.jbox2d.dynamics.Island> {
        /**
         *
         * @return {org.jbox2d.dynamics.Island}
         */
        newObjectInstance(): org.jbox2d.dynamics.Island;
    }
}
declare namespace org.jbox2d.pooling.stacks {
    class TimeStepStack extends org.jbox2d.pooling.stacks.DynamicTLStack<org.jbox2d.dynamics.TimeStep> {
        /**
         *
         * @return {org.jbox2d.dynamics.TimeStep}
         */
        newObjectInstance(): org.jbox2d.dynamics.TimeStep;
    }
}
declare namespace org.jbox2d.collision.shapes {
    /**
     * A shape is used for collision detection. Shapes are created in World.
     * You can use shape for collision detection before they are attached to the world.
     * <BR><BR><em>Warning</em>: you cannot reuse shapes on different bodies, they must
     * be re-created or copied.
     * @param {org.jbox2d.collision.shapes.ShapeDef} def
     * @class
     */
    abstract class Shape {
        /**
         * Unique id for shape for sorting (C++ version uses memory address)
         */
        uid: number;
        /**
         * Used to generate uids - not initialized on applet reload,
         * but that's okay since these just have to be unique.
         */
        static uidcount: number;
        m_type: org.jbox2d.collision.shapes.ShapeType;
        m_next: Shape;
        m_body: org.jbox2d.dynamics.Body;
        /**
         * Sweep radius relative to the parent body's center of mass.
         */
        m_sweepRadius: number;
        m_density: number;
        m_friction: number;
        m_restitution: number;
        m_proxyId: number;
        m_filter: org.jbox2d.collision.FilterData;
        m_isSensor: boolean;
        m_userData: any;
        constructor(def: org.jbox2d.collision.shapes.ShapeDef);
        /**
         * Get the coefficient of friction.
         * @return {number}
         */
        getFriction(): number;
        /**
         * Set the coefficient of friction.
         * @param {number} friction
         */
        setFriction(friction: number): void;
        /**
         * Get the coefficient of restitution.
         * @return {number}
         */
        getRestitution(): number;
        /**
         * Set the coefficient of restitution.
         * @param {number} restitution
         */
        setRestitution(restitution: number): void;
        /**
         * Set the collision filtering data.
         * @param {org.jbox2d.collision.FilterData} filter
         */
        setFilterData(filter: org.jbox2d.collision.FilterData): void;
        /**
         * Get the collision filtering data.
         * @return {org.jbox2d.collision.FilterData}
         */
        getFilterData(): org.jbox2d.collision.FilterData;
        /**
         * Get the type of this shape. You can use this to down cast to the concrete shape.
         * @return {org.jbox2d.collision.shapes.ShapeType} the shape type.
         */
        getType(): org.jbox2d.collision.shapes.ShapeType;
        /**
         * Is this shape a sensor (non-solid)?
         * @return {boolean} the true if the shape is a sensor.
         */
        isSensor(): boolean;
        /**
         * Get the user data that was assigned in the shape definition. Use this to
         * store your application specific data.
         * @return {*}
         */
        getUserData(): any;
        /**
         * Set the user data associated with the object.
         * @param {*} o User data to set
         */
        setUserData(o: any): void;
        /**
         * Get the parent body of this shape. This is NULL if the shape is not attached.
         * @return {org.jbox2d.dynamics.Body} the parent body.
         */
        getBody(): org.jbox2d.dynamics.Body;
        /**
         * Get the next shape in the parent body's shape list.
         * @return {org.jbox2d.collision.shapes.Shape} the next shape.
         */
        getNext(): Shape;
        /**
         * Get the sweep radius of the shape.
         * @return {number} the sweep radius
         */
        getSweepRadius(): number;
        /**
         * Test a point for containment in this shape. This only works for convex shapes.
         * @param {org.jbox2d.common.XForm} xf the shape world transform.
         * @param {org.jbox2d.common.Vec2} p a point in world coordinates.
         * @return {boolean} true if the point is within the shape
         */
        abstract testPoint(xf: org.jbox2d.common.XForm, p: org.jbox2d.common.Vec2): boolean;
        /**
         * Perform a ray cast against this shape.
         * @param {org.jbox2d.common.XForm} xf the shape world transform.
         * @param {org.jbox2d.common.RaycastResult} out is where the results are placed: <ul><li>lambda returns the hit fraction, based on
         * the distance between the two points. You can use this to compute the contact point
         * p = (1 - lambda) * segment.p1 + lambda * segment.p2.</li>
         * <li>normal returns the normal at the contact point. If there is no intersection, the normal
         * is not set.</li></ul>
         * @param {org.jbox2d.collision.Segment} segment defines the begin and end point of the ray cast.
         * @param {number} maxLambda a number typically in the range [0,1].
         * @return {org.jbox2d.collision.SegmentCollide} true if there was an intersection.
         */
        abstract testSegment(xf: org.jbox2d.common.XForm, out: org.jbox2d.common.RaycastResult, segment: org.jbox2d.collision.Segment, maxLambda: number): org.jbox2d.collision.SegmentCollide;
        /**
         * Given a transform, compute the associated axis aligned bounding box for this shape.
         * @param {org.jbox2d.collision.AABB} aabb returns the axis aligned box.
         * @param {org.jbox2d.common.XForm} xf the world transform of the shape.
         */
        abstract computeAABB(aabb: org.jbox2d.collision.AABB, xf: org.jbox2d.common.XForm): any;
        /**
         * Given two transforms, compute the associated swept axis aligned bounding box for this shape.
         * @param {org.jbox2d.collision.AABB} aabb returns the axis aligned box. (return parameter)
         * @param {org.jbox2d.common.XForm} xf1 the starting shape world transform.
         * @param {org.jbox2d.common.XForm} xf2 the ending shape world transform.
         */
        abstract computeSweptAABB(aabb: org.jbox2d.collision.AABB, xf1: org.jbox2d.common.XForm, xf2: org.jbox2d.common.XForm): any;
        /**
         * Compute the mass properties of this shape using its dimensions and density.
         * The inertia tensor is computed about the local origin, not the centroid.
         * @param {org.jbox2d.collision.MassData} massData returns the mass data for this shape. (return parameter)
         */
        abstract computeMass(massData: org.jbox2d.collision.MassData): any;
        /**
         * Internal
         * @param {org.jbox2d.common.Vec2} center
         */
        abstract updateSweepRadius(center: org.jbox2d.common.Vec2): any;
        static tlAabb: org.jbox2d.pooling.TLAABB;
        static tlAabb_$LI$(): org.jbox2d.pooling.TLAABB;
        /**
         * Internal
         * @param {org.jbox2d.collision.BroadPhase} broadPhase
         * @param {org.jbox2d.common.XForm} transform1
         * @param {org.jbox2d.common.XForm} transform2
         * @return {boolean}
         */
        synchronize(broadPhase: org.jbox2d.collision.BroadPhase, transform1: org.jbox2d.common.XForm, transform2: org.jbox2d.common.XForm): boolean;
        /**
         * Internal
         * @param {org.jbox2d.collision.BroadPhase} broadPhase
         * @param {org.jbox2d.common.XForm} transform
         */
        refilterProxy(broadPhase: org.jbox2d.collision.BroadPhase, transform: org.jbox2d.common.XForm): void;
        /**
         * Internal
         * @param {org.jbox2d.collision.shapes.ShapeDef} def
         * @return {org.jbox2d.collision.shapes.Shape}
         */
        static create(def: org.jbox2d.collision.shapes.ShapeDef): Shape;
        /**
         * Internal
         * @param {org.jbox2d.collision.shapes.Shape} s
         */
        static destroy(s: Shape): void;
        /**
         * Internal
         */
        destructor(): void;
        /**
         * Internal
         * @param {org.jbox2d.collision.BroadPhase} broadPhase
         * @param {org.jbox2d.common.XForm} transform
         */
        createProxy(broadPhase: org.jbox2d.collision.BroadPhase, transform: org.jbox2d.common.XForm): void;
        /**
         * Internal
         * @param {org.jbox2d.collision.BroadPhase} broadPhase
         */
        destroyProxy(broadPhase: org.jbox2d.collision.BroadPhase): void;
        computeSubmergedArea$org_jbox2d_common_Vec2$float$org_jbox2d_common_Vec2(normal: org.jbox2d.common.Vec2, offset: number, c: org.jbox2d.common.Vec2): number;
        computeSubmergedArea$org_jbox2d_common_Vec2$float$org_jbox2d_common_XForm$org_jbox2d_common_Vec2(normal: org.jbox2d.common.Vec2, offset: number, form: org.jbox2d.common.XForm, c: org.jbox2d.common.Vec2): number;
        /**
         * @param {org.jbox2d.common.Vec2} normal
         * @param {number} offset
         * @param {org.jbox2d.common.XForm} form
         * @param {org.jbox2d.common.Vec2} c
         * @return
         * @return {number}
         */
        computeSubmergedArea(normal?: any, offset?: any, form?: any, c?: any): any;
        /**
         * @return {number} shape density
         */
        getDensity(): number;
        /**
         * @return {org.jbox2d.collision.shapes.Shape[]} a Set<Shape> of all shapes in contact with this one
         */
        getShapesInContact(): Array<Shape>;
        /**
         * @return {org.jbox2d.dynamics.contacts.Contact[]} a Set<Contact> of all (active) contacts involving this shape
         */
        getContacts(): Array<org.jbox2d.dynamics.contacts.Contact>;
    }
}
declare namespace org.jbox2d.common {
    class OBBViewportTransform implements org.jbox2d.common.IViewportTransform {
        box: org.jbox2d.collision.OBB;
        yFlip: boolean;
        yFlipMat: org.jbox2d.common.Mat22;
        yFlipMatInv: org.jbox2d.common.Mat22;
        constructor();
        set(vpt: OBBViewportTransform): void;
        /**
         * @see IViewportTransform#setCamera(float, float, float)
         * @param {number} x
         * @param {number} y
         * @param {number} scale
         */
        setCamera(x: number, y: number, scale: number): void;
        /**
         * @see IViewportTransform#getExtents()
         * @return {org.jbox2d.common.Vec2}
         */
        getExtents(): org.jbox2d.common.Vec2;
        setExtents$org_jbox2d_common_Vec2(argExtents: org.jbox2d.common.Vec2): void;
        setExtents$float$float(argHalfWidth: number, argHalfHeight: number): void;
        /**
         * @see IViewportTransform#setExtents(float, float)
         * @param {number} argHalfWidth
         * @param {number} argHalfHeight
         */
        setExtents(argHalfWidth?: any, argHalfHeight?: any): any;
        /**
         * @see IViewportTransform#getCenter()
         * @return {org.jbox2d.common.Vec2}
         */
        getCenter(): org.jbox2d.common.Vec2;
        setCenter$org_jbox2d_common_Vec2(argPos: org.jbox2d.common.Vec2): void;
        setCenter$float$float(x: number, y: number): void;
        /**
         * @see IViewportTransform#setCenter(float, float)
         * @param {number} x
         * @param {number} y
         */
        setCenter(x?: any, y?: any): any;
        /**
         * gets the transform of the viewport, transforms around the center.
         * Not a copy.
         * @return
         * @return {org.jbox2d.common.Mat22}
         */
        getTransform(): org.jbox2d.common.Mat22;
        /**
         * Sets the transform of the viewport.  Transforms about the center.
         * @param {org.jbox2d.common.Mat22} transform
         */
        setTransform(transform: org.jbox2d.common.Mat22): void;
        /**
         * Lerps to the given transform
         * @param {org.jbox2d.common.Mat22} transform
         * @param {number} amt
         */
        lerpTransform(transform: org.jbox2d.common.Mat22, amt: number): void;
        /**
         * Multiplies the obb transform by the given transform
         * @param {org.jbox2d.common.Mat22} argTransform
         */
        mulByTransform(argTransform: org.jbox2d.common.Mat22): void;
        /**
         * @see IViewportTransform#isYFlip()
         * @return {boolean}
         */
        isYFlip(): boolean;
        /**
         * @see IViewportTransform#setYFlip(boolean)
         * @param {boolean} yFlip
         */
        setYFlip(yFlip: boolean): void;
        static tlInv: org.jbox2d.pooling.TLMat22;
        static tlInv_$LI$(): org.jbox2d.pooling.TLMat22;
        /**
         * @see IViewportTransform#vectorInverseTransform(Vec2, Vec2)
         * @param {org.jbox2d.common.Vec2} argScreen
         * @param {org.jbox2d.common.Vec2} argWorld
         */
        vectorInverseTransform(argScreen: org.jbox2d.common.Vec2, argWorld: org.jbox2d.common.Vec2): void;
        /**
         * @see IViewportTransform#vectorTransform(Vec2, Vec2)
         * @param {org.jbox2d.common.Vec2} argWorld
         * @param {org.jbox2d.common.Vec2} argScreen
         */
        vectorTransform(argWorld: org.jbox2d.common.Vec2, argScreen: org.jbox2d.common.Vec2): void;
        /**
         * @see IViewportTransform#getWorldToScreen(Vec2, Vec2)
         * @param {org.jbox2d.common.Vec2} argWorld
         * @param {org.jbox2d.common.Vec2} argScreen
         */
        getWorldToScreen(argWorld: org.jbox2d.common.Vec2, argScreen: org.jbox2d.common.Vec2): void;
        /**
         * @see IViewportTransform#getScreenToWorld(Vec2, Vec2)
         * @param {org.jbox2d.common.Vec2} argScreen
         * @param {org.jbox2d.common.Vec2} argWorld
         */
        getScreenToWorld(argScreen: org.jbox2d.common.Vec2, argWorld: org.jbox2d.common.Vec2): void;
    }
}
declare namespace org.jbox2d.collision {
    class Segment {
        toString(): string;
        /**
         * The starting point
         */
        p1: org.jbox2d.common.Vec2;
        /**
         * The ending point
         */
        p2: org.jbox2d.common.Vec2;
        static tlR: org.jbox2d.pooling.TLVec2;
        static tlR_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlD: org.jbox2d.pooling.TLVec2;
        static tlD_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlN: org.jbox2d.pooling.TLVec2;
        static tlN_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlB: org.jbox2d.pooling.TLVec2;
        static tlB_$LI$(): org.jbox2d.pooling.TLVec2;
        testSegment(out: org.jbox2d.common.RaycastResult, segment: Segment, maxLambda: number): boolean;
    }
}
declare namespace org.jbox2d.dynamics {
    /**
     * Delegate of World - for internal use.
     * @class
     */
    class ContactManager implements org.jbox2d.collision.PairCallback {
        m_world: org.jbox2d.dynamics.World;
        m_nullContact: org.jbox2d.dynamics.contacts.NullContact;
        m_destroyImmediate: boolean;
        constructor();
        pairAdded(proxyUserData1: any, proxyUserData2: any): any;
        pairRemoved(proxyUserData1: any, proxyUserData2: any, pairUserData: any): void;
        static tlV1: org.jbox2d.pooling.TLVec2;
        static tlV1_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlCp: org.jbox2d.pooling.TLContactPoint;
        static tlCp_$LI$(): org.jbox2d.pooling.TLContactPoint;
        destroy(c: org.jbox2d.dynamics.contacts.Contact): void;
        collide(): void;
    }
}
declare namespace org.jbox2d.dynamics.contacts {
    class CircleContact extends org.jbox2d.dynamics.contacts.Contact implements org.jbox2d.dynamics.contacts.ContactCreateFcn {
        m_manifold: org.jbox2d.collision.Manifold;
        manifoldList: Array<org.jbox2d.collision.Manifold>;
        create(shape1: org.jbox2d.collision.shapes.Shape, shape2: org.jbox2d.collision.shapes.Shape): org.jbox2d.dynamics.contacts.Contact;
        static Destroy(contact: org.jbox2d.dynamics.contacts.Contact): void;
        /**
         *
         * @return {org.jbox2d.dynamics.contacts.CircleContact}
         */
        clone(): CircleContact;
        constructor(shape1?: any, shape2?: any);
        destructor(): void;
        static tlm0: org.jbox2d.pooling.TLManifold;
        static tlm0_$LI$(): org.jbox2d.pooling.TLManifold;
        static tlV1: org.jbox2d.pooling.TLVec2;
        static tlV1_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlCp: org.jbox2d.pooling.TLContactPoint;
        static tlCp_$LI$(): org.jbox2d.pooling.TLContactPoint;
        /**
         *
         * @param {*} listener
         */
        evaluate(listener: org.jbox2d.dynamics.ContactListener): void;
        /**
         *
         * @return {org.jbox2d.collision.Manifold[]}
         */
        getManifolds(): Array<org.jbox2d.collision.Manifold>;
    }
}
declare namespace org.jbox2d.dynamics.contacts {
    class EdgeAndCircleContact extends org.jbox2d.dynamics.contacts.Contact implements org.jbox2d.dynamics.contacts.ContactCreateFcn {
        m_manifold: org.jbox2d.collision.Manifold;
        manifoldList: Array<org.jbox2d.collision.Manifold>;
        constructor(s1?: any, s2?: any);
        /**
         *
         * @return {org.jbox2d.dynamics.contacts.Contact}
         */
        clone(): org.jbox2d.dynamics.contacts.Contact;
        static Destroy(contact: org.jbox2d.dynamics.contacts.Contact): void;
        destructor(): void;
        static tlm0: org.jbox2d.pooling.TLManifold;
        static tlm0_$LI$(): org.jbox2d.pooling.TLManifold;
        static tlV1: org.jbox2d.pooling.TLVec2;
        static tlV1_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlCp: org.jbox2d.pooling.TLContactPoint;
        static tlCp_$LI$(): org.jbox2d.pooling.TLContactPoint;
        /**
         *
         * @param {*} listener
         */
        evaluate(listener: org.jbox2d.dynamics.ContactListener): void;
        /**
         *
         * @return {org.jbox2d.collision.Manifold[]}
         */
        getManifolds(): Array<org.jbox2d.collision.Manifold>;
        create(s1: org.jbox2d.collision.shapes.Shape, s2: org.jbox2d.collision.shapes.Shape): org.jbox2d.dynamics.contacts.Contact;
    }
}
declare namespace org.jbox2d.dynamics.contacts {
    class PointAndCircleContact extends org.jbox2d.dynamics.contacts.Contact implements org.jbox2d.dynamics.contacts.ContactCreateFcn {
        m_manifold: org.jbox2d.collision.Manifold;
        manifoldList: Array<org.jbox2d.collision.Manifold>;
        create(shape1: org.jbox2d.collision.shapes.Shape, shape2: org.jbox2d.collision.shapes.Shape): org.jbox2d.dynamics.contacts.Contact;
        static Destroy(contact: org.jbox2d.dynamics.contacts.Contact): void;
        /**
         *
         * @return {org.jbox2d.dynamics.contacts.PointAndCircleContact}
         */
        clone(): PointAndCircleContact;
        constructor(shape1?: any, shape2?: any);
        destructor(): void;
        static tlm0: org.jbox2d.pooling.TLManifold;
        static tlm0_$LI$(): org.jbox2d.pooling.TLManifold;
        static tlV1: org.jbox2d.pooling.TLVec2;
        static tlV1_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlCp: org.jbox2d.pooling.TLContactPoint;
        static tlCp_$LI$(): org.jbox2d.pooling.TLContactPoint;
        /**
         *
         * @param {*} listener
         */
        evaluate(listener: org.jbox2d.dynamics.ContactListener): void;
        /**
         *
         * @return {org.jbox2d.collision.Manifold[]}
         */
        getManifolds(): Array<org.jbox2d.collision.Manifold>;
    }
}
declare namespace org.jbox2d.dynamics.contacts {
    class PolyContact extends org.jbox2d.dynamics.contacts.Contact implements org.jbox2d.dynamics.contacts.ContactCreateFcn {
        m_manifold: org.jbox2d.collision.Manifold;
        manifoldList: Array<org.jbox2d.collision.Manifold>;
        constructor(s1?: any, s2?: any);
        /**
         *
         * @return {org.jbox2d.dynamics.contacts.Contact}
         */
        clone(): org.jbox2d.dynamics.contacts.Contact;
        /**
         *
         * @return {org.jbox2d.collision.Manifold[]}
         */
        getManifolds(): Array<org.jbox2d.collision.Manifold>;
        create(shape1: org.jbox2d.collision.shapes.Shape, shape2: org.jbox2d.collision.shapes.Shape): org.jbox2d.dynamics.contacts.Contact;
        dumpManifoldPoints(): void;
        static tlm0: org.jbox2d.pooling.TLManifold;
        static tlm0_$LI$(): org.jbox2d.pooling.TLManifold;
        static tlV1: org.jbox2d.pooling.TLVec2;
        static tlV1_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlCp: org.jbox2d.pooling.TLContactPoint;
        static tlCp_$LI$(): org.jbox2d.pooling.TLContactPoint;
        /**
         *
         * @param {*} listener
         */
        evaluate(listener: org.jbox2d.dynamics.ContactListener): void;
    }
}
declare namespace org.jbox2d.dynamics.joints {
    class DistanceJoint extends org.jbox2d.dynamics.joints.Joint {
        m_localAnchor1: org.jbox2d.common.Vec2;
        m_localAnchor2: org.jbox2d.common.Vec2;
        m_u: org.jbox2d.common.Vec2;
        m_impulse: number;
        m_mass: number;
        m_length: number;
        m_frequencyHz: number;
        m_dampingRatio: number;
        m_gamma: number;
        m_bias: number;
        constructor(def: org.jbox2d.dynamics.joints.DistanceJointDef);
        setFrequency(hz: number): void;
        getFrequency(): number;
        setDampingRatio(damp: number): void;
        getDampingRatio(): number;
        /**
         *
         * @return {org.jbox2d.common.Vec2}
         */
        getAnchor1(): org.jbox2d.common.Vec2;
        /**
         *
         * @return {org.jbox2d.common.Vec2}
         */
        getAnchor2(): org.jbox2d.common.Vec2;
        static tlReactionForce: org.jbox2d.pooling.TLVec2;
        static tlReactionForce_$LI$(): org.jbox2d.pooling.TLVec2;
        /**
         *
         * @return {org.jbox2d.common.Vec2}
         */
        getReactionForce(): org.jbox2d.common.Vec2;
        /**
         *
         * @return {number}
         */
        getReactionTorque(): number;
        static tlr1: org.jbox2d.pooling.TLVec2;
        static tlr1_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlr2: org.jbox2d.pooling.TLVec2;
        static tlr2_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlP: org.jbox2d.pooling.TLVec2;
        static tlP_$LI$(): org.jbox2d.pooling.TLVec2;
        /**
         *
         * @param {org.jbox2d.dynamics.TimeStep} step
         */
        initVelocityConstraints(step: org.jbox2d.dynamics.TimeStep): void;
        static tld: org.jbox2d.pooling.TLVec2;
        static tld_$LI$(): org.jbox2d.pooling.TLVec2;
        /**
         *
         * @return {boolean}
         */
        solvePositionConstraints(): boolean;
        static tlv1: org.jbox2d.pooling.TLVec2;
        static tlv1_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlv2: org.jbox2d.pooling.TLVec2;
        static tlv2_$LI$(): org.jbox2d.pooling.TLVec2;
        /**
         *
         * @param {org.jbox2d.dynamics.TimeStep} step
         */
        solveVelocityConstraints(step: org.jbox2d.dynamics.TimeStep): void;
    }
}
declare namespace org.jbox2d.dynamics.joints {
    class MouseJoint extends org.jbox2d.dynamics.joints.Joint {
        m_localAnchor: org.jbox2d.common.Vec2;
        m_target: org.jbox2d.common.Vec2;
        m_force: org.jbox2d.common.Vec2;
        m_mass: org.jbox2d.common.Mat22;
        m_C: org.jbox2d.common.Vec2;
        m_maxForce: number;
        m_beta: number;
        m_gamma: number;
        constructor(def: org.jbox2d.dynamics.joints.MouseJointDef);
        /**
         * Use this to update the target point.
         * @param {org.jbox2d.common.Vec2} target
         */
        setTarget(target: org.jbox2d.common.Vec2): void;
        /**
         *
         * @return {org.jbox2d.common.Vec2}
         */
        getAnchor1(): org.jbox2d.common.Vec2;
        static tlanchor2: org.jbox2d.pooling.TLVec2;
        static tlanchor2_$LI$(): org.jbox2d.pooling.TLVec2;
        /**
         * this comes from a pooled value
         * @return {org.jbox2d.common.Vec2}
         */
        getAnchor2(): org.jbox2d.common.Vec2;
        static tlr: org.jbox2d.pooling.TLVec2;
        static tlr_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlK1: org.jbox2d.pooling.TLMat22;
        static tlK1_$LI$(): org.jbox2d.pooling.TLMat22;
        static tlK2: org.jbox2d.pooling.TLMat22;
        static tlK2_$LI$(): org.jbox2d.pooling.TLMat22;
        /**
         *
         * @param {org.jbox2d.dynamics.TimeStep} step
         */
        initVelocityConstraints(step: org.jbox2d.dynamics.TimeStep): void;
        /**
         *
         * @return {boolean}
         */
        solvePositionConstraints(): boolean;
        static tlCdot: org.jbox2d.pooling.TLVec2;
        static tlCdot_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlforce: org.jbox2d.pooling.TLVec2;
        static tlforce_$LI$(): org.jbox2d.pooling.TLVec2;
        static tloldForce: org.jbox2d.pooling.TLVec2;
        static tloldForce_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlP: org.jbox2d.pooling.TLVec2;
        static tlP_$LI$(): org.jbox2d.pooling.TLVec2;
        /**
         *
         * @param {org.jbox2d.dynamics.TimeStep} step
         */
        solveVelocityConstraints(step: org.jbox2d.dynamics.TimeStep): void;
        /**
         *
         * @return {org.jbox2d.common.Vec2}
         */
        getReactionForce(): org.jbox2d.common.Vec2;
        /**
         *
         * @return {number}
         */
        getReactionTorque(): number;
    }
}
declare namespace org.jbox2d.dynamics.joints {
    /**
     * A prismatic joint. This joint provides one degree of freedom: translation
     * along an axis fixed in body1. Relative rotation is prevented. You can
     * use a joint limit to restrict the range of motion and a joint motor to
     * drive the motion or to model joint friction.
     * @param {org.jbox2d.dynamics.joints.PrismaticJointDef} def
     * @class
     * @extends org.jbox2d.dynamics.joints.Joint
     */
    class PrismaticJoint extends org.jbox2d.dynamics.joints.Joint {
        m_localAnchor1: org.jbox2d.common.Vec2;
        m_localAnchor2: org.jbox2d.common.Vec2;
        m_localXAxis1: org.jbox2d.common.Vec2;
        m_localYAxis1: org.jbox2d.common.Vec2;
        m_refAngle: number;
        m_linearJacobian: org.jbox2d.dynamics.joints.Jacobian;
        m_linearMass: number;
        m_force: number;
        m_angularMass: number;
        m_torque: number;
        m_motorJacobian: org.jbox2d.dynamics.joints.Jacobian;
        m_motorMass: number;
        m_motorForce: number;
        m_limitForce: number;
        m_limitPositionImpulse: number;
        m_lowerTranslation: number;
        m_upperTranslation: number;
        m_maxMotorForce: number;
        m_motorSpeed: number;
        m_enableLimit: boolean;
        m_enableMotor: boolean;
        m_limitState: org.jbox2d.dynamics.joints.LimitState;
        constructor(def: org.jbox2d.dynamics.joints.PrismaticJointDef);
        static tlr1: org.jbox2d.pooling.TLVec2;
        static tlr1_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlr2: org.jbox2d.pooling.TLVec2;
        static tlr2_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlax1: org.jbox2d.pooling.TLVec2;
        static tlax1_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlay1: org.jbox2d.pooling.TLVec2;
        static tlay1_$LI$(): org.jbox2d.pooling.TLVec2;
        static tle: org.jbox2d.pooling.TLVec2;
        static tle_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlax1Neg: org.jbox2d.pooling.TLVec2;
        static tlax1Neg_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlay1Neg: org.jbox2d.pooling.TLVec2;
        static tlay1Neg_$LI$(): org.jbox2d.pooling.TLVec2;
        static tld: org.jbox2d.pooling.TLVec2;
        static tld_$LI$(): org.jbox2d.pooling.TLVec2;
        /**
         *
         * @param {org.jbox2d.dynamics.TimeStep} step
         */
        initVelocityConstraints(step: org.jbox2d.dynamics.TimeStep): void;
        m_lastWarmStartingForce: number;
        m_lastWarmStartingTorque: number;
        /**
         *
         * @param {org.jbox2d.dynamics.TimeStep} step
         */
        solveVelocityConstraints(step: org.jbox2d.dynamics.TimeStep): void;
        static tltemp: org.jbox2d.pooling.TLVec2;
        static tltemp_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlp1: org.jbox2d.pooling.TLVec2;
        static tlp1_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlp2: org.jbox2d.pooling.TLVec2;
        static tlp2_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlr1z: org.jbox2d.pooling.TLVec2;
        static tlr1z_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlr2z: org.jbox2d.pooling.TLVec2;
        static tlr2z_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlp1z: org.jbox2d.pooling.TLVec2;
        static tlp1z_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlp2z: org.jbox2d.pooling.TLVec2;
        static tlp2z_$LI$(): org.jbox2d.pooling.TLVec2;
        static tldz: org.jbox2d.pooling.TLVec2;
        static tldz_$LI$(): org.jbox2d.pooling.TLVec2;
        /**
         *
         * @return {boolean}
         */
        solvePositionConstraints(): boolean;
        /**
         *
         * @return {org.jbox2d.common.Vec2}
         */
        getAnchor1(): org.jbox2d.common.Vec2;
        getAnchor1ToOut(out: org.jbox2d.common.Vec2): void;
        /**
         *
         * @return {org.jbox2d.common.Vec2}
         */
        getAnchor2(): org.jbox2d.common.Vec2;
        getAnchor2ToOut(out: org.jbox2d.common.Vec2): void;
        static tlaxis: org.jbox2d.pooling.TLVec2;
        static tlaxis_$LI$(): org.jbox2d.pooling.TLVec2;
        getJointTranslation(): number;
        static tlw1xAxis: org.jbox2d.pooling.TLVec2;
        static tlw1xAxis_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlv22: org.jbox2d.pooling.TLVec2;
        static tlv22_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlw2xR2: org.jbox2d.pooling.TLVec2;
        static tlw2xR2_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlw1xR1: org.jbox2d.pooling.TLVec2;
        static tlw1xR1_$LI$(): org.jbox2d.pooling.TLVec2;
        getJointSpeed(): number;
        /**
         *
         * @return {number}
         */
        getReactionTorque(): number;
        /**
         *
         * @return {org.jbox2d.common.Vec2}
         */
        getReactionForce(): org.jbox2d.common.Vec2;
        static tlreactionAx1: org.jbox2d.pooling.TLVec2;
        static tlreactionAx1_$LI$(): org.jbox2d.pooling.TLVec2;
        getReactionForceToOut(out: org.jbox2d.common.Vec2): void;
        /**
         * Is the joint limit enabled?
         * @return {boolean}
         */
        isLimitEnabled(): boolean;
        /**
         * Enable/disable the joint limit.
         * @param {boolean} flag
         */
        enableLimit(flag: boolean): void;
        /**
         * Get the lower joint limit, usually in meters.
         * @return {number}
         */
        getLowerLimit(): number;
        /**
         * Get the upper joint limit, usually in meters.
         * @return {number}
         */
        getUpperLimit(): number;
        /**
         * Set the joint limits, usually in meters.
         * @param {number} lower
         * @param {number} upper
         */
        setLimits(lower: number, upper: number): void;
        /**
         * Is the joint motor enabled?
         * @return {boolean}
         */
        isMotorEnabled(): boolean;
        /**
         * Enable/disable the joint motor.
         * @param {boolean} flag
         */
        enableMotor(flag: boolean): void;
        /**
         * Set the motor speed, usually in meters per second.
         * @param {number} speed
         */
        setMotorSpeed(speed: number): void;
        /**
         * Get the motor speed, usually in meters per second.
         * @return {number}
         */
        getMotorSpeed(): number;
        /**
         * Set the maximum motor torque, usually in N.
         * @param {number} force
         */
        setMaxMotorForce(force: number): void;
        /**
         * Get the current motor torque, usually in N.
         * @return {number}
         */
        getMotorForce(): number;
    }
}
declare namespace org.jbox2d.dynamics.joints {
    class PulleyJoint extends org.jbox2d.dynamics.joints.Joint {
        static MIN_PULLEY_LENGTH: number;
        m_ground: org.jbox2d.dynamics.Body;
        m_groundAnchor1: org.jbox2d.common.Vec2;
        m_groundAnchor2: org.jbox2d.common.Vec2;
        m_localAnchor1: org.jbox2d.common.Vec2;
        m_localAnchor2: org.jbox2d.common.Vec2;
        m_u1: org.jbox2d.common.Vec2;
        m_u2: org.jbox2d.common.Vec2;
        m_constant: number;
        m_ratio: number;
        m_maxLength1: number;
        m_maxLength2: number;
        m_pulleyMass: number;
        m_limitMass1: number;
        m_limitMass2: number;
        m_force: number;
        m_limitForce1: number;
        m_limitForce2: number;
        m_positionImpulse: number;
        m_limitPositionImpulse1: number;
        m_limitPositionImpulse2: number;
        m_state: org.jbox2d.dynamics.joints.LimitState;
        m_limitState1: org.jbox2d.dynamics.joints.LimitState;
        m_limitState2: org.jbox2d.dynamics.joints.LimitState;
        constructor(def: org.jbox2d.dynamics.joints.PulleyJointDef);
        static tlr1: org.jbox2d.pooling.TLVec2;
        static tlr1_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlr2: org.jbox2d.pooling.TLVec2;
        static tlr2_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlp1: org.jbox2d.pooling.TLVec2;
        static tlp1_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlp2: org.jbox2d.pooling.TLVec2;
        static tlp2_$LI$(): org.jbox2d.pooling.TLVec2;
        static tls1: org.jbox2d.pooling.TLVec2;
        static tls1_$LI$(): org.jbox2d.pooling.TLVec2;
        static tls2: org.jbox2d.pooling.TLVec2;
        static tls2_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlP1: org.jbox2d.pooling.TLVec2;
        static tlP1_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlP2: org.jbox2d.pooling.TLVec2;
        static tlP2_$LI$(): org.jbox2d.pooling.TLVec2;
        /**
         *
         * @param {org.jbox2d.dynamics.TimeStep} step
         */
        initVelocityConstraints(step: org.jbox2d.dynamics.TimeStep): void;
        static tlv1: org.jbox2d.pooling.TLVec2;
        static tlv1_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlv2: org.jbox2d.pooling.TLVec2;
        static tlv2_$LI$(): org.jbox2d.pooling.TLVec2;
        /**
         *
         * @param {org.jbox2d.dynamics.TimeStep} step
         */
        solveVelocityConstraints(step: org.jbox2d.dynamics.TimeStep): void;
        /**
         *
         * @return {boolean}
         */
        solvePositionConstraints(): boolean;
        /**
         *
         * @return {org.jbox2d.common.Vec2}
         */
        getAnchor1(): org.jbox2d.common.Vec2;
        /**
         *
         * @return {org.jbox2d.common.Vec2}
         */
        getAnchor2(): org.jbox2d.common.Vec2;
        /**
         *
         * @return {org.jbox2d.common.Vec2}
         */
        getReactionForce(): org.jbox2d.common.Vec2;
        /**
         *
         * @return {number}
         */
        getReactionTorque(): number;
        getGroundAnchor1(): org.jbox2d.common.Vec2;
        getGroundAnchor2(): org.jbox2d.common.Vec2;
        getLength1(): number;
        getLength2(): number;
        getRatio(): number;
    }
}
declare namespace org.jbox2d.dynamics.joints {
    class RevoluteJoint extends org.jbox2d.dynamics.joints.Joint {
        m_localAnchor1: org.jbox2d.common.Vec2;
        m_localAnchor2: org.jbox2d.common.Vec2;
        m_pivotForce: org.jbox2d.common.Vec2;
        m_motorForce: number;
        m_limitForce: number;
        m_limitPositionImpulse: number;
        m_pivotMass: org.jbox2d.common.Mat22;
        m_motorMass: number;
        m_enableMotor: boolean;
        m_maxMotorTorque: number;
        m_motorSpeed: number;
        m_enableLimit: boolean;
        m_referenceAngle: number;
        m_lowerAngle: number;
        m_upperAngle: number;
        m_limitState: org.jbox2d.dynamics.joints.LimitState;
        constructor(def: org.jbox2d.dynamics.joints.RevoluteJointDef);
        static tlr1: org.jbox2d.pooling.TLVec2;
        static tlr1_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlr2: org.jbox2d.pooling.TLVec2;
        static tlr2_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlK1: org.jbox2d.pooling.TLMat22;
        static tlK1_$LI$(): org.jbox2d.pooling.TLMat22;
        static tlK2: org.jbox2d.pooling.TLMat22;
        static tlK2_$LI$(): org.jbox2d.pooling.TLMat22;
        static tlK3: org.jbox2d.pooling.TLMat22;
        static tlK3_$LI$(): org.jbox2d.pooling.TLMat22;
        /**
         *
         * @param {org.jbox2d.dynamics.TimeStep} step
         */
        initVelocityConstraints(step: org.jbox2d.dynamics.TimeStep): void;
        m_lastWarmStartingPivotForce: org.jbox2d.common.Vec2;
        static tltemp: org.jbox2d.pooling.TLVec2;
        static tltemp_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlpivotCdot: org.jbox2d.pooling.TLVec2;
        static tlpivotCdot_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlpivotForce: org.jbox2d.pooling.TLVec2;
        static tlpivotForce_$LI$(): org.jbox2d.pooling.TLVec2;
        /**
         *
         * @param {org.jbox2d.dynamics.TimeStep} step
         */
        solveVelocityConstraints(step: org.jbox2d.dynamics.TimeStep): void;
        static tlp1: org.jbox2d.pooling.TLVec2;
        static tlp1_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlp2: org.jbox2d.pooling.TLVec2;
        static tlp2_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlptpC: org.jbox2d.pooling.TLVec2;
        static tlptpC_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlimpulse: org.jbox2d.pooling.TLVec2;
        static tlimpulse_$LI$(): org.jbox2d.pooling.TLVec2;
        /**
         *
         * @return {boolean}
         */
        solvePositionConstraints(): boolean;
        /**
         *
         * @return {org.jbox2d.common.Vec2}
         */
        getAnchor1(): org.jbox2d.common.Vec2;
        /**
         *
         * @return {org.jbox2d.common.Vec2}
         */
        getAnchor2(): org.jbox2d.common.Vec2;
        /**
         *
         * @return {org.jbox2d.common.Vec2}
         */
        getReactionForce(): org.jbox2d.common.Vec2;
        /**
         *
         * @return {number}
         */
        getReactionTorque(): number;
        getJointAngle(): number;
        getJointSpeed(): number;
        isMotorEnabled(): boolean;
        enableMotor(flag: boolean): void;
        getMotorTorque(): number;
        setMotorSpeed(speed: number): void;
        setMaxMotorTorque(torque: number): void;
        isLimitEnabled(): boolean;
        enableLimit(flag: boolean): void;
        getLowerLimit(): number;
        getUpperLimit(): number;
        setLimits(lower: number, upper: number): void;
    }
}
declare namespace org.jbox2d.collision {
    /**
     * Handles conservative advancement to compute time of impact between shapes.
     * @class
     */
    class TOI {
        static tlxf1: org.jbox2d.pooling.TLXForm;
        static tlxf1_$LI$(): org.jbox2d.pooling.TLXForm;
        static tlxf2: org.jbox2d.pooling.TLXForm;
        static tlxf2_$LI$(): org.jbox2d.pooling.TLXForm;
        static tlP1: org.jbox2d.pooling.TLVec2;
        static tlP1_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlP2: org.jbox2d.pooling.TLVec2;
        static tlP2_$LI$(): org.jbox2d.pooling.TLVec2;
        /**
         * Compute the time when two shapes begin to touch or touch at a closer distance.
         * <BR><BR><em>Warning</em>: the sweeps must have the same time interval.
         * @return {number} the fraction between [0,1] in which the shapes first touch.
         * fraction=0 means the shapes begin touching/overlapped, and fraction=1 means the shapes don't touch.
         * @param {org.jbox2d.collision.shapes.Shape} shape1
         * @param {org.jbox2d.common.Sweep} sweep1
         * @param {org.jbox2d.collision.shapes.Shape} shape2
         * @param {org.jbox2d.common.Sweep} sweep2
         */
        static timeOfImpact(shape1: org.jbox2d.collision.shapes.Shape, sweep1: org.jbox2d.common.Sweep, shape2: org.jbox2d.collision.shapes.Shape, sweep2: org.jbox2d.common.Sweep): number;
    }
}
declare namespace org.jbox2d.dynamics {
    /**
     * Should not be called by user, as it will not
     * be properly added to the world.  Instead,
     * create a BodyDef object and pass it
     * to World.createDynamicBody or World.createStaticBody.
     *
     * @param {org.jbox2d.dynamics.BodyDef} bd Body definition
     * @param {org.jbox2d.dynamics.World} world World to create body in
     * @class
     */
    class Body {
        static nextID: number;
        static idLock: any;
        static idLock_$LI$(): any;
        m_uniqueID: number;
        static e_frozenFlag: number;
        static e_islandFlag: number;
        static e_sleepFlag: number;
        static e_allowSleepFlag: number;
        static e_bulletFlag: number;
        static e_fixedRotationFlag: number;
        m_flags: number;
        static e_staticType: number;
        static e_dynamicType: number;
        static e_maxTypes: number;
        m_type: number;
        m_controllerList: org.jbox2d.dynamics.controllers.ControllerEdge;
        /**
         * The body origin transform
         */
        m_xf: org.jbox2d.common.XForm;
        /**
         * The swept motion for CCD
         */
        m_sweep: org.jbox2d.common.Sweep;
        m_linearVelocity: org.jbox2d.common.Vec2;
        m_angularVelocity: number;
        m_force: org.jbox2d.common.Vec2;
        m_torque: number;
        m_world: org.jbox2d.dynamics.World;
        m_prev: Body;
        m_next: Body;
        m_shapeList: org.jbox2d.collision.shapes.Shape;
        m_shapeCount: number;
        m_jointList: org.jbox2d.dynamics.joints.JointEdge;
        m_contactList: org.jbox2d.dynamics.contacts.ContactEdge;
        m_mass: number;
        m_invMass: number;
        m_I: number;
        m_invI: number;
        m_linearDamping: number;
        m_angularDamping: number;
        m_sleepTime: number;
        /**
         * A holder to attach external data to a body.
         * Useful to keep track of what game entity
         * each body represents. This is copied from
         * the BodyDef used to create the body, so may
         * be set there instead.
         */
        m_userData: any;
        constructor(bd?: any, world?: any);
        connectEdges(s1: org.jbox2d.collision.shapes.EdgeShape, s2: org.jbox2d.collision.shapes.EdgeShape, angle1: number): number;
        /**
         * Creates a shape and attach it to this body.
         * <BR><em>Warning</em>: This function is locked during callbacks.
         * @param {org.jbox2d.collision.shapes.ShapeDef} def the shape definition.
         * @return {org.jbox2d.collision.shapes.Shape}
         */
        createShape(def: org.jbox2d.collision.shapes.ShapeDef): org.jbox2d.collision.shapes.Shape;
        /**
         * Destroy a shape. This removes the shape from the broad-phase and
         * therefore destroys any contacts associated with this shape. All shapes
         * attached to a body are implicitly destroyed when the body is destroyed.
         * <BR><em>Warning</em>: This function is locked during callbacks.
         * @param {org.jbox2d.collision.shapes.Shape} s the shape to be removed.
         */
        destroyShape(s: org.jbox2d.collision.shapes.Shape): void;
        /**
         * Set the mass properties. Note that this changes the center of mass position.
         * If you are not sure how to compute mass properties, use setMassFromShapes().
         * The inertia tensor is assumed to be relative to the center of mass.
         * @param {org.jbox2d.collision.MassData} massData the mass properties.
         */
        setMass(massData: org.jbox2d.collision.MassData): void;
        static tlCenter: org.jbox2d.pooling.TLVec2;
        static tlCenter_$LI$(): org.jbox2d.pooling.TLVec2;
        /**
         * Compute the mass properties from the attached shapes. You typically call this
         * after adding all the shapes. If you add or remove shapes later, you may want
         * to call this again. Note that this changes the center of mass position.
         */
        setMassFromShapes(): void;
        /**
         * Set the position of the body's origin and rotation (radians).
         * This breaks any contacts and wakes the other bodies.
         * @param {org.jbox2d.common.Vec2} position the new world position of the body's origin (not necessarily
         * the center of mass).
         * @param {number} angle the new world rotation angle of the body in radians.
         * @return {boolean} false if the movement put a shape outside the world. In this case the
         * body is automatically frozen.
         */
        setXForm(position: org.jbox2d.common.Vec2, angle: number): boolean;
        /**
         * Get a copy of the body transform for the body's origin.
         * @return {org.jbox2d.common.XForm} the world transform of the body's origin.
         */
        getXForm(): org.jbox2d.common.XForm;
        /**
         * More for internal use.  It isn't copied,
         * so don't modify it.  instead try to use {@link #setXForm(Vec2, float)}.
         * Otherwise, this also gives you direct access to the body's XForm, if you
         * really need to change something (careful!).
         * @see #getXForm()
         * @see #setXForm(Vec2, float)
         * @return {org.jbox2d.common.XForm} an uncopied version of this body's XForm
         */
        getMemberXForm(): org.jbox2d.common.XForm;
        /**
         * You probably don't want to use this
         * function.  What you really want is getWorldCenter(),
         * which returns the center of mass (which actually has
         * some physical significance).
         * <p>
         * Just in case you do want to use this,
         * Get a copy of the world body origin position.  This
         * is not necessarily the same as the center of mass.
         * In fact, it's not anything in particular.  Just a
         * point.
         * <p>
         * @return {org.jbox2d.common.Vec2} a copy of the world position of the body's origin.
         */
        getPosition(): org.jbox2d.common.Vec2;
        /**
         * This is more for internal use.  It isn't copied, so don't
         * modify it.  This is the position of the body's XForm
         * ({@link #getXForm()}), and if you want to change that I would
         * suggest using {@link #setXForm(Vec2, float)}.  Modifying this
         * will not do what you want.
         * @see #getPosition()
         * @return {org.jbox2d.common.Vec2} the body's world position of the body's origin.
         */
        getMemberPosition(): org.jbox2d.common.Vec2;
        /**
         * Get the angle in radians.
         * @return {number} the current world rotation angle in radians.
         */
        getAngle(): number;
        /**
         * Get a copy of the world position of the center of mass.
         * @return {org.jbox2d.common.Vec2} a copy of the world position
         */
        getWorldCenter(): org.jbox2d.common.Vec2;
        /**
         * More for internal use. It isn't copied, so don't
         * modify it.  Modifying this will not do what you want,
         * instead use {@link #setXForm(Vec2, float)}
         * @see #getWorldCenter()
         * @return {org.jbox2d.common.Vec2} the world position
         */
        getMemberWorldCenter(): org.jbox2d.common.Vec2;
        /**
         * Get local position of the center of mass.
         * @return {org.jbox2d.common.Vec2} a copy of the local position of the center of mass
         */
        getLocalCenter(): org.jbox2d.common.Vec2;
        /**
         * More for internal use. It isn't a copy, so don't
         * modify it.
         * @return {org.jbox2d.common.Vec2} the local position of the center of mass
         */
        getMemberLocalCenter(): org.jbox2d.common.Vec2;
        /**
         * Set the linear velocity of the center of mass.
         * @param {org.jbox2d.common.Vec2} v the new linear velocity of the center of mass.
         */
        setLinearVelocity(v: org.jbox2d.common.Vec2): void;
        /**
         * Get the linear velocity of the center of mass. This isn't a copy,
         * so modifying this will change the linear velocity.
         * @return {org.jbox2d.common.Vec2} a the linear velocity of the center of mass.
         */
        getLinearVelocity(): org.jbox2d.common.Vec2;
        /**
         * Set the angular velocity.
         * @param {number} omega the new angular velocity in radians/second.
         */
        setAngularVelocity(omega: number): void;
        /**
         * Get the angular velocity.
         * @return {number} the angular velocity in radians/second.
         */
        getAngularVelocity(): number;
        /**
         * Apply a force at a world point. If the force is not
         * applied at the center of mass, it will generate a torque and
         * affect the angular velocity. This wakes up the body.
         * @param {org.jbox2d.common.Vec2} force the world force vector, usually in Newtons (N).
         * @param {org.jbox2d.common.Vec2} point the world position of the point of application.
         */
        applyForce(force: org.jbox2d.common.Vec2, point: org.jbox2d.common.Vec2): void;
        /**
         * Apply a torque. This affects the angular velocity
         * without affecting the linear velocity of the center of mass.
         * This wakes up the body.
         * @param {number} torque about the z-axis (out of the screen), usually in N-m.
         */
        applyTorque(torque: number): void;
        /**
         * Apply an impulse at a point. This immediately modifies the velocity.
         * It also modifies the angular velocity if the point of application
         * is not at the center of mass. This wakes up the body.
         * @param {org.jbox2d.common.Vec2} impulse the world impulse vector, usually in N-seconds or kg-m/s.
         * @param {org.jbox2d.common.Vec2} point the world position of the point of application.
         */
        applyImpulse(impulse: org.jbox2d.common.Vec2, point: org.jbox2d.common.Vec2): void;
        /**
         * Get the total mass of the body.
         * @return {number} the mass, usually in kilograms (kg).
         */
        getMass(): number;
        /**
         * Get the central rotational inertia of the body.
         * @return {number} the rotational inertia, usually in kg-m^2.
         */
        getInertia(): number;
        /**
         * Get the world coordinates of a point given the local coordinates.
         * @param {org.jbox2d.common.Vec2} localPoint a point on the body measured relative the the body's origin.
         * @return {org.jbox2d.common.Vec2} the same point expressed in world coordinates.
         */
        getWorldLocation(localPoint: org.jbox2d.common.Vec2): org.jbox2d.common.Vec2;
        /**
         * Get the world coordinates of a point given the local coordinates.
         * @param {org.jbox2d.common.Vec2} localPoint a point on the body measured relative the the body's origin.
         * @param {org.jbox2d.common.Vec2} out where to put the same point expressed in world coordinates.
         */
        getWorldLocationToOut(localPoint: org.jbox2d.common.Vec2, out: org.jbox2d.common.Vec2): void;
        /**
         * Get the world coordinates of a point given the local coordinates.
         * @param {org.jbox2d.common.Vec2} localPoint a point on the body measured relative the the body's origin.
         * @return {org.jbox2d.common.Vec2} the same point expressed in world coordinates.
         * @deprecated Use getWorldLocation instead (clearer naming convention)
         */
        getWorldPoint(localPoint: org.jbox2d.common.Vec2): org.jbox2d.common.Vec2;
        /**
         * Get the world coordinates of a vector given the local coordinates.
         * @param {org.jbox2d.common.Vec2} localVector a vector fixed in the body.
         * @return {org.jbox2d.common.Vec2} the same vector expressed in world coordinates.
         * @deprecated Use getWorldDirection instead (clearer naming convention)
         */
        getWorldVector(localVector: org.jbox2d.common.Vec2): org.jbox2d.common.Vec2;
        /**
         * Get the world coordinates of a direction given the local direction.
         * @param {org.jbox2d.common.Vec2} localDirection a vector fixed in the body.
         * @return {org.jbox2d.common.Vec2} the same vector expressed in world coordinates.
         */
        getWorldDirection(localDirection: org.jbox2d.common.Vec2): org.jbox2d.common.Vec2;
        /**
         * Get the world coordinates of a direction given the local direction.
         * @param {org.jbox2d.common.Vec2} localDirection a vector fixed in the body.
         * @param {org.jbox2d.common.Vec2} out where to put the same vector expressed in world coordinates.
         */
        getWorldDirectionToOut(localDirection: org.jbox2d.common.Vec2, out: org.jbox2d.common.Vec2): void;
        /**
         * Gets a local point relative to the body's origin given a world point.
         * @param {org.jbox2d.common.Vec2} worldPoint a point in world coordinates.
         * @return {org.jbox2d.common.Vec2} the corresponding local point relative to the body's origin.
         */
        getLocalPoint(worldPoint: org.jbox2d.common.Vec2): org.jbox2d.common.Vec2;
        /**
         * Gets a local point relative to the body's origin given a world point.
         * @param {org.jbox2d.common.Vec2} worldPoint a point in world coordinates.
         * @param {org.jbox2d.common.Vec2} out where to put the the corresponding local point relative to the body's origin.
         */
        getLocalPointToOut(worldPoint: org.jbox2d.common.Vec2, out: org.jbox2d.common.Vec2): void;
        /**
         * Gets a local vector given a world vector.
         * @param {org.jbox2d.common.Vec2} worldVector a vector in world coordinates.
         * @return {org.jbox2d.common.Vec2} the corresponding local vector.
         */
        getLocalVector(worldVector: org.jbox2d.common.Vec2): org.jbox2d.common.Vec2;
        /**
         * Gets a local vector given a world vector.
         * @param {org.jbox2d.common.Vec2} worldVector a vector in world coordinates.
         * @param {org.jbox2d.common.Vec2} out where to put the corresponding local vector.
         */
        getLocalVectorToOut(worldVector: org.jbox2d.common.Vec2, out: org.jbox2d.common.Vec2): void;
        /**
         * Is this body treated like a bullet for continuous collision detection?
         * @return {boolean}
         */
        isBullet(): boolean;
        /**
         * Should this body be treated like a bullet for continuous collision detection?
         * Use sparingly, as continuous collision detection can be expensive.
         * @param {boolean} flag
         */
        setBullet(flag: boolean): void;
        /**
         * Is this body static (immovable)?
         * @return {boolean}
         */
        isStatic(): boolean;
        /**
         * Is this body dynamic (movable)?
         * @return {boolean}
         */
        isDynamic(): boolean;
        /**
         * Is this body frozen?
         * @return {boolean}
         */
        isFrozen(): boolean;
        /**
         * Is this body sleeping (not simulating).
         * @return {boolean}
         */
        isSleeping(): boolean;
        /**
         * Set to false to prevent this body from sleeping due to inactivity.
         * @param {boolean} flag
         */
        allowSleeping(flag: boolean): void;
        /**
         * Wake up this body so it will begin simulating.
         */
        wakeUp(): void;
        /**
         * Get the linked list of all shapes attached to this body.
         * @return {org.jbox2d.collision.shapes.Shape} first Shape in linked list
         */
        getShapeList(): org.jbox2d.collision.shapes.Shape;
        /**
         * Get the linked list of all joints attached to this body.
         * @return {org.jbox2d.dynamics.joints.JointEdge} first JointEdge in linked list
         */
        getJointList(): org.jbox2d.dynamics.joints.JointEdge;
        /**
         * Get the next body in the world's body list.
         * @return {org.jbox2d.dynamics.Body}
         */
        getNext(): Body;
        /**
         * Get the user data Object reference that was provided in the body definition.
         * @return {*}
         */
        getUserData(): any;
        /**
         * For internal use only.
         */
        computeMass(): void;
        static tlXf1: org.jbox2d.pooling.TLXForm;
        static tlXf1_$LI$(): org.jbox2d.pooling.TLXForm;
        /**
         * For internal use only.
         * @return {boolean}
         */
        synchronizeShapes(): boolean;
        /**
         * For internal use only.
         */
        synchronizeTransform(): void;
        /**
         * This is used to prevent connected bodies from colliding.
         * It may lie, depending on the collideConnected flag, so
         * it won't be very useful external to the engine.
         * @param {org.jbox2d.dynamics.Body} other
         * @return {boolean}
         */
        isConnected(other: Body): boolean;
        /**
         * For internal use only.
         * @param {number} t
         */
        advance(t: number): void;
        /**
         * Get the world linear velocity of a world point attached to this body.
         * @param {org.jbox2d.common.Vec2} worldPoint a point in world coordinates.
         * @return {org.jbox2d.common.Vec2} the world velocity of a point.
         */
        getLinearVelocityFromWorldPoint(worldPoint: org.jbox2d.common.Vec2): org.jbox2d.common.Vec2;
        /**
         * Get the world linear velocity of a world point attached to this body.
         * @param {org.jbox2d.common.Vec2} worldPoint a point in world coordinates.
         * @param {org.jbox2d.common.Vec2} out where to put the world velocity of a point.
         */
        getLinearVelocityFromWorldPointToOut(worldPoint: org.jbox2d.common.Vec2, out: org.jbox2d.common.Vec2): void;
        /**
         * Get the world velocity of a local point.
         * @param {org.jbox2d.common.Vec2} localPoint a point in local coordinates.
         * @return {org.jbox2d.common.Vec2} the world velocity of a point.
         */
        getLinearVelocityFromLocalPoint(localPoint: org.jbox2d.common.Vec2): org.jbox2d.common.Vec2;
        /**
         * Get the world velocity of a local point.
         * @param {org.jbox2d.common.Vec2} localPoint a point in local coordinates.
         * @param {org.jbox2d.common.Vec2} out where to put the world velocity of a point.
         */
        getLinearVelocityFromLocalPointToOut(localPoint: org.jbox2d.common.Vec2, out: org.jbox2d.common.Vec2): void;
        /**
         * Put this body to sleep so it will stop simulating.
         * This also sets the velocity to zero.
         */
        putToSleep(): void;
        setUserData(data: any): void;
        getWorld(): org.jbox2d.dynamics.World;
        /**
         * Get the contact list, represented as a linked list of ContactEdges. Will
         * return null if no contacts are present.
         *
         * @return {org.jbox2d.dynamics.contacts.ContactEdge} the head of the linked list of contacts
         */
        getContactList(): org.jbox2d.dynamics.contacts.ContactEdge;
        /**
         * Get the set of bodies in contact with this body.
         *
         * @return {org.jbox2d.dynamics.Body[]} all bodies touching this one
         */
        getBodiesInContact(): Array<Body>;
        /**
         * Get the set of bodies connected to this one by a joint.
         * Note: this does not return the entire island of connected bodies,
         * only those directly connected to this one.
         * @return {org.jbox2d.dynamics.Body[]} all bodies connected directly to this body by a joint
         */
        getConnectedBodies(): Array<Body>;
        /**
         * Get the set of dynamic bodies connected to this one by a joint.
         * Note: this does not return the entire island of connected bodies,
         * only those directly connected to this one.
         * @return {org.jbox2d.dynamics.Body[]} all bodies connected directly to this body by a joint
         */
        getConnectedDynamicBodies(): Array<Body>;
        /**
         * Get the island of connected bodies, including the current body.
         * <em>Warning</em>: will continue walking the joint tree past static bodies,
         * which may lead to unwanted results esp. if bodies are connected to the ground
         * body.
         * @return {org.jbox2d.dynamics.Body[]} Set<Body> of all bodies accessible from this one by walking the joint tree
         */
        getConnectedBodyIsland(): Array<Body>;
        getConnectedBodyIsland_impl(parent: Body, parentResult: Array<Body>): Array<Body>;
        /**
         * Get the island of joint-connected dynamic bodies, including the current body.
         * Stops walking tree if it encounters a static body.
         * @see Body#getConnectedBodyIsland()
         * @return {org.jbox2d.dynamics.Body[]} Set<Body> of all bodies accessible from this one by walking the joint tree
         */
        getConnectedDynamicBodyIsland(): Array<Body>;
        getConnectedDynamicBodyIsland_impl(parent: Body, parentResult: Array<Body>): Array<Body>;
        /**
         * Get the island of bodies in contact, including the current body.
         * <em>Warning</em>: will continue walking the contact tree past static bodies,
         * which may lead to unwanted results esp. if bodies are touching the ground
         * body.
         * @return {org.jbox2d.dynamics.Body[]} Set<Body> of all bodies accessible from this one by walking the contact tree
         */
        getTouchingBodyIsland(): Array<Body>;
        getTouchingBodyIsland_impl(parent: Body, parentResult: Array<Body>): Array<Body>;
        /**
         * Get the island of dynamic bodies in contact, including the current body.
         * Stops walking tree if it encounters a static body.
         * @return {org.jbox2d.dynamics.Body[]} Set<Body> of all bodies accessible from this one by walking the contact tree
         */
        getTouchingDynamicBodyIsland(): Array<Body>;
        getTouchingDynamicBodyIsland_impl(parent: Body, parentResult: Array<Body>): Array<Body>;
        /**
         * @return {boolean} true if this Body is currently in contact with the passed body
         * @param {org.jbox2d.dynamics.Body} other
         */
        isTouching(other: Body): boolean;
        setLinearDamping(damping: number): void;
        getLinearDamping(): number;
        setAngularDamping(damping: number): void;
        getAngularDamping(): number;
    }
}
declare namespace org.jbox2d.util.blob {
    /**
     * Simple non-reinforced square lattice.
     * Easy to collapse, but simple.
     * @class
     * @extends org.jbox2d.util.blob.BlobStructure
     */
    class SquareLatticeStructure extends org.jbox2d.util.blob.BlobStructure {
        constructor();
    }
}
declare namespace org.jbox2d.util.sph {
    class SmoothParticle extends org.jbox2d.util.sph.Particle {
        ID: number;
        pressure: number;
        density: number;
        h: number;
        c: number;
        ChangeDensity: number;
        ChangeVelocity: org.jbox2d.common.Vec2;
        shapeID: number;
        numNeighbors: number;
        minDensity: number;
        maxDensity: number;
        constructor(p?: any, v?: any, m?: any, d?: any, pr?: any, len?: any, speedSnd?: any);
        setPressure(p: number): void;
        setDensity(den: number): void;
        setSmoothingLength(len: number): void;
        setSpeedSound(spSnd: number): void;
        setChangeVelocity(ChangeV: org.jbox2d.common.Vec2): void;
        setChangeDensity(cd: number): void;
        setMinDensity(m: number): void;
        setMaxDensity(m: number): void;
        setShapeID(id: number): void;
        setNumNeighbors(n: number): void;
        getPressure(): number;
        getSmoothingLength(): number;
        getDensity(): number;
        getSpeedSound(): number;
        getChangeDensity(): number;
        getChangeVelocity(): org.jbox2d.common.Vec2;
        getChangeVelocityX(): number;
        getChangeVelocityY(): number;
        getShapeID(): number;
        getMinDensity(): number;
        getMaxDensity(): number;
        getNumNeighbors(): number;
        /**
         * Calculates the pressure for this SPH particle.
         * The state equation comes from **** and is used to model water.
         * @param {org.jbox2d.util.sph.Parameter} p
         */
        calcPressure(p: org.jbox2d.util.sph.Parameter): void;
        /**
         * Zeroes out the delta values. The reason is because the delta
         * values come from summing up contributions from all neighbors and
         * we don't want to add to the last time steps value.
         */
        zeroSPHVars(): void;
        /**
         * Adds the contributions of the change in density from the neighbor
         * "sp".
         * @param {org.jbox2d.util.sph.SmoothParticle} sp
         * @param {org.jbox2d.common.Vec2} v_ij
         */
        calcChangeDensity(sp: SmoothParticle, v_ij: org.jbox2d.common.Vec2): void;
        /**
         * Add the contribution of the acceleration due to the free
         * particle "sp".
         * @param {org.jbox2d.util.sph.SmoothParticle} sp
         * @param {org.jbox2d.common.Vec2} v_ij
         * @param {org.jbox2d.util.sph.Parameter} param
         */
        calcChangeVelocity(sp: SmoothParticle, v_ij: org.jbox2d.common.Vec2, param: org.jbox2d.util.sph.Parameter): void;
        /**
         * Add the body force in the x direction.
         * @param {org.jbox2d.util.sph.Parameter} param
         */
        addForceX(param: org.jbox2d.util.sph.Parameter): void;
        /**
         * Add the body force in the y direction.
         * @param {org.jbox2d.util.sph.Parameter} param
         */
        addForceY(param: org.jbox2d.util.sph.Parameter): void;
        /**
         * Calculate the pressure gradient due to the free particle "sp".
         * @param {org.jbox2d.util.sph.SmoothParticle} sp
         * @return {number}
         * @private
         */
        pressureTerm(sp: SmoothParticle): number;
        /**
         * This is used to simulate viscous fluids.
         * @param {org.jbox2d.util.sph.SmoothParticle} sp
         * @param {org.jbox2d.util.sph.Parameter} param
         * @return {number}
         */
        artificialViscosity(sp: SmoothParticle, param: org.jbox2d.util.sph.Parameter): number;
        /**
         * SPH smoothing kernel
         * @param {org.jbox2d.util.sph.SmoothParticle} sp
         * @return {number}
         */
        kernel(sp: SmoothParticle): number;
        /**
         * X component of gradient of SPH smoothing kernel
         * @param {org.jbox2d.util.sph.SmoothParticle} sp
         * @return {number}
         */
        gradientKernelX(sp: SmoothParticle): number;
        /**
         * Y component of gradient of SPH smoothing kernel
         * @param {org.jbox2d.util.sph.SmoothParticle} sp
         * @return {number}
         */
        gradientKernelY(sp: SmoothParticle): number;
        /**
         * Keeps track of our particle's min and max density
         */
        setMinMaxDensity(): void;
        /**
         * Used to keep a total of the number of neighbors within 2*h of
         * this smooth particle.  Used for debugging.
         * @param {org.jbox2d.util.sph.SmoothParticle} sp
         */
        updateNumNeighbors(sp: SmoothParticle): void;
    }
}
declare namespace org.jbox2d.dynamics.contacts {
    class PointAndPolyContact extends org.jbox2d.dynamics.contacts.Contact implements org.jbox2d.dynamics.contacts.ContactCreateFcn {
        m_manifold: org.jbox2d.collision.Manifold;
        manifoldList: Array<org.jbox2d.collision.Manifold>;
        constructor(s1?: any, s2?: any);
        /**
         *
         * @return {org.jbox2d.dynamics.contacts.Contact}
         */
        clone(): org.jbox2d.dynamics.contacts.Contact;
        create(shape1: org.jbox2d.collision.shapes.Shape, shape2: org.jbox2d.collision.shapes.Shape): org.jbox2d.dynamics.contacts.Contact;
        /**
         *
         * @return {org.jbox2d.collision.Manifold[]}
         */
        getManifolds(): Array<org.jbox2d.collision.Manifold>;
        getFirstManifold(): org.jbox2d.collision.Manifold;
        static tlm0: org.jbox2d.pooling.TLManifold;
        static tlm0_$LI$(): org.jbox2d.pooling.TLManifold;
        static tlV1: org.jbox2d.pooling.TLVec2;
        static tlV1_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlCp: org.jbox2d.pooling.TLContactPoint;
        static tlCp_$LI$(): org.jbox2d.pooling.TLContactPoint;
        static tlPersisted: org.jbox2d.pooling.arrays.BooleanArray;
        static tlPersisted_$LI$(): org.jbox2d.pooling.arrays.BooleanArray;
        /**
         *
         * @param {*} listener
         */
        evaluate(listener: org.jbox2d.dynamics.ContactListener): void;
    }
}
declare namespace org.jbox2d.dynamics.contacts {
    class PolyAndCircleContact extends org.jbox2d.dynamics.contacts.Contact implements org.jbox2d.dynamics.contacts.ContactCreateFcn {
        m_manifold: org.jbox2d.collision.Manifold;
        manifoldList: Array<org.jbox2d.collision.Manifold>;
        constructor(s1?: any, s2?: any);
        /**
         *
         * @return {org.jbox2d.dynamics.contacts.Contact}
         */
        clone(): org.jbox2d.dynamics.contacts.Contact;
        create(shape1: org.jbox2d.collision.shapes.Shape, shape2: org.jbox2d.collision.shapes.Shape): org.jbox2d.dynamics.contacts.Contact;
        /**
         *
         * @return {org.jbox2d.collision.Manifold[]}
         */
        getManifolds(): Array<org.jbox2d.collision.Manifold>;
        static tlm0: org.jbox2d.pooling.TLManifold;
        static tlm0_$LI$(): org.jbox2d.pooling.TLManifold;
        static tlV1: org.jbox2d.pooling.TLVec2;
        static tlV1_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlCp: org.jbox2d.pooling.TLContactPoint;
        static tlCp_$LI$(): org.jbox2d.pooling.TLContactPoint;
        static tlPersisted: org.jbox2d.pooling.arrays.BooleanArray;
        static tlPersisted_$LI$(): org.jbox2d.pooling.arrays.BooleanArray;
        /**
         *
         * @param {*} listener
         */
        evaluate(listener: org.jbox2d.dynamics.ContactListener): void;
    }
}
declare namespace org.jbox2d.dynamics.contacts {
    class PolyAndEdgeContact extends org.jbox2d.dynamics.contacts.Contact implements org.jbox2d.dynamics.contacts.ContactCreateFcn {
        m_manifold: org.jbox2d.collision.Manifold;
        manifoldList: Array<org.jbox2d.collision.Manifold>;
        create(s1: org.jbox2d.collision.shapes.Shape, s2: org.jbox2d.collision.shapes.Shape): org.jbox2d.dynamics.contacts.Contact;
        constructor(shape1?: any, shape2?: any);
        static Destroy(contact: org.jbox2d.dynamics.contacts.Contact): void;
        destructor(): void;
        /**
         *
         * @return {org.jbox2d.dynamics.contacts.Contact}
         */
        clone(): org.jbox2d.dynamics.contacts.Contact;
        static tlm0: org.jbox2d.pooling.TLManifold;
        static tlm0_$LI$(): org.jbox2d.pooling.TLManifold;
        static tlV1: org.jbox2d.pooling.TLVec2;
        static tlV1_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlCp: org.jbox2d.pooling.TLContactPoint;
        static tlCp_$LI$(): org.jbox2d.pooling.TLContactPoint;
        static tlPersisted: org.jbox2d.pooling.arrays.BooleanArray;
        static tlPersisted_$LI$(): org.jbox2d.pooling.arrays.BooleanArray;
        /**
         *
         * @param {*} listener
         */
        evaluate(listener: org.jbox2d.dynamics.ContactListener): void;
        /**
         *
         * @return {org.jbox2d.collision.Manifold[]}
         */
        getManifolds(): Array<org.jbox2d.collision.Manifold>;
    }
}
declare namespace org.jbox2d.collision {
    /**
     * This broad phase uses the Sweep and Prune algorithm as described in:
     * Collision Detection in Interactive 3D Environments by Gino van den Bergen
     * Also, some ideas, such as using integral values for fast compares comes from
     * Bullet (http:/www.bulletphysics.com).<br/>
     * <br/>
     *
     * Notes:<br/>
     * - we use bound arrays instead of linked lists for cache coherence.<br/>
     * - we use quantized integral values for fast compares.<br/>
     * - we use short indices rather than pointers to save memory.<br/>
     * - we use a stabbing count for fast overlap queries (less than order N).<br/>
     * - we also use a time stamp on each proxy to speed up the registration of
     * overlap query results.<br/>
     * - where possible, we compare bound indices instead of values to reduce cache
     * misses (TODO_ERIN).<br/>
     * - no broadphase is perfect and neither is this one: it is not great for huge
     * worlds (use a multi-SAP instead), it is not great for large objects.
     * @param {org.jbox2d.collision.AABB} worldAABB
     * @param {*} callback
     * @class
     */
    class BroadPhase {
        static INVALID: number;
        static INVALID_$LI$(): number;
        static NULL_EDGE: number;
        static NULL_EDGE_$LI$(): number;
        m_pairManager: org.jbox2d.collision.PairManager;
        m_proxyPool: org.jbox2d.collision.Proxy[];
        m_freeProxy: number;
        pairBuffer: org.jbox2d.collision.BufferedPair[];
        m_pairBufferCount: number;
        m_bounds: org.jbox2d.collision.Bound[][];
        m_queryResults: number[];
        m_querySortKeys: number[];
        m_queryResultCount: number;
        m_worldAABB: org.jbox2d.collision.AABB;
        m_quantizationFactor: org.jbox2d.common.Vec2;
        m_proxyCount: number;
        m_timeStamp: number;
        static debugPrint: boolean;
        static s_validate: boolean;
        dump(): void;
        constructor(worldAABB: org.jbox2d.collision.AABB, callback: org.jbox2d.collision.PairCallback);
        testOverlap$org_jbox2d_collision_Proxy$org_jbox2d_collision_Proxy(p1: org.jbox2d.collision.Proxy, p2: org.jbox2d.collision.Proxy): boolean;
        testOverlap(p1?: any, p2?: any): any;
        testOverlap$org_jbox2d_collision_BoundValues$org_jbox2d_collision_Proxy(b: org.jbox2d.collision.BoundValues, p: org.jbox2d.collision.Proxy): boolean;
        getProxy(proxyId: number): org.jbox2d.collision.Proxy;
        static tlLowerValues: org.jbox2d.pooling.arrays.IntegerArray;
        static tlLowerValues_$LI$(): org.jbox2d.pooling.arrays.IntegerArray;
        static tlUpperValues: org.jbox2d.pooling.arrays.IntegerArray;
        static tlUpperValues_$LI$(): org.jbox2d.pooling.arrays.IntegerArray;
        static tlIndexes: org.jbox2d.pooling.arrays.IntegerArray;
        static tlIndexes_$LI$(): org.jbox2d.pooling.arrays.IntegerArray;
        /**
         * internal
         * @param {org.jbox2d.collision.AABB} aabb
         * @param {*} userData
         * @return {number}
         */
        createProxy(aabb: org.jbox2d.collision.AABB, userData: any): number;
        static tlIgnored: org.jbox2d.pooling.arrays.IntegerArray;
        static tlIgnored_$LI$(): org.jbox2d.pooling.arrays.IntegerArray;
        destroyProxy(proxyId: number): void;
        static tlNewValues: org.jbox2d.pooling.TLBoundValues;
        static tlNewValues_$LI$(): org.jbox2d.pooling.TLBoundValues;
        static tlOldValues: org.jbox2d.pooling.TLBoundValues;
        static tlOldValues_$LI$(): org.jbox2d.pooling.TLBoundValues;
        /**
         * internal
         * @param {number} proxyId
         * @param {org.jbox2d.collision.AABB} aabb
         */
        moveProxy(proxyId: number, aabb: org.jbox2d.collision.AABB): void;
        commit(): void;
        query$org_jbox2d_collision_AABB$int(aabb: org.jbox2d.collision.AABB, maxCount: number): any[];
        validate(): void;
        computeBounds(lowerValues: number[], upperValues: number[], aabb: org.jbox2d.collision.AABB): void;
        query$java_lang_Integer_A$int$int$org_jbox2d_collision_Bound_A$int$int(indexes: number[], lowerValue: number, upperValue: number, bounds: org.jbox2d.collision.Bound[], boundCount: number, axis: number): void;
        /**
         * @param {Array} indexes
         * out variable
         * @param {number} lowerValue
         * @param {number} upperValue
         * @param {Array} bounds
         * @param {number} boundCount
         * @param {number} axis
         * @private
         */
        query(indexes?: any, lowerValue?: any, upperValue?: any, bounds?: any, boundCount?: any, axis?: any): any;
        incrementOverlapCount(proxyId: number): void;
        incrementTimeStamp(): void;
        static binarySearch(bounds: org.jbox2d.collision.Bound[], count: number, value: number): number;
        inRange(aabb: org.jbox2d.collision.AABB): boolean;
        static tlResults: org.jbox2d.pooling.arrays.IntegerArray;
        static tlResults_$LI$(): org.jbox2d.pooling.arrays.IntegerArray;
        querySegment(segment: org.jbox2d.collision.Segment, userData: any[], maxCount: number, sortKey: org.jbox2d.collision.SortKeyFunc): number;
        addProxyResult(proxyId: number, proxy: org.jbox2d.collision.Proxy, maxCount: number, sortKey: org.jbox2d.collision.SortKeyFunc): void;
    }
}
declare namespace org.jbox2d.dynamics.joints {
    class ConstantVolumeJoint extends org.jbox2d.dynamics.joints.Joint {
        bodies: org.jbox2d.dynamics.Body[];
        targetLengths: number[];
        targetVolume: number;
        world: org.jbox2d.dynamics.World;
        normals: org.jbox2d.common.Vec2[];
        m_step: org.jbox2d.dynamics.TimeStep;
        m_impulse: number;
        distanceJoints: org.jbox2d.dynamics.joints.DistanceJoint[];
        getBodies(): org.jbox2d.dynamics.Body[];
        inflate(factor: number): void;
        constructor(def: org.jbox2d.dynamics.joints.ConstantVolumeJointDef);
        /**
         *
         */
        destructor(): void;
        getArea(): number;
        /**
         * Apply the position correction to the particles.
         * @param {org.jbox2d.dynamics.TimeStep} step
         * @return {boolean}
         */
        constrainEdges(step: org.jbox2d.dynamics.TimeStep): boolean;
        static tlD: org.jbox2d.pooling.arrays.Vec2Array;
        static tlD_$LI$(): org.jbox2d.pooling.arrays.Vec2Array;
        /**
         *
         * @param {org.jbox2d.dynamics.TimeStep} step
         */
        initVelocityConstraints(step: org.jbox2d.dynamics.TimeStep): void;
        /**
         *
         * @return {boolean}
         */
        solvePositionConstraints(): boolean;
        /**
         *
         * @param {org.jbox2d.dynamics.TimeStep} step
         */
        solveVelocityConstraints(step: org.jbox2d.dynamics.TimeStep): void;
        /**
         *
         * @return {org.jbox2d.common.Vec2}
         */
        getAnchor1(): org.jbox2d.common.Vec2;
        /**
         *
         * @return {org.jbox2d.common.Vec2}
         */
        getAnchor2(): org.jbox2d.common.Vec2;
        /**
         *
         * @return {org.jbox2d.common.Vec2}
         */
        getReactionForce(): org.jbox2d.common.Vec2;
        /**
         *
         * @return {number}
         */
        getReactionTorque(): number;
    }
}
declare namespace org.jbox2d.dynamics {
    /**
     * TODO djm: make this so it isn't created every time step
     * @class
     */
    class Island {
        m_bodies: org.jbox2d.dynamics.Body[];
        m_contacts: org.jbox2d.dynamics.contacts.Contact[];
        m_joints: org.jbox2d.dynamics.joints.Joint[];
        m_bodyCount: number;
        m_jointCount: number;
        m_contactCount: number;
        m_bodyCapacity: number;
        m_contactCapacity: number;
        m_jointCapacity: number;
        static m_positionIterationCount: number;
        m_positionError: number;
        m_listener: org.jbox2d.dynamics.ContactListener;
        clear(): void;
        add$org_jbox2d_dynamics_Body(body: org.jbox2d.dynamics.Body): void;
        add(body?: any): any;
        add$org_jbox2d_dynamics_contacts_Contact(contact: org.jbox2d.dynamics.contacts.Contact): void;
        add$org_jbox2d_dynamics_joints_Joint(joint: org.jbox2d.dynamics.joints.Joint): void;
        constructor();
        init(bodyCapacity: number, contactCapacity: number, jointCapacity: number, listener: org.jbox2d.dynamics.ContactListener): void;
        static contactSolvers: org.jbox2d.pooling.stacks.ContactSolverStack;
        static contactSolvers_$LI$(): org.jbox2d.pooling.stacks.ContactSolverStack;
        solve(step: org.jbox2d.dynamics.TimeStep, gravity: org.jbox2d.common.Vec2, correctPositions: boolean, allowSleep: boolean): void;
        solveTOI(subStep: org.jbox2d.dynamics.TimeStep): void;
        report$java_util_List(constraints: Array<org.jbox2d.dynamics.contacts.ContactConstraint>): void;
        report(constraints?: any): any;
        report$org_jbox2d_dynamics_contacts_ContactConstraint_A(constraints: org.jbox2d.dynamics.contacts.ContactConstraint[]): void;
    }
}
declare namespace org.jbox2d.dynamics {
    /**
     * Construct a world object.
     * @param {org.jbox2d.collision.AABB} worldAABB a bounding box that completely encompasses all your shapes.
     * @param {org.jbox2d.common.Vec2} gravity the world gravity vector.
     * @param {boolean} doSleep improve performance by not simulating inactive bodies.
     * @class
     */
    class World {
        m_lock: boolean;
        m_broadPhase: org.jbox2d.collision.BroadPhase;
        m_contactManager: org.jbox2d.dynamics.ContactManager;
        m_bodyList: org.jbox2d.dynamics.Body;
        /**
         * Do not access, won't be useful!
         */
        m_contactList: org.jbox2d.dynamics.contacts.Contact;
        m_jointList: org.jbox2d.dynamics.joints.Joint;
        m_controllerList: org.jbox2d.dynamics.controllers.Controller;
        m_controllerCount: number;
        m_bodyCount: number;
        m_contactCount: number;
        m_jointCount: number;
        m_gravity: org.jbox2d.common.Vec2;
        m_allowSleep: boolean;
        m_groundBody: org.jbox2d.dynamics.Body;
        m_positionIterationCount: number;
        /**
         * Should we apply position correction?
         */
        m_positionCorrection: boolean;
        /**
         * Should we use warm-starting?  Improves stability in stacking scenarios.
         */
        m_warmStarting: boolean;
        /**
         * Should we enable continuous collision detection?
         */
        m_continuousPhysics: boolean;
        m_destructionListener: org.jbox2d.dynamics.DestructionListener;
        m_boundaryListener: org.jbox2d.dynamics.BoundaryListener;
        m_contactFilter: org.jbox2d.dynamics.ContactFilter;
        m_contactListener: org.jbox2d.dynamics.ContactListener;
        m_debugDraw: org.jbox2d.dynamics.DebugDraw;
        m_drawDebugData: boolean;
        m_inv_dt0: number;
        postStepList: Array<org.jbox2d.dynamics.Steppable>;
        autoDebugDraw: boolean;
        /**
         * @return {boolean} the autoDebugDraw
         */
        isAutoDebugDraw(): boolean;
        /**
         * @param {boolean} autoDebugDraw the autoDebugDraw to set
         */
        setAutoDebugDraw(autoDebugDraw: boolean): void;
        setDrawDebugData(tf: boolean): void;
        isDrawingDebugData(): boolean;
        /**
         * Get the number of bodies.
         * @return {number}
         */
        getBodyCount(): number;
        /**
         * Get the number of joints.
         * @return {number}
         */
        getJointCount(): number;
        /**
         * Get the number of contacts (each may have 0 or more contact points).
         * @return {number}
         */
        getContactCount(): number;
        /**
         * Change the global gravity vector.
         * @param {org.jbox2d.common.Vec2} gravity
         */
        setGravity(gravity: org.jbox2d.common.Vec2): void;
        /**
         * Get a clone of the global gravity vector.
         * @return {org.jbox2d.common.Vec2} Clone of gravity vector
         */
        getGravity(): org.jbox2d.common.Vec2;
        /**
         * The world provides a single static ground body with no collision shapes.
         * You can use this to simplify the creation of joints and static shapes.
         * @return {org.jbox2d.dynamics.Body}
         */
        getGroundBody(): org.jbox2d.dynamics.Body;
        /**
         * Get the world body list. With the returned body, use Body.getNext() to get
         * the next body in the world list. A NULL body indicates the end of the list.
         * @return {org.jbox2d.dynamics.Body} the head of the world body list.
         */
        getBodyList(): org.jbox2d.dynamics.Body;
        /**
         * Get the world joint list. With the returned joint, use Joint.getNext() to get
         * the next joint in the world list. A NULL joint indicates the end of the list.
         * @return {org.jbox2d.dynamics.joints.Joint} the head of the world joint list.
         */
        getJointList(): org.jbox2d.dynamics.joints.Joint;
        constructor(worldAABB: org.jbox2d.collision.AABB, gravity: org.jbox2d.common.Vec2, doSleep: boolean);
        /**
         * Register a destruction listener.
         * @param {*} listener
         */
        setDestructionListener(listener: org.jbox2d.dynamics.DestructionListener): void;
        /**
         * Register a broad-phase boundary listener.
         * @param {*} listener
         */
        setBoundaryListener(listener: org.jbox2d.dynamics.BoundaryListener): void;
        /**
         * Register a contact event listener
         * @param {*} listener
         */
        setContactListener(listener: org.jbox2d.dynamics.ContactListener): void;
        /**
         * Register a contact filter to provide specific control over collision.
         * Otherwise the default filter is used (b2_defaultFilter).
         * @param {*} filter
         */
        setContactFilter(filter: org.jbox2d.dynamics.ContactFilter): void;
        /**
         * Register a routine for debug drawing. The debug draw functions are called
         * inside the World.step() method, so make sure your renderer is ready to
         * consume draw commands when you call step().
         * @param {org.jbox2d.dynamics.DebugDraw} debugDraw
         */
        setDebugDraw(debugDraw: org.jbox2d.dynamics.DebugDraw): void;
        getDebugDraw(): org.jbox2d.dynamics.DebugDraw;
        /**
         * Create a body given a definition. No reference to the definition
         * is retained.  Body will be static unless mass is nonzero.
         * <BR><em>Warning</em>: This function is locked during callbacks.
         * @param {org.jbox2d.dynamics.BodyDef} def
         * @return {org.jbox2d.dynamics.Body}
         */
        createBody(def: org.jbox2d.dynamics.BodyDef): org.jbox2d.dynamics.Body;
        /**
         * Destroy a rigid body given a definition. No reference to the definition
         * is retained. This function is locked during callbacks.
         * <BR><em>Warning</em>: This automatically deletes all associated shapes and joints.
         * <BR><em>Warning</em>: This function is locked during callbacks.
         * @param {org.jbox2d.dynamics.Body} b
         */
        destroyBody(b: org.jbox2d.dynamics.Body): void;
        /**
         * Create a joint to constrain bodies together. No reference to the definition
         * is retained. This may cause the connected bodies to cease colliding.
         * <BR><em>Warning</em> This function is locked during callbacks.
         * @param {org.jbox2d.dynamics.joints.JointDef} def
         * @return {org.jbox2d.dynamics.joints.Joint}
         */
        createJoint(def: org.jbox2d.dynamics.joints.JointDef): org.jbox2d.dynamics.joints.Joint;
        /**
         * Destroy a joint. This may cause the connected bodies to begin colliding.
         * <BR><em>Warning</em>: This function is locked during callbacks.
         * @param {org.jbox2d.dynamics.joints.Joint} j
         */
        destroyJoint(j: org.jbox2d.dynamics.joints.Joint): void;
        createController(def: org.jbox2d.dynamics.controllers.ControllerDef): org.jbox2d.dynamics.controllers.Controller;
        destroyController(controller: org.jbox2d.dynamics.controllers.Controller): void;
        static tlStep: org.jbox2d.pooling.TLTimeStep;
        static tlStep_$LI$(): org.jbox2d.pooling.TLTimeStep;
        /**
         * Take a time step. This performs collision detection, integration,
         * and constraint solution.
         * @param {number} dt the amount of time to simulate, this should not vary.
         * @param {number} iterations the number of iterations to be used by the constraint solver.
         */
        step(dt: number, iterations: number): void;
        /**
         * Goes through the registered postStep functions and calls them.
         * @param {number} dt
         * @param {number} iterations
         * @private
         */
        postStep(dt: number, iterations: number): void;
        /**
         * Registers a Steppable object to be stepped
         * immediately following the physics step, once
         * the locks are lifted.
         * @param {*} s
         */
        registerPostStep(s: org.jbox2d.dynamics.Steppable): void;
        /**
         * Unregisters a method from post-stepping.
         * Fails silently if method is not found.
         * @param {*} s
         */
        unregisterPostStep(s: org.jbox2d.dynamics.Steppable): void;
        /**
         * Re-filter a shape. This re-runs contact filtering on a shape.
         * @param {org.jbox2d.collision.shapes.Shape} shape
         */
        refilter(shape: org.jbox2d.collision.shapes.Shape): void;
        /**
         * Query the world for all shapes that potentially overlap the
         * provided AABB up to max count.
         * The number of shapes found is returned.
         * @param {org.jbox2d.collision.AABB} aabb the query box.
         * @param {number} maxCount the capacity of the shapes array.
         * @return {Array} array of shapes overlapped, up to maxCount in length
         */
        query(aabb: org.jbox2d.collision.AABB, maxCount: number): org.jbox2d.collision.shapes.Shape[];
        static islands: org.jbox2d.pooling.stacks.IslandStack;
        static islands_$LI$(): org.jbox2d.pooling.stacks.IslandStack;
        /**
         * For internal use
         * @param {org.jbox2d.dynamics.TimeStep} step
         */
        solve(step: org.jbox2d.dynamics.TimeStep): void;
        static steps: org.jbox2d.pooling.stacks.TimeStepStack;
        static steps_$LI$(): org.jbox2d.pooling.stacks.TimeStepStack;
        /**
         * For internal use: find TOI contacts and solve them.
         * @param {org.jbox2d.dynamics.TimeStep} step
         */
        solveTOI(step: org.jbox2d.dynamics.TimeStep): void;
        static LIQUID_INT: number;
        static LIQUID_INT_$LI$(): number;
        liquidLength: number;
        averageLinearVel: number;
        coreColor: org.jbox2d.common.Color3f;
        drawingCenter: org.jbox2d.common.Vec2;
        liquidOffset: org.jbox2d.common.Vec2;
        circCenterMoved: org.jbox2d.common.Vec2;
        liquidColor: org.jbox2d.common.Color3f;
        segLeft: org.jbox2d.common.Vec2;
        segRight: org.jbox2d.common.Vec2;
        /**
         * For internal use
         * @param {org.jbox2d.collision.shapes.Shape} shape
         * @param {org.jbox2d.common.XForm} xf
         * @param {org.jbox2d.common.Color3f} color
         * @param {boolean} core
         */
        drawShape(shape: org.jbox2d.collision.shapes.Shape, xf: org.jbox2d.common.XForm, color: org.jbox2d.common.Color3f, core: boolean): void;
        jointColor: org.jbox2d.common.Color3f;
        /**
         * For internal use
         * @param {org.jbox2d.dynamics.joints.Joint} joint
         */
        drawJoint(joint: org.jbox2d.dynamics.joints.Joint): void;
        staticColor: org.jbox2d.common.Color3f;
        sleepingColor: org.jbox2d.common.Color3f;
        activeColor: org.jbox2d.common.Color3f;
        pairColor: org.jbox2d.common.Color3f;
        aabbColor: org.jbox2d.common.Color3f;
        obbColor: org.jbox2d.common.Color3f;
        worldColor: org.jbox2d.common.Color3f;
        pairB1: org.jbox2d.collision.AABB;
        pairB2: org.jbox2d.collision.AABB;
        pairX1: org.jbox2d.common.Vec2;
        pairX2: org.jbox2d.common.Vec2;
        aabbB: org.jbox2d.collision.AABB;
        cornerVecs: org.jbox2d.common.Vec2[];
        /**
         * For internal use
         */
        drawDebugData(): void;
        /**
         * Enable/disable warm starting. For testing.
         * @param {boolean} flag
         */
        setWarmStarting(flag: boolean): void;
        /**
         * Enable/disable position correction. For testing.
         * @param {boolean} flag
         */
        setPositionCorrection(flag: boolean): void;
        /**
         * Enable/disable continuous physics. For testing.
         * @param {boolean} flag
         */
        setContinuousPhysics(flag: boolean): void;
        /**
         * Perform validation of internal data structures.
         */
        validate(): void;
        /**
         * Get the number of broad-phase proxies.
         * @return {number}
         */
        getProxyCount(): number;
        /**
         * Get the number of broad-phase pairs.
         * @return {number}
         */
        getPairCount(): number;
        /**
         * Get the world bounding box.
         * @return {org.jbox2d.collision.AABB}
         */
        getWorldAABB(): org.jbox2d.collision.AABB;
        /**
         * Return true if the bounding box is within range of the world AABB.
         * @param {org.jbox2d.collision.AABB} aabb
         * @return {boolean}
         */
        inRange(aabb: org.jbox2d.collision.AABB): boolean;
        m_raycastSegment: org.jbox2d.collision.Segment;
        m_raycastNormal: org.jbox2d.common.Vec2;
        m_raycastUserData: any;
        m_raycastSolidShape: boolean;
        /**
         *
         * Query the world for all fixtures that intersect a given segment. You provide a shape
         * pointer buffer of specified size. The number of shapes found is returned, and the buffer
         * is filled in order of intersection
         * @param {org.jbox2d.collision.Segment} segment defines the begin and end point of the ray cast, from p1 to p2.
         * @param {Array} shapes a user allocated shape pointer array of size maxCount (or greater).
         * @param {number} maxCount the capacity of the shapes array
         * @param {boolean} solidShapes determines if shapes that the ray starts in are counted as hits.
         * @param {*} userData passed through the worlds contact filter, with method RayCollide. This can be used to filter valid shapes
         * @return {number} the number of shapes found
         */
        raycast(segment: org.jbox2d.collision.Segment, shapes: org.jbox2d.collision.shapes.Shape[], maxCount: number, solidShapes: boolean, userData: any): number;
        /**
         *
         * Performs a ray-cast as with {@link #raycast(Segment, Shape[], int, boolean, Object)}, finding the first intersecting shape
         * @param {org.jbox2d.collision.Segment} segment defines the begin and end point of the ray cast, from p1 to p2
         * @param lambda returns the hit fraction. You can use this to compute the contact point
         * p = (1 - lambda) * segment.p1 + lambda * segment.p2.
         * @param normal returns the normal at the contact point. If there is no intersection, the normal
         * is not set.
         * @param {boolean} solidShapes determines if shapes that the ray starts in are counted as hits.
         * @returns the colliding shape shape, or null if not found
         * @see #raycast(Segment, Shape[], int, boolean, Object)
         * @param {org.jbox2d.common.RaycastResult} result
         * @param {*} userData
         * @return {org.jbox2d.collision.shapes.Shape}
         */
        raycastOne(segment: org.jbox2d.collision.Segment, result: org.jbox2d.common.RaycastResult, solidShapes: boolean, userData: any): org.jbox2d.collision.shapes.Shape;
        raycastSortKey: org.jbox2d.collision.SortKeyFunc;
        raycastSortKeyFunc(data: any): number;
    }
    namespace World {
        class World$0 implements org.jbox2d.collision.SortKeyFunc {
            __parent: any;
            apply(shape: any): number;
            constructor(__parent: any);
        }
    }
}
declare namespace org.jbox2d.collision.shapes {
    /**
     * this is used internally, instead use {@link Body#createShape(ShapeDef)}
     * with a {@link CircleDef}
     *
     * @see Body#createShape(ShapeDef)
     * @see CircleDef
     * @param {org.jbox2d.collision.shapes.ShapeDef} def
     * @class
     * @extends org.jbox2d.collision.shapes.Shape
     */
    class CircleShape extends org.jbox2d.collision.shapes.Shape {
        m_radius: number;
        m_localPosition: org.jbox2d.common.Vec2;
        constructor(def: org.jbox2d.collision.shapes.ShapeDef);
        /**
         * @see Shape#updateSweepRadius(Vec2)
         * @param {org.jbox2d.common.Vec2} center
         */
        updateSweepRadius(center: org.jbox2d.common.Vec2): void;
        static tlCenter: org.jbox2d.pooling.TLVec2;
        static tlCenter_$LI$(): org.jbox2d.pooling.TLVec2;
        /**
         * checks to see if the point is in this shape.
         *
         * @see Shape#testPoint(XForm, Vec2)
         * @param {org.jbox2d.common.XForm} transform
         * @param {org.jbox2d.common.Vec2} p
         * @return {boolean}
         */
        testPoint(transform: org.jbox2d.common.XForm, p: org.jbox2d.common.Vec2): boolean;
        static tlS: org.jbox2d.pooling.TLVec2;
        static tlS_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlPosition: org.jbox2d.pooling.TLVec2;
        static tlPosition_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlR: org.jbox2d.pooling.TLVec2;
        static tlR_$LI$(): org.jbox2d.pooling.TLVec2;
        /**
         * @see Shape#testSegment(XForm, RaycastResult, Segment, float)
         * @param {org.jbox2d.common.XForm} xf
         * @param {org.jbox2d.common.RaycastResult} out
         * @param {org.jbox2d.collision.Segment} segment
         * @param {number} maxLambda
         * @return {org.jbox2d.collision.SegmentCollide}
         */
        testSegment(xf: org.jbox2d.common.XForm, out: org.jbox2d.common.RaycastResult, segment: org.jbox2d.collision.Segment, maxLambda: number): org.jbox2d.collision.SegmentCollide;
        static tlP: org.jbox2d.pooling.TLVec2;
        static tlP_$LI$(): org.jbox2d.pooling.TLVec2;
        /**
         * @see Shape#computeAABB(AABB, XForm)
         * @param {org.jbox2d.collision.AABB} aabb
         * @param {org.jbox2d.common.XForm} transform
         */
        computeAABB(aabb: org.jbox2d.collision.AABB, transform: org.jbox2d.common.XForm): void;
        /**
         * @see Shape#computeSweptAABB(AABB, XForm, XForm)
         * @param {org.jbox2d.collision.AABB} aabb
         * @param {org.jbox2d.common.XForm} transform1
         * @param {org.jbox2d.common.XForm} transform2
         */
        computeSweptAABB(aabb: org.jbox2d.collision.AABB, transform1: org.jbox2d.common.XForm, transform2: org.jbox2d.common.XForm): void;
        /**
         * @see Shape#computeMass(MassData)
         * @param {org.jbox2d.collision.MassData} massData
         */
        computeMass(massData: org.jbox2d.collision.MassData): void;
        getRadius(): number;
        /**
         * Returns a copy of the local position
         *
         * @return
         * @return {org.jbox2d.common.Vec2}
         */
        getLocalPosition(): org.jbox2d.common.Vec2;
        /**
         * Returns the member variable of the local position. Don't change this.
         *
         * @return
         * @return {org.jbox2d.common.Vec2}
         */
        getMemberLocalPosition(): org.jbox2d.common.Vec2;
        computeSubmergedArea$org_jbox2d_common_Vec2$float$org_jbox2d_common_XForm$org_jbox2d_common_Vec2(normal: org.jbox2d.common.Vec2, offset: number, xf: org.jbox2d.common.XForm, c: org.jbox2d.common.Vec2): number;
        /**
         * @see Shape#computeSubmergedArea(Vec2, float, XForm, Vec2)
         * @param {org.jbox2d.common.Vec2} normal
         * @param {number} offset
         * @param {org.jbox2d.common.XForm} xf
         * @param {org.jbox2d.common.Vec2} c
         * @return {number}
         */
        computeSubmergedArea(normal?: any, offset?: any, xf?: any, c?: any): any;
    }
}
declare namespace org.jbox2d.collision.shapes {
    /**
     * Don't use this.  Instead create using {@link Body#createShape(ShapeDef)} with an
     * {@link EdgeChainDef}, not the constructor here.
     * @see Body#createShape(ShapeDef)
     * @see EdgeChainDef
     * @param {org.jbox2d.common.Vec2} v1
     * @param {org.jbox2d.common.Vec2} v2
     * @param {org.jbox2d.collision.shapes.ShapeDef} def
     * @class
     * @extends org.jbox2d.collision.shapes.Shape
     * @author daniel
     */
    class EdgeShape extends org.jbox2d.collision.shapes.Shape implements org.jbox2d.collision.SupportsGenericDistance {
        m_v1: org.jbox2d.common.Vec2;
        m_v2: org.jbox2d.common.Vec2;
        m_coreV1: org.jbox2d.common.Vec2;
        m_coreV2: org.jbox2d.common.Vec2;
        m_length: number;
        m_normal: org.jbox2d.common.Vec2;
        m_direction: org.jbox2d.common.Vec2;
        m_cornerDir1: org.jbox2d.common.Vec2;
        m_cornerDir2: org.jbox2d.common.Vec2;
        m_cornerConvex1: boolean;
        m_cornerConvex2: boolean;
        m_nextEdge: EdgeShape;
        m_prevEdge: EdgeShape;
        constructor(v1: org.jbox2d.common.Vec2, v2: org.jbox2d.common.Vec2, def: org.jbox2d.collision.shapes.ShapeDef);
        /**
         * @see Shape#updateSweepRadius(Vec2)
         * @param {org.jbox2d.common.Vec2} center
         */
        updateSweepRadius(center: org.jbox2d.common.Vec2): void;
        /**
         * @see Shape#testPoint(XForm, Vec2)
         * @param {org.jbox2d.common.XForm} transform
         * @param {org.jbox2d.common.Vec2} p
         * @return {boolean}
         */
        testPoint(transform: org.jbox2d.common.XForm, p: org.jbox2d.common.Vec2): boolean;
        static tlR: org.jbox2d.pooling.TLVec2;
        static tlR_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlV1: org.jbox2d.pooling.TLVec2;
        static tlV1_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlD: org.jbox2d.pooling.TLVec2;
        static tlD_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlN: org.jbox2d.pooling.TLVec2;
        static tlN_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlB: org.jbox2d.pooling.TLVec2;
        static tlB_$LI$(): org.jbox2d.pooling.TLVec2;
        /**
         * @see Shape#testSegment(XForm, RaycastResult, Segment, float)
         * @param {org.jbox2d.common.XForm} xf
         * @param {org.jbox2d.common.RaycastResult} out
         * @param {org.jbox2d.collision.Segment} segment
         * @param {number} maxLambda
         * @return {org.jbox2d.collision.SegmentCollide}
         */
        testSegment(xf: org.jbox2d.common.XForm, out: org.jbox2d.common.RaycastResult, segment: org.jbox2d.collision.Segment, maxLambda: number): org.jbox2d.collision.SegmentCollide;
        static tlV2: org.jbox2d.pooling.TLVec2;
        static tlV2_$LI$(): org.jbox2d.pooling.TLVec2;
        /**
         * @see Shape#computeAABB(AABB, XForm)
         * @param {org.jbox2d.collision.AABB} aabb
         * @param {org.jbox2d.common.XForm} transform
         */
        computeAABB(aabb: org.jbox2d.collision.AABB, transform: org.jbox2d.common.XForm): void;
        static tlSwept1: org.jbox2d.pooling.TLVec2;
        static tlSwept1_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlSwept2: org.jbox2d.pooling.TLVec2;
        static tlSwept2_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlSwept3: org.jbox2d.pooling.TLVec2;
        static tlSwept3_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlSwept4: org.jbox2d.pooling.TLVec2;
        static tlSwept4_$LI$(): org.jbox2d.pooling.TLVec2;
        /**
         * @see Shape#computeSweptAABB(AABB, XForm, XForm)
         * @param {org.jbox2d.collision.AABB} aabb
         * @param {org.jbox2d.common.XForm} transform1
         * @param {org.jbox2d.common.XForm} transform2
         */
        computeSweptAABB(aabb: org.jbox2d.collision.AABB, transform1: org.jbox2d.common.XForm, transform2: org.jbox2d.common.XForm): void;
        /**
         * @see Shape#computeMass(MassData)
         * @param {org.jbox2d.collision.MassData} massData
         */
        computeMass(massData: org.jbox2d.collision.MassData): void;
        static tlSupportV1: org.jbox2d.pooling.TLVec2;
        static tlSupportV1_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlSupportV2: org.jbox2d.pooling.TLVec2;
        static tlSupportV2_$LI$(): org.jbox2d.pooling.TLVec2;
        /**
         * @see SupportsGenericDistance#support(Vec2, XForm, Vec2)
         * @param {org.jbox2d.common.Vec2} dest
         * @param {org.jbox2d.common.XForm} xf
         * @param {org.jbox2d.common.Vec2} d
         */
        support(dest: org.jbox2d.common.Vec2, xf: org.jbox2d.common.XForm, d: org.jbox2d.common.Vec2): void;
        setPrevEdge(edge: EdgeShape, core: org.jbox2d.common.Vec2, cornerDir: org.jbox2d.common.Vec2, convex: boolean): void;
        setNextEdge(edge: EdgeShape, core: org.jbox2d.common.Vec2, cornerDir: org.jbox2d.common.Vec2, convex: boolean): void;
        /**
         * Linear distance from vertex1 to vertex2
         * @return {number}
         */
        getLength(): number;
        /**
         * Local position of vertex in parent body
         * @return {org.jbox2d.common.Vec2}
         */
        getVertex1(): org.jbox2d.common.Vec2;
        /**
         * Local position of vertex in parent body
         * @return {org.jbox2d.common.Vec2}
         */
        getVertex2(): org.jbox2d.common.Vec2;
        /**
         * "Core" vertex with TOI slop for b2Distance functions
         * @return {org.jbox2d.common.Vec2}
         */
        getCoreVertex1(): org.jbox2d.common.Vec2;
        /**
         * "Core" vertex with TOI slop for b2Distance functions
         * @return {org.jbox2d.common.Vec2}
         */
        getCoreVertex2(): org.jbox2d.common.Vec2;
        /**
         * Perpendecular unit vector point, pointing from the solid side to the empty side.
         * @return {org.jbox2d.common.Vec2}
         */
        getNormalVector(): org.jbox2d.common.Vec2;
        /**
         * Parallel unit vector, pointing from vertex1 to vertex2
         * @return {org.jbox2d.common.Vec2}
         */
        getDirectionVector(): org.jbox2d.common.Vec2;
        getCorner1Vector(): org.jbox2d.common.Vec2;
        getCorner2Vector(): org.jbox2d.common.Vec2;
        /**
         * Get the next edge in the chain.
         * @return {org.jbox2d.collision.shapes.EdgeShape}
         */
        getNextEdge(): EdgeShape;
        /**
         * Get the previous edge in the chain.
         * @return {org.jbox2d.collision.shapes.EdgeShape}
         */
        getPrevEdge(): EdgeShape;
        /**
         * @see SupportsGenericDistance#getFirstVertexToOut(XForm, Vec2)
         * @param {org.jbox2d.common.XForm} xf
         * @param {org.jbox2d.common.Vec2} out
         */
        getFirstVertexToOut(xf: org.jbox2d.common.XForm, out: org.jbox2d.common.Vec2): void;
        corner1IsConvex(): boolean;
        corner2IsConvex(): boolean;
        static tlV0: org.jbox2d.pooling.TLVec2;
        static tlV0_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlTemp: org.jbox2d.pooling.TLVec2;
        static tlTemp_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlE1: org.jbox2d.pooling.TLVec2;
        static tlE1_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlE2: org.jbox2d.pooling.TLVec2;
        static tlE2_$LI$(): org.jbox2d.pooling.TLVec2;
        computeSubmergedArea$org_jbox2d_common_Vec2$float$org_jbox2d_common_XForm$org_jbox2d_common_Vec2(normal: org.jbox2d.common.Vec2, offset: number, xf: org.jbox2d.common.XForm, c: org.jbox2d.common.Vec2): number;
        computeSubmergedArea(normal?: any, offset?: any, xf?: any, c?: any): any;
    }
}
declare namespace org.jbox2d.collision.shapes {
    /**
     * Point shape.  Like a circle shape of zero radius, except
     * that it has a finite mass.
     * @param {org.jbox2d.collision.shapes.ShapeDef} def
     * @class
     * @extends org.jbox2d.collision.shapes.Shape
     */
    class PointShape extends org.jbox2d.collision.shapes.Shape {
        m_localPosition: org.jbox2d.common.Vec2;
        m_mass: number;
        constructor(def: org.jbox2d.collision.shapes.ShapeDef);
        static tlP: org.jbox2d.pooling.TLVec2;
        static tlP_$LI$(): org.jbox2d.pooling.TLVec2;
        /**
         * @see Shape#computeAABB(AABB, XForm)
         * @param {org.jbox2d.collision.AABB} aabb
         * @param {org.jbox2d.common.XForm} transform
         */
        computeAABB(aabb: org.jbox2d.collision.AABB, transform: org.jbox2d.common.XForm): void;
        /**
         * @see Shape#computeMass(MassData)
         * @param {org.jbox2d.collision.MassData} massData
         */
        computeMass(massData: org.jbox2d.collision.MassData): void;
        static tlSwept1: org.jbox2d.pooling.TLVec2;
        static tlSwept1_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlSwept2: org.jbox2d.pooling.TLVec2;
        static tlSwept2_$LI$(): org.jbox2d.pooling.TLVec2;
        /**
         * @see Shape#computeSweptAABB(AABB, XForm, XForm)
         * @param {org.jbox2d.collision.AABB} aabb
         * @param {org.jbox2d.common.XForm} transform1
         * @param {org.jbox2d.common.XForm} transform2
         */
        computeSweptAABB(aabb: org.jbox2d.collision.AABB, transform1: org.jbox2d.common.XForm, transform2: org.jbox2d.common.XForm): void;
        /**
         * @see Shape#testPoint(XForm, Vec2)
         * @param {org.jbox2d.common.XForm} xf
         * @param {org.jbox2d.common.Vec2} p
         * @return {boolean}
         */
        testPoint(xf: org.jbox2d.common.XForm, p: org.jbox2d.common.Vec2): boolean;
        static tlS: org.jbox2d.pooling.TLVec2;
        static tlS_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlPosition: org.jbox2d.pooling.TLVec2;
        static tlPosition_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlR: org.jbox2d.pooling.TLVec2;
        static tlR_$LI$(): org.jbox2d.pooling.TLVec2;
        /**
         * @see Shape#testSegment(XForm, RaycastResult, Segment, float)
         * @param {org.jbox2d.common.XForm} xf
         * @param {org.jbox2d.common.RaycastResult} out
         * @param {org.jbox2d.collision.Segment} segment
         * @param {number} maxLambda
         * @return {org.jbox2d.collision.SegmentCollide}
         */
        testSegment(xf: org.jbox2d.common.XForm, out: org.jbox2d.common.RaycastResult, segment: org.jbox2d.collision.Segment, maxLambda: number): org.jbox2d.collision.SegmentCollide;
        /**
         * @see Shape#updateSweepRadius(Vec2)
         * @param {org.jbox2d.common.Vec2} center
         */
        updateSweepRadius(center: org.jbox2d.common.Vec2): void;
        /**
         * @return {org.jbox2d.common.Vec2} a copy of local position
         */
        getLocalPosition(): org.jbox2d.common.Vec2;
        /**
         * This is the member variable for the local position.
         * Don't change this.
         * @return
         * @return {org.jbox2d.common.Vec2}
         */
        getMemberLocalPosition(): org.jbox2d.common.Vec2;
        getMass(): number;
    }
}
declare namespace org.jbox2d.collision.shapes {
    /**
     * A convex polygon shape.  Create using Body.createShape(ShapeDef), not the constructor here.
     * @param {org.jbox2d.collision.shapes.ShapeDef} def
     * @class
     * @extends org.jbox2d.collision.shapes.Shape
     */
    class PolygonShape extends org.jbox2d.collision.shapes.Shape implements org.jbox2d.collision.SupportsGenericDistance {
        /**
         * Dump lots of debug information.
         */
        static m_debug: boolean;
        /**
         * Local position of the shape centroid in parent body frame.
         */
        m_centroid: org.jbox2d.common.Vec2;
        /**
         * The oriented bounding box of the shape.
         */
        m_obb: org.jbox2d.collision.OBB;
        /**
         * The vertices of the shape.  Note: use getVertexCount(), not m_vertices.length, to get number of active vertices.
         */
        m_vertices: org.jbox2d.common.Vec2[];
        /**
         * The normals of the shape.  Note: use getVertexCount(), not m_normals.length, to get number of active normals.
         */
        m_normals: org.jbox2d.common.Vec2[];
        /**
         * The normals of the shape.  Note: use getVertexCount(), not m_coreVertices.length, to get number of active vertices.
         */
        m_coreVertices: org.jbox2d.common.Vec2[];
        /**
         * Number of active vertices in the shape.
         */
        m_vertexCount: number;
        static tlEdge: org.jbox2d.pooling.TLVec2;
        static tlEdge_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlV: org.jbox2d.pooling.TLVec2;
        static tlV_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlD: org.jbox2d.pooling.TLVec2;
        static tlD_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlA: org.jbox2d.pooling.TLMat22;
        static tlA_$LI$(): org.jbox2d.pooling.TLMat22;
        constructor(def: org.jbox2d.collision.shapes.ShapeDef);
        /**
         * @see Shape#updateSweepRadius(Vec2)
         * @param {org.jbox2d.common.Vec2} center
         */
        updateSweepRadius(center: org.jbox2d.common.Vec2): void;
        static tlTemp: org.jbox2d.pooling.TLVec2;
        static tlTemp_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlPLocal: org.jbox2d.pooling.TLVec2;
        static tlPLocal_$LI$(): org.jbox2d.pooling.TLVec2;
        /**
         * @see Shape#testPoint(XForm, Vec2)
         * @param {org.jbox2d.common.XForm} xf
         * @param {org.jbox2d.common.Vec2} p
         * @return {boolean}
         */
        testPoint(xf: org.jbox2d.common.XForm, p: org.jbox2d.common.Vec2): boolean;
        static tlP1: org.jbox2d.pooling.TLVec2;
        static tlP1_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlP2: org.jbox2d.pooling.TLVec2;
        static tlP2_$LI$(): org.jbox2d.pooling.TLVec2;
        /**
         * @see Shape#testSegment(XForm, RaycastResult, Segment, float)
         * @param {org.jbox2d.common.XForm} xf
         * @param {org.jbox2d.common.RaycastResult} out
         * @param {org.jbox2d.collision.Segment} segment
         * @param {number} maxLambda
         * @return {org.jbox2d.collision.SegmentCollide}
         */
        testSegment(xf: org.jbox2d.common.XForm, out: org.jbox2d.common.RaycastResult, segment: org.jbox2d.collision.Segment, maxLambda: number): org.jbox2d.collision.SegmentCollide;
        static tlSupDLocal: org.jbox2d.pooling.TLVec2;
        static tlSupDLocal_$LI$(): org.jbox2d.pooling.TLVec2;
        /**
         * Get the support point in the given world direction.
         * Use the supplied transform.
         * @see SupportsGenericDistance#support(Vec2, XForm, Vec2)
         * @param {org.jbox2d.common.Vec2} dest
         * @param {org.jbox2d.common.XForm} xf
         * @param {org.jbox2d.common.Vec2} d
         */
        support(dest: org.jbox2d.common.Vec2, xf: org.jbox2d.common.XForm, d: org.jbox2d.common.Vec2): void;
        static tlPRef: org.jbox2d.pooling.TLVec2;
        static tlPRef_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlE1: org.jbox2d.pooling.TLVec2;
        static tlE1_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlE2: org.jbox2d.pooling.TLVec2;
        static tlE2_$LI$(): org.jbox2d.pooling.TLVec2;
        static computeCentroid(vs: Array<org.jbox2d.common.Vec2>): org.jbox2d.common.Vec2;
        static tlUX: org.jbox2d.pooling.TLVec2;
        static tlUX_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlUY: org.jbox2d.pooling.TLVec2;
        static tlUY_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlLower: org.jbox2d.pooling.TLVec2;
        static tlLower_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlUpper: org.jbox2d.pooling.TLVec2;
        static tlUpper_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlR: org.jbox2d.pooling.TLVec2;
        static tlR_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlCenter: org.jbox2d.pooling.TLVec2;
        static tlCenter_$LI$(): org.jbox2d.pooling.TLVec2;
        static computeOBB(obb: org.jbox2d.collision.OBB, vs: org.jbox2d.common.Vec2[]): void;
        static tlCaabbR: org.jbox2d.pooling.TLMat22;
        static tlCaabbR_$LI$(): org.jbox2d.pooling.TLMat22;
        static tlCaabbH: org.jbox2d.pooling.TLVec2;
        static tlCaabbH_$LI$(): org.jbox2d.pooling.TLVec2;
        /**
         * @see Shape#computeAABB(AABB, XForm)
         * @param {org.jbox2d.collision.AABB} aabb
         * @param {org.jbox2d.common.XForm} xf
         */
        computeAABB(aabb: org.jbox2d.collision.AABB, xf: org.jbox2d.common.XForm): void;
        static tlSwept1: org.jbox2d.pooling.TLAABB;
        static tlSwept1_$LI$(): org.jbox2d.pooling.TLAABB;
        static tlSwept2: org.jbox2d.pooling.TLAABB;
        static tlSwept2_$LI$(): org.jbox2d.pooling.TLAABB;
        /**
         * @see Shape#computeSweptAABB(AABB, XForm, XForm)
         * @param {org.jbox2d.collision.AABB} aabb
         * @param {org.jbox2d.common.XForm} transform1
         * @param {org.jbox2d.common.XForm} transform2
         */
        computeSweptAABB(aabb: org.jbox2d.collision.AABB, transform1: org.jbox2d.common.XForm, transform2: org.jbox2d.common.XForm): void;
        computeMass$org_jbox2d_collision_MassData(massData: org.jbox2d.collision.MassData): void;
        computeMass$org_jbox2d_collision_MassData$float(massData: org.jbox2d.collision.MassData, density: number): void;
        /**
         * @see Shape#computeMass(MassData)
         * @param {org.jbox2d.collision.MassData} massData
         * @param {number} density
         */
        computeMass(massData?: any, density?: any): any;
        /**
         * Get the first vertex and apply the supplied transform.
         * @param {org.jbox2d.common.XForm} xf
         * @param {org.jbox2d.common.Vec2} out
         */
        getFirstVertexToOut(xf: org.jbox2d.common.XForm, out: org.jbox2d.common.Vec2): void;
        /**
         * Get the oriented bounding box relative to the parent body.
         * @return {org.jbox2d.collision.OBB}
         */
        getOBB(): org.jbox2d.collision.OBB;
        /**
         * Get the local centroid relative to the parent body.
         * @return {org.jbox2d.common.Vec2}
         */
        getCentroid(): org.jbox2d.common.Vec2;
        /**
         * Get the number of vertices.
         * @return {number}
         */
        getVertexCount(): number;
        /**
         * Get the vertices in local coordinates.
         * @return {Array}
         */
        getVertices(): org.jbox2d.common.Vec2[];
        /**
         * Get the core vertices in local coordinates. These vertices
         * represent a smaller polygon that is used for time of impact
         * computations.
         * @return {Array}
         */
        getCoreVertices(): org.jbox2d.common.Vec2[];
        /**
         * Get the edge normal vectors.  There is one for each vertex.
         * @return {Array}
         */
        getNormals(): org.jbox2d.common.Vec2[];
        /**
         * Get the centroid and apply the supplied transform.
         * @param {org.jbox2d.common.XForm} xf
         * @return {org.jbox2d.common.Vec2}
         */
        centroid(xf: org.jbox2d.common.XForm): org.jbox2d.common.Vec2;
        static tlNormalL: org.jbox2d.pooling.TLVec2;
        static tlNormalL_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlMd: org.jbox2d.pooling.TLMassData;
        static tlMd_$LI$(): org.jbox2d.pooling.TLMassData;
        static tlIntoVec: org.jbox2d.pooling.TLVec2;
        static tlIntoVec_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlOutoVec: org.jbox2d.pooling.TLVec2;
        static tlOutoVec_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlP2b: org.jbox2d.pooling.TLVec2;
        static tlP2b_$LI$(): org.jbox2d.pooling.TLVec2;
        static tlP3: org.jbox2d.pooling.TLVec2;
        static tlP3_$LI$(): org.jbox2d.pooling.TLVec2;
        computeSubmergedArea$org_jbox2d_common_Vec2$float$org_jbox2d_common_XForm$org_jbox2d_common_Vec2(normal: org.jbox2d.common.Vec2, offset: number, xf: org.jbox2d.common.XForm, c: org.jbox2d.common.Vec2): number;
        /**
         * @see Shape#computeSubmergedArea(Vec2, float, XForm, Vec2)
         * @param {org.jbox2d.common.Vec2} normal
         * @param {number} offset
         * @param {org.jbox2d.common.XForm} xf
         * @param {org.jbox2d.common.Vec2} c
         * @return {number}
         */
        computeSubmergedArea(normal?: any, offset?: any, xf?: any, c?: any): any;
    }
}
declare var __Function: FunctionConstructor;
