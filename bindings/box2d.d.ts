// TypeScript definitions for Box2D Emscripten bindings
export interface Vec2 {
    x: number;
    y: number;
}

export enum BodyType {
    staticBody = 0,
    kinematicBody = 1,
    dynamicBody = 2
}

export class World {
    constructor();
    constructor(gravityX: number, gravityY: number);
    step(timeStep?: number, subStepCount?: number): void;
    isValid(): boolean;
    castRayClosest(originX: number, originY: number, translationX: number, translationY: number): any;
    setGravity(x: number, y: number): void;
    getGravity(): Vec2;
    enableSleeping(flag: boolean): void;
    enableContinuous(flag: boolean): void;
}

export class Body {
    constructor(world: World);
    constructor(world: World, x: number, y: number);
    constructor(world: World, x: number, y: number, type: BodyType);
    isValid(): boolean;
    getPosition(): Vec2;
    getAngle(): number;
    setPosition(x: number, y: number): void;
    setAngle(angle: number): void;
    applyForce(forceX: number, forceY: number, pointX: number, pointY: number): void;
    applyImpulse(impulseX: number, impulseY: number, pointX: number, pointY: number): void;
    getLinearVelocity(): Vec2;
    setLinearVelocity(velX: number, velY: number): void;
    getAngularVelocity(): number;
    setAngularVelocity(omega: number): void;
    getMass(): number;
    getInertia(): number;
    getCenterOfMass(): Vec2;
    isAwake(): boolean;
    setAwake(awake: boolean): void;
    isSleepingEnabled(): boolean;
    enableSleep(flag: boolean): void;
    getType(): BodyType;
    setType(type: BodyType): void;
    isEnabled(): boolean;
    setEnabled(flag: boolean): void;
}

export class ShapeFactory {
    static createBox(body: Body, halfWidth: number, halfHeight: number, density?: number, friction?: number): void;
    static createCircle(body: Body, radius: number, density?: number, friction?: number): void;
    static createCapsule(body: Body, height: number, radius: number, density?: number, friction?: number): void;
    static createSegment(body: Body, x1: number, y1: number, x2: number, y2: number, friction?: number): void;
    static createPolygon(body: Body, points: Array<Vec2>, density?: number, friction?: number): void;
}

export class Joint {
    isValid(): boolean;
    getConstraintForce(): Vec2;
    getConstraintTorque(): number;
    wakeBodies(): void;
    setCollideConnected(shouldCollide: boolean): void;
    getCollideConnected(): boolean;
}

export class DistanceJoint extends Joint {
    constructor(world: World, bodyA: Body, bodyB: Body, ax: number, ay: number, bx: number, by: number);
}

export class RevoluteJoint extends Joint {
    constructor(world: World, bodyA: Body, bodyB: Body, anchorX: number, anchorY: number);
}

export class PrismaticJoint extends Joint {
    constructor(world: World, bodyA: Body, bodyB: Body, anchorX: number, anchorY: number, axisX: number, axisY: number);
}

export class MouseJoint extends Joint {
    constructor(world: World, body: Body, targetX: number, targetY: number);
    setTarget(x: number, y: number): void;
}

export class WeldJoint extends Joint {
    constructor(world: World, bodyA: Body, bodyB: Body, anchorX: number, anchorY: number);
}

interface Box2DModule {
    World: typeof World;
    Body: typeof Body;
    ShapeFactory: typeof ShapeFactory;
    Joint: typeof Joint;
    DistanceJoint: typeof DistanceJoint;
    RevoluteJoint: typeof RevoluteJoint;
    PrismaticJoint: typeof PrismaticJoint;
    MouseJoint: typeof MouseJoint;
    WeldJoint: typeof WeldJoint;
    BodyType: typeof BodyType;
}

declare function Box2D(): Promise<Box2DModule>;
export default Box2D;