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
}

export class ShapeFactory {
    static createBox(body: Body, halfWidth: number, halfHeight: number, density?: number, friction?: number): void;
    static createCircle(body: Body, radius: number, density?: number, friction?: number): void;
}

interface Box2DModule {
    World: typeof World;
    Body: typeof Body;
    ShapeFactory: typeof ShapeFactory;
    BodyType: typeof BodyType;
}

declare function Box2D(): Promise<Box2DModule>;
export default Box2D;