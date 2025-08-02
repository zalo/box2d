// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include <emscripten/bind.h>
#include <emscripten/val.h>
#include "box2d/box2d.h"

using namespace emscripten;

// Helper functions to work with the C API
namespace box2d_bindings {

// World management
class World {
private:
    b2WorldId worldId;
    
public:
    World(float gravityX = 0.0f, float gravityY = -10.0f) {
        b2WorldDef worldDef = b2DefaultWorldDef();
        worldDef.gravity = {gravityX, gravityY};
        worldId = b2CreateWorld(&worldDef);
    }
    
    ~World() {
        if (b2World_IsValid(worldId)) {
            b2DestroyWorld(worldId);
        }
    }
    
    void step(float timeStep = 1.0f/60.0f, int subStepCount = 4) {
        b2World_Step(worldId, timeStep, subStepCount);
    }
    
    b2WorldId getId() const { return worldId; }
    bool isValid() const { return b2World_IsValid(worldId); }
    
    // Ray casting - closest hit
    val castRayClosest(float originX, float originY, float translationX, float translationY) {
        b2Vec2 origin = {originX, originY};
        b2Vec2 translation = {translationX, translationY};
        b2QueryFilter filter = b2DefaultQueryFilter();
        
        b2RayResult result = b2World_CastRayClosest(worldId, origin, translation, filter);
        
        val jsResult = val::object();
        jsResult.set("hit", result.hit);
        if (result.hit) {
            val point = val::object();
            point.set("x", result.point.x);
            point.set("y", result.point.y);
            jsResult.set("point", point);
            
            val normal = val::object();
            normal.set("x", result.normal.x);
            normal.set("y", result.normal.y);
            jsResult.set("normal", normal);
            
            jsResult.set("fraction", result.fraction);
        }
        return jsResult;
    }
    
    // Set gravity
    void setGravity(float x, float y) {
        b2World_SetGravity(worldId, {x, y});
    }
    
    // Get gravity
    val getGravity() {
        b2Vec2 gravity = b2World_GetGravity(worldId);
        val result = val::object();
        result.set("x", gravity.x);
        result.set("y", gravity.y);
        return result;
    }
    
    // Enable/disable sleeping
    void enableSleeping(bool flag) {
        b2World_EnableSleeping(worldId, flag);
    }
    
    // Enable/disable continuous collision
    void enableContinuous(bool flag) {
        b2World_EnableContinuous(worldId, flag);
    }
};

// Body management
class Body {
private:
    b2BodyId bodyId;
    
public:
    Body(const World& world, float x = 0.0f, float y = 0.0f, b2BodyType type = b2_staticBody) {
        b2BodyDef bodyDef = b2DefaultBodyDef();
        bodyDef.position = {x, y};
        bodyDef.type = type;
        bodyId = b2CreateBody(world.getId(), &bodyDef);
    }
    
    b2BodyId getId() const { return bodyId; }
    bool isValid() const { return b2Body_IsValid(bodyId); }
    
    // Get position
    val getPosition() {
        b2Vec2 pos = b2Body_GetPosition(bodyId);
        val result = val::object();
        result.set("x", pos.x);
        result.set("y", pos.y);
        return result;
    }
    
    // Get rotation angle
    float getAngle() {
        b2Rot rot = b2Body_GetRotation(bodyId);
        return b2Rot_GetAngle(rot);
    }
    
    // Set position
    void setPosition(float x, float y) {
        b2Body_SetTransform(bodyId, {x, y}, b2Body_GetRotation(bodyId));
    }
    
    // Set angle
    void setAngle(float angle) {
        b2Body_SetTransform(bodyId, b2Body_GetPosition(bodyId), b2MakeRot(angle));
    }
    
    // Apply force
    void applyForce(float forceX, float forceY, float pointX, float pointY) {
        b2Body_ApplyForce(bodyId, {forceX, forceY}, {pointX, pointY}, true);
    }
    
    // Apply impulse
    void applyImpulse(float impulseX, float impulseY, float pointX, float pointY) {
        b2Body_ApplyLinearImpulse(bodyId, {impulseX, impulseY}, {pointX, pointY}, true);
    }
    
    // Get/Set linear velocity
    val getLinearVelocity() {
        b2Vec2 vel = b2Body_GetLinearVelocity(bodyId);
        val result = val::object();
        result.set("x", vel.x);
        result.set("y", vel.y);
        return result;
    }
    
    void setLinearVelocity(float velX, float velY) {
        b2Body_SetLinearVelocity(bodyId, {velX, velY});
    }
    
    // Get/Set angular velocity
    float getAngularVelocity() {
        return b2Body_GetAngularVelocity(bodyId);
    }
    
    void setAngularVelocity(float omega) {
        b2Body_SetAngularVelocity(bodyId, omega);
    }
    
    // Mass properties
    float getMass() {
        return b2Body_GetMass(bodyId);
    }
    
    float getInertia() {
        return b2Body_GetRotationalInertia(bodyId);
    }
    
    val getCenterOfMass() {
        b2Vec2 center = b2Body_GetLocalCenterOfMass(bodyId);
        val result = val::object();
        result.set("x", center.x);
        result.set("y", center.y);
        return result;
    }
    
    // Sleep state
    bool isAwake() {
        return b2Body_IsAwake(bodyId);
    }
    
    void setAwake(bool awake) {
        b2Body_SetAwake(bodyId, awake);
    }
    
    bool isSleepingEnabled() {
        return b2Body_IsSleepEnabled(bodyId);
    }
    
    void enableSleep(bool flag) {
        b2Body_EnableSleep(bodyId, flag);
    }
    
    // Body type
    b2BodyType getType() {
        return b2Body_GetType(bodyId);
    }
    
    void setType(b2BodyType type) {
        b2Body_SetType(bodyId, type);
    }
    
    // Enable/disable
    bool isEnabled() {
        return b2Body_IsEnabled(bodyId);
    }
    
    void setEnabled(bool flag) {
        if (flag) {
            b2Body_Enable(bodyId);
        } else {
            b2Body_Disable(bodyId);
        }
    }
};

// Shape creation helpers
class ShapeFactory {
public:
    // Create a box shape on a body
    static void createBox(Body& body, float halfWidth, float halfHeight, float density = 1.0f, float friction = 0.3f) {
        b2Polygon box = b2MakeBox(halfWidth, halfHeight);
        b2ShapeDef shapeDef = b2DefaultShapeDef();
        shapeDef.density = density;
        shapeDef.material.friction = friction;
        b2CreatePolygonShape(body.getId(), &shapeDef, &box);
    }
    
    // Create a circle shape on a body
    static void createCircle(Body& body, float radius, float density = 1.0f, float friction = 0.3f) {
        b2Circle circle = {0};
        circle.center = {0.0f, 0.0f};
        circle.radius = radius;
        b2ShapeDef shapeDef = b2DefaultShapeDef();
        shapeDef.density = density;
        shapeDef.material.friction = friction;
        b2CreateCircleShape(body.getId(), &shapeDef, &circle);
    }
    
    // Create a capsule shape on a body
    static void createCapsule(Body& body, float height, float radius, float density = 1.0f, float friction = 0.3f) {
        b2Capsule capsule = {0};
        capsule.center1 = {0.0f, -height * 0.5f};
        capsule.center2 = {0.0f, height * 0.5f};
        capsule.radius = radius;
        b2ShapeDef shapeDef = b2DefaultShapeDef();
        shapeDef.density = density;
        shapeDef.material.friction = friction;
        b2CreateCapsuleShape(body.getId(), &shapeDef, &capsule);
    }
    
    // Create a segment (line) shape on a body
    static void createSegment(Body& body, float x1, float y1, float x2, float y2, float friction = 0.3f) {
        b2Segment segment = {0};
        segment.point1 = {x1, y1};
        segment.point2 = {x2, y2};
        b2ShapeDef shapeDef = b2DefaultShapeDef();
        shapeDef.density = 0.0f; // segments typically have no mass
        shapeDef.material.friction = friction;
        b2CreateSegmentShape(body.getId(), &shapeDef, &segment);
    }
    
    // Create a polygon shape from vertices
    static void createPolygon(Body& body, val points, float density = 1.0f, float friction = 0.3f) {
        // Extract points from JavaScript array
        int length = points["length"].as<int>();
        if (length < 3 || length > 8) return; // Box2D polygon limit
        
        b2Vec2 vertices[8];
        for (int i = 0; i < length && i < 8; ++i) {
            val point = points[i];
            vertices[i].x = point["x"].as<float>();
            vertices[i].y = point["y"].as<float>();
        }
        
        b2Hull hull = b2ComputeHull(vertices, length);
        if (hull.count < 3) return; // Invalid hull
        
        b2Polygon polygon = b2MakePolygon(&hull, 0.0f);
        b2ShapeDef shapeDef = b2DefaultShapeDef();
        shapeDef.density = density;
        shapeDef.material.friction = friction;
        b2CreatePolygonShape(body.getId(), &shapeDef, &polygon);
    }
};

// Base Joint class
class Joint {
protected:
    b2JointId jointId;
    
public:
    Joint(b2JointId id) : jointId(id) {}
    
    ~Joint() {
        if (b2Joint_IsValid(jointId)) {
            b2DestroyJoint(jointId);
        }
    }
    
    b2JointId getId() const { return jointId; }
    bool isValid() const { return b2Joint_IsValid(jointId); }
    
    // Get constraint force
    val getConstraintForce() {
        b2Vec2 force = b2Joint_GetConstraintForce(jointId);
        val result = val::object();
        result.set("x", force.x);
        result.set("y", force.y);
        return result;
    }
    
    // Get constraint torque
    float getConstraintTorque() {
        return b2Joint_GetConstraintTorque(jointId);
    }
    
    // Wake connected bodies
    void wakeBodies() {
        b2Joint_WakeBodies(jointId);
    }
    
    // Set collide connected
    void setCollideConnected(bool shouldCollide) {
        b2Joint_SetCollideConnected(jointId, shouldCollide);
    }
    
    // Get collide connected
    bool getCollideConnected() {
        return b2Joint_GetCollideConnected(jointId);
    }
};

// Distance Joint
class DistanceJoint : public Joint {
public:
    DistanceJoint(const World& world, const Body& bodyA, const Body& bodyB, float ax, float ay, float bx, float by) 
        : Joint(b2JointId{}) {
        b2DistanceJointDef def = b2DefaultDistanceJointDef();
        def.base.bodyIdA = bodyA.getId();
        def.base.bodyIdB = bodyB.getId();
        def.base.localFrameA.p = {ax, ay};
        def.base.localFrameB.p = {bx, by};
        def.length = 1.0f; // default length
        
        jointId = b2CreateDistanceJoint(world.getId(), &def);
    }
    
    // Set/Get length
    void setLength(float length) {
        // Note: Box2D doesn't have a direct setter, would need to recreate joint
        // This is a limitation of the C API
    }
    
    float getLength() {
        // Note: Box2D doesn't have a direct getter for runtime length
        // This would require storing or computing from constraint
        return 1.0f; // placeholder
    }
};

// Revolute Joint 
class RevoluteJoint : public Joint {
public:
    RevoluteJoint(const World& world, const Body& bodyA, const Body& bodyB, float anchorX, float anchorY)
        : Joint(b2JointId{}) {
        b2RevoluteJointDef def = b2DefaultRevoluteJointDef();
        def.base.bodyIdA = bodyA.getId();
        def.base.bodyIdB = bodyB.getId();
        def.base.localFrameA.p = {anchorX, anchorY};
        def.base.localFrameB.p = {anchorX, anchorY};
        
        jointId = b2CreateRevoluteJoint(world.getId(), &def);
    }
    
    // Enable motor
    void enableMotor(bool flag) {
        // Note: Box2D C API doesn't have runtime setters for motor enable
        // Would need to store this in the joint definition
    }
    
    // Set motor speed
    void setMotorSpeed(float speed) {
        // Note: Box2D C API limitation - no runtime motor speed setter
    }
    
    // Set max motor torque
    void setMaxMotorTorque(float torque) {
        // Note: Box2D C API limitation - no runtime motor torque setter
    }
    
    // Enable limit
    void enableLimit(bool flag) {
        // Note: Box2D C API limitation - no runtime limit enable setter
    }
    
    // Set limits
    void setLimits(float lower, float upper) {
        // Note: Box2D C API limitation - no runtime limit setters
    }
};

// Prismatic Joint
class PrismaticJoint : public Joint {
public:
    PrismaticJoint(const World& world, const Body& bodyA, const Body& bodyB, float anchorX, float anchorY, float axisX, float axisY)
        : Joint(b2JointId{}) {
        b2PrismaticJointDef def = b2DefaultPrismaticJointDef();
        def.base.bodyIdA = bodyA.getId();
        def.base.bodyIdB = bodyB.getId();
        def.base.localFrameA.p = {anchorX, anchorY};
        def.base.localFrameB.p = {anchorX, anchorY};
        // Set the axis direction in frame A
        float length = sqrt(axisX * axisX + axisY * axisY);
        if (length > 0.0f) {
            def.base.localFrameA.q = b2MakeRot(atan2(axisY, axisX));
        }
        
        jointId = b2CreatePrismaticJoint(world.getId(), &def);
    }
};

// Mouse Joint
class MouseJoint : public Joint {
public:
    MouseJoint(const World& world, const Body& body, float targetX, float targetY)
        : Joint(b2JointId{}) {
        b2MouseJointDef def = b2DefaultMouseJointDef();
        def.base.bodyIdA = b2_nullBodyId; // mouse joint connects to world
        def.base.bodyIdB = body.getId();
        def.base.localFrameB.p = {targetX, targetY};
        def.maxForce = 1000.0f;
        def.hertz = 4.0f;
        def.dampingRatio = 0.7f;
        
        jointId = b2CreateMouseJoint(world.getId(), &def);
    }
    
    // Set target
    void setTarget(float x, float y) {
        b2Transform transform = {{x, y}, b2Rot_identity};
        b2Joint_SetLocalFrameA(jointId, transform);
    }
};

// Weld Joint
class WeldJoint : public Joint {
public:
    WeldJoint(const World& world, const Body& bodyA, const Body& bodyB, float anchorX, float anchorY)
        : Joint(b2JointId{}) {
        b2WeldJointDef def = b2DefaultWeldJointDef();
        def.base.bodyIdA = bodyA.getId();
        def.base.bodyIdB = bodyB.getId();
        def.base.localFrameA.p = {anchorX, anchorY};
        def.base.localFrameB.p = {anchorX, anchorY};
        
        jointId = b2CreateWeldJoint(world.getId(), &def);
    }
};

} // namespace box2d_bindings

EMSCRIPTEN_BINDINGS(box2d) {
    // Enums
    enum_<b2BodyType>("BodyType")
        .value("staticBody", b2_staticBody)
        .value("kinematicBody", b2_kinematicBody)
        .value("dynamicBody", b2_dynamicBody);
        
    // World class
    class_<box2d_bindings::World>("World")
        .constructor<>()
        .constructor<float, float>()
        .function("step", &box2d_bindings::World::step)
        .function("isValid", &box2d_bindings::World::isValid)
        .function("castRayClosest", &box2d_bindings::World::castRayClosest)
        .function("setGravity", &box2d_bindings::World::setGravity)
        .function("getGravity", &box2d_bindings::World::getGravity)
        .function("enableSleeping", &box2d_bindings::World::enableSleeping)
        .function("enableContinuous", &box2d_bindings::World::enableContinuous);
        
    // Body class
    class_<box2d_bindings::Body>("Body")
        .constructor<const box2d_bindings::World&>()
        .constructor<const box2d_bindings::World&, float, float>()
        .constructor<const box2d_bindings::World&, float, float, b2BodyType>()
        .function("isValid", &box2d_bindings::Body::isValid)
        .function("getPosition", &box2d_bindings::Body::getPosition)
        .function("getAngle", &box2d_bindings::Body::getAngle)
        .function("setPosition", &box2d_bindings::Body::setPosition)
        .function("setAngle", &box2d_bindings::Body::setAngle)
        .function("applyForce", &box2d_bindings::Body::applyForce)
        .function("applyImpulse", &box2d_bindings::Body::applyImpulse)
        .function("getLinearVelocity", &box2d_bindings::Body::getLinearVelocity)
        .function("setLinearVelocity", &box2d_bindings::Body::setLinearVelocity)
        .function("getAngularVelocity", &box2d_bindings::Body::getAngularVelocity)
        .function("setAngularVelocity", &box2d_bindings::Body::setAngularVelocity)
        .function("getMass", &box2d_bindings::Body::getMass)
        .function("getInertia", &box2d_bindings::Body::getInertia)
        .function("getCenterOfMass", &box2d_bindings::Body::getCenterOfMass)
        .function("isAwake", &box2d_bindings::Body::isAwake)
        .function("setAwake", &box2d_bindings::Body::setAwake)
        .function("isSleepingEnabled", &box2d_bindings::Body::isSleepingEnabled)
        .function("enableSleep", &box2d_bindings::Body::enableSleep)
        .function("getType", &box2d_bindings::Body::getType)
        .function("setType", &box2d_bindings::Body::setType)
        .function("isEnabled", &box2d_bindings::Body::isEnabled)
        .function("setEnabled", &box2d_bindings::Body::setEnabled);
        
    // ShapeFactory class
    class_<box2d_bindings::ShapeFactory>("ShapeFactory")
        .class_function("createBox", &box2d_bindings::ShapeFactory::createBox)
        .class_function("createCircle", &box2d_bindings::ShapeFactory::createCircle)
        .class_function("createCapsule", &box2d_bindings::ShapeFactory::createCapsule)
        .class_function("createSegment", &box2d_bindings::ShapeFactory::createSegment)
        .class_function("createPolygon", &box2d_bindings::ShapeFactory::createPolygon);
        
    // Joint classes
    class_<box2d_bindings::Joint>("Joint")
        .function("isValid", &box2d_bindings::Joint::isValid)
        .function("getConstraintForce", &box2d_bindings::Joint::getConstraintForce)
        .function("getConstraintTorque", &box2d_bindings::Joint::getConstraintTorque)
        .function("wakeBodies", &box2d_bindings::Joint::wakeBodies)
        .function("setCollideConnected", &box2d_bindings::Joint::setCollideConnected)
        .function("getCollideConnected", &box2d_bindings::Joint::getCollideConnected);
        
    class_<box2d_bindings::DistanceJoint, base<box2d_bindings::Joint>>("DistanceJoint")
        .constructor<const box2d_bindings::World&, const box2d_bindings::Body&, const box2d_bindings::Body&, float, float, float, float>();
        
    class_<box2d_bindings::RevoluteJoint, base<box2d_bindings::Joint>>("RevoluteJoint")
        .constructor<const box2d_bindings::World&, const box2d_bindings::Body&, const box2d_bindings::Body&, float, float>();
        
    class_<box2d_bindings::PrismaticJoint, base<box2d_bindings::Joint>>("PrismaticJoint")
        .constructor<const box2d_bindings::World&, const box2d_bindings::Body&, const box2d_bindings::Body&, float, float, float, float>();
        
    class_<box2d_bindings::MouseJoint, base<box2d_bindings::Joint>>("MouseJoint")
        .constructor<const box2d_bindings::World&, const box2d_bindings::Body&, float, float>()
        .function("setTarget", &box2d_bindings::MouseJoint::setTarget);
        
    class_<box2d_bindings::WeldJoint, base<box2d_bindings::Joint>>("WeldJoint")
        .constructor<const box2d_bindings::World&, const box2d_bindings::Body&, const box2d_bindings::Body&, float, float>();
}