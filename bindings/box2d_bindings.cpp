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
};

} // namespace box2d_bindings

// Emscripten bindings
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
        .function("isValid", &box2d_bindings::World::isValid);
        
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
        .function("setAngularVelocity", &box2d_bindings::Body::setAngularVelocity);
        
    // ShapeFactory class
    class_<box2d_bindings::ShapeFactory>("ShapeFactory")
        .class_function("createBox", &box2d_bindings::ShapeFactory::createBox)
        .class_function("createCircle", &box2d_bindings::ShapeFactory::createCircle);
}