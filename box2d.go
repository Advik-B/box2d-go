// Package box2d provides Go bindings for Box2D physics engine using CGO
package box2d

/*
#cgo CFLAGS: -I${SRCDIR}/box2d_vendor/include -I${SRCDIR}/box2d_vendor/src -std=c17 -D_POSIX_C_SOURCE=199309L
#cgo LDFLAGS: -lm

#include <box2d/box2d.h>
#include <stdlib.h>
#include "box2d_vendor/src/aabb.c"
#include "box2d_vendor/src/arena_allocator.c"
#include "box2d_vendor/src/array.c"
#include "box2d_vendor/src/bitset.c"
#include "box2d_vendor/src/body.c"
#include "box2d_vendor/src/broad_phase.c"
#include "box2d_vendor/src/constraint_graph.c"
#include "box2d_vendor/src/contact.c"
#include "box2d_vendor/src/contact_solver.c"
#include "box2d_vendor/src/core.c"
#include "box2d_vendor/src/distance.c"
#include "box2d_vendor/src/distance_joint.c"
#include "box2d_vendor/src/dynamic_tree.c"
#include "box2d_vendor/src/geometry.c"
#include "box2d_vendor/src/hull.c"
#include "box2d_vendor/src/id_pool.c"
#include "box2d_vendor/src/island.c"
#include "box2d_vendor/src/joint.c"
#include "box2d_vendor/src/manifold.c"
#include "box2d_vendor/src/math_functions.c"
#include "box2d_vendor/src/motor_joint.c"
#include "box2d_vendor/src/mover.c"
#include "box2d_vendor/src/physics_world.c"
#include "box2d_vendor/src/prismatic_joint.c"
#include "box2d_vendor/src/revolute_joint.c"
#include "box2d_vendor/src/sensor.c"
#include "box2d_vendor/src/shape.c"
#include "box2d_vendor/src/solver.c"
#include "box2d_vendor/src/solver_set.c"
#include "box2d_vendor/src/table.c"
#include "box2d_vendor/src/timer.c"
#include "box2d_vendor/src/types.c"
#include "box2d_vendor/src/weld_joint.c"
#include "box2d_vendor/src/wheel_joint.c"
*/
import "C"

// Vec2 represents a 2D vector
type Vec2 struct {
	X, Y float32
}

// WorldDef is the definition for creating a world
type WorldDef struct {
	Gravity Vec2
}

// WorldId is a handle to a Box2D world
type WorldId struct {
	id C.b2WorldId
}

// BodyDef is the definition for creating a body
type BodyDef struct {
	Type     BodyType
	Position Vec2
	Angle    float32
}

// BodyType represents the type of a physics body
type BodyType int32

const (
	StaticBody    BodyType = C.b2_staticBody
	KinematicBody BodyType = C.b2_kinematicBody
	DynamicBody   BodyType = C.b2_dynamicBody
)

// BodyId is a handle to a Box2D body
type BodyId struct {
	id C.b2BodyId
}

// ShapeDef is the definition for creating a shape
type ShapeDef struct {
	Density  float32
	Friction float32
}

// ShapeId is a handle to a Box2D shape
type ShapeId struct {
	id C.b2ShapeId
}

// Circle represents a circle shape
type Circle struct {
	Center Vec2
	Radius float32
}

// Polygon represents a polygon shape
type Polygon struct {
	cPoly C.b2Polygon
}

// DefaultWorldDef returns a world definition with default values
func DefaultWorldDef() WorldDef {
	cDef := C.b2DefaultWorldDef()
	return WorldDef{
		Gravity: Vec2{X: float32(cDef.gravity.x), Y: float32(cDef.gravity.y)},
	}
}

// CreateWorld creates a new physics world
func CreateWorld(def *WorldDef) WorldId {
	cDef := C.b2DefaultWorldDef()
	cDef.gravity.x = C.float(def.Gravity.X)
	cDef.gravity.y = C.float(def.Gravity.Y)
	
	worldId := C.b2CreateWorld(&cDef)
	return WorldId{id: worldId}
}

// DestroyWorld destroys a physics world
func DestroyWorld(worldId WorldId) {
	C.b2DestroyWorld(worldId.id)
}

// Step simulates the world for one time step
func (w WorldId) Step(timeStep float32, subStepCount int32) {
	C.b2World_Step(w.id, C.float(timeStep), C.int(subStepCount))
}

// DefaultBodyDef returns a body definition with default values
func DefaultBodyDef() BodyDef {
	cDef := C.b2DefaultBodyDef()
	return BodyDef{
		Type:     BodyType(cDef._type),
		Position: Vec2{X: float32(cDef.position.x), Y: float32(cDef.position.y)},
		Angle:    float32(C.b2Rot_GetAngle(cDef.rotation)),
	}
}

// CreateBody creates a new body in the world
func (w WorldId) CreateBody(def *BodyDef) BodyId {
	cDef := C.b2DefaultBodyDef()
	cDef._type = C.b2BodyType(def.Type)
	cDef.position.x = C.float(def.Position.X)
	cDef.position.y = C.float(def.Position.Y)
	cDef.rotation = C.b2MakeRot(C.float(def.Angle))
	
	bodyId := C.b2CreateBody(w.id, &cDef)
	return BodyId{id: bodyId}
}

// DestroyBody destroys a body
func DestroyBody(bodyId BodyId) {
	C.b2DestroyBody(bodyId.id)
}

// GetPosition returns the position of a body
func (b BodyId) GetPosition() Vec2 {
	pos := C.b2Body_GetPosition(b.id)
	return Vec2{X: float32(pos.x), Y: float32(pos.y)}
}

// GetAngle returns the angle of a body
func (b BodyId) GetAngle() float32 {
	rot := C.b2Body_GetRotation(b.id)
	return float32(C.b2Rot_GetAngle(rot))
}

// SetTransform sets the position and angle of a body
func (b BodyId) SetTransform(position Vec2, angle float32) {
	cPos := C.b2Vec2{x: C.float(position.X), y: C.float(position.Y)}
	cRot := C.b2MakeRot(C.float(angle))
	C.b2Body_SetTransform(b.id, cPos, cRot)
}

// DefaultShapeDef returns a shape definition with default values
func DefaultShapeDef() ShapeDef {
	cDef := C.b2DefaultShapeDef()
	return ShapeDef{
		Density:  float32(cDef.density),
		Friction: float32(cDef.material.friction),
	}
}

// MakeCircle creates a circle shape definition
func MakeCircle(center Vec2, radius float32) Circle {
	return Circle{
		Center: center,
		Radius: radius,
	}
}

// CreateCircleShape creates a circle shape and attaches it to a body
func (b BodyId) CreateCircleShape(def *ShapeDef, circle *Circle) ShapeId {
	cDef := C.b2DefaultShapeDef()
	cDef.density = C.float(def.Density)
	cDef.material.friction = C.float(def.Friction)
	
	cCircle := C.b2Circle{
		center: C.b2Vec2{x: C.float(circle.Center.X), y: C.float(circle.Center.Y)},
		radius: C.float(circle.Radius),
	}
	
	shapeId := C.b2CreateCircleShape(b.id, &cDef, &cCircle)
	return ShapeId{id: shapeId}
}

// MakeBox creates a box polygon shape
func MakeBox(hx, hy float32) Polygon {
	cPoly := C.b2MakeBox(C.float(hx), C.float(hy))
	return Polygon{cPoly: cPoly}
}

// CreatePolygonShape creates a polygon shape and attaches it to a body
func (b BodyId) CreatePolygonShape(def *ShapeDef, polygon *Polygon) ShapeId {
	cDef := C.b2DefaultShapeDef()
	cDef.density = C.float(def.Density)
	cDef.material.friction = C.float(def.Friction)
	
	shapeId := C.b2CreatePolygonShape(b.id, &cDef, &polygon.cPoly)
	return ShapeId{id: shapeId}
}

// DestroyShape destroys a shape
func DestroyShape(shapeId ShapeId) {
	C.b2DestroyShape(shapeId.id, C.bool(true))
}
