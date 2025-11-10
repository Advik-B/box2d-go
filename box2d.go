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
	Density     float32
	Friction    float32
	Restitution float32
}

// ShapeId is a handle to a Box2D shape
type ShapeId struct {
	id C.b2ShapeId
}

// JointId is a handle to a Box2D joint
type JointId struct {
	id C.b2JointId
}

// ChainId is a handle to a Box2D chain
type ChainId struct {
	id C.b2ChainId
}

// ContactId is a handle to a Box2D contact
type ContactId struct {
	id C.b2ContactId
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

// Segment represents a line segment
type Segment struct {
	Point1 Vec2
	Point2 Vec2
}

// Capsule represents a capsule shape
type Capsule struct {
	Center1 Vec2
	Center2 Vec2
	Radius  float32
}

// Rot represents a 2D rotation
type Rot struct {
	C, S float32 // cosine and sine
}

// Transform represents a 2D transform (position and rotation)
type Transform struct {
	Position Vec2
	Rotation Rot
}

// AABB represents an axis-aligned bounding box
type AABB struct {
	LowerBound Vec2
	UpperBound Vec2
}

// RayResult represents the result of a ray cast
type RayResult struct {
	ShapeId    ShapeId
	Point      Vec2
	Normal     Vec2
	Fraction   float32
	NodeVisits int32
	LeafVisits int32
	Hit        bool
}

// MassData represents mass properties
type MassData struct {
	Mass              float32
	Center            Vec2
	RotationalInertia float32
}

// MotionLocks restricts body movement
type MotionLocks struct {
	LinearX  bool
	LinearY  bool
	AngularZ bool
}

// Filter is used for collision filtering
type Filter struct {
	CategoryBits uint64
	MaskBits     uint64
	GroupIndex   int32
}

// QueryFilter is used for queries
type QueryFilter struct {
	CategoryBits uint64
	MaskBits     uint64
}

// Profile contains timing information
type Profile struct {
	Step                 float32
	Pairs                float32
	Collide              float32
	Solve                float32
	PrepareStages        float32
	SolveConstraints     float32
	PrepareConstraints   float32
	IntegrateVelocities  float32
	WarmStart            float32
	SolveImpulses        float32
	IntegratePositions   float32
	RelaxImpulses        float32
	ApplyRestitution     float32
	StoreImpulses        float32
	SplitIslands         float32
	Transforms           float32
	SensorHits           float32
	JointEvents          float32
	HitEvents            float32
}

// Counters contains simulation counters
type Counters struct {
	BodyCount        int32
	ShapeCount       int32
	ContactCount     int32
	JointCount       int32
	IslandCount      int32
	StackUsed        int32
	StaticTreeHeight int32
	TreeHeight       int32
	ByteCount        int32
	TaskCount        int32
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
		Density:     float32(cDef.density),
		Friction:    float32(cDef.material.friction),
		Restitution: float32(cDef.material.restitution),
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
	cDef.material.restitution = C.float(def.Restitution)
	
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
	cDef.material.restitution = C.float(def.Restitution)
	
	shapeId := C.b2CreatePolygonShape(b.id, &cDef, &polygon.cPoly)
	return ShapeId{id: shapeId}
}

// DestroyShape destroys a shape
func DestroyShape(shapeId ShapeId) {
	C.b2DestroyShape(shapeId.id, C.bool(true))
}

// GetType returns the type of a body
func (b BodyId) GetType() BodyType {
	return BodyType(C.b2Body_GetType(b.id))
}

// SetLinearVelocity sets the linear velocity of a body
func (b BodyId) SetLinearVelocity(velocity Vec2) {
	cVel := C.b2Vec2{x: C.float(velocity.X), y: C.float(velocity.Y)}
	C.b2Body_SetLinearVelocity(b.id, cVel)
}

// ApplyForceToCenter applies a force to the center of mass of a body
func (b BodyId) ApplyForceToCenter(force Vec2, wake bool) {
	cForce := C.b2Vec2{x: C.float(force.X), y: C.float(force.Y)}
	C.b2Body_ApplyForceToCenter(b.id, cForce, C.bool(wake))
}

// ====================================================================
// Additional World Functions
// ====================================================================

// IsValid checks if a world ID is valid
func (w WorldId) IsValid() bool {
	return bool(C.b2World_IsValid(w.id))
}

// SetGravity sets the gravity vector for the world
func (w WorldId) SetGravity(gravity Vec2) {
	cGravity := C.b2Vec2{x: C.float(gravity.X), y: C.float(gravity.Y)}
	C.b2World_SetGravity(w.id, cGravity)
}

// GetGravity returns the gravity vector for the world
func (w WorldId) GetGravity() Vec2 {
	cGravity := C.b2World_GetGravity(w.id)
	return Vec2{X: float32(cGravity.x), Y: float32(cGravity.y)}
}

// EnableSleeping enables/disables sleeping for the world
func (w WorldId) EnableSleeping(flag bool) {
	C.b2World_EnableSleeping(w.id, C.bool(flag))
}

// IsSleepingEnabled returns whether sleeping is enabled
func (w WorldId) IsSleepingEnabled() bool {
	return bool(C.b2World_IsSleepingEnabled(w.id))
}

// EnableContinuous enables/disables continuous collision
func (w WorldId) EnableContinuous(flag bool) {
	C.b2World_EnableContinuous(w.id, C.bool(flag))
}

// IsContinuousEnabled returns whether continuous collision is enabled
func (w WorldId) IsContinuousEnabled() bool {
	return bool(C.b2World_IsContinuousEnabled(w.id))
}

// SetRestitutionThreshold sets the restitution threshold
func (w WorldId) SetRestitutionThreshold(value float32) {
	C.b2World_SetRestitutionThreshold(w.id, C.float(value))
}

// GetRestitutionThreshold returns the restitution threshold
func (w WorldId) GetRestitutionThreshold() float32 {
	return float32(C.b2World_GetRestitutionThreshold(w.id))
}

// SetHitEventThreshold sets the hit event threshold
func (w WorldId) SetHitEventThreshold(value float32) {
	C.b2World_SetHitEventThreshold(w.id, C.float(value))
}

// GetHitEventThreshold returns the hit event threshold
func (w WorldId) GetHitEventThreshold() float32 {
	return float32(C.b2World_GetHitEventThreshold(w.id))
}

// SetContactTuning sets contact tuning parameters
func (w WorldId) SetContactTuning(hertz, dampingRatio, pushSpeed float32) {
	C.b2World_SetContactTuning(w.id, C.float(hertz), C.float(dampingRatio), C.float(pushSpeed))
}

// SetMaximumLinearSpeed sets the maximum linear speed
func (w WorldId) SetMaximumLinearSpeed(maximumLinearSpeed float32) {
	C.b2World_SetMaximumLinearSpeed(w.id, C.float(maximumLinearSpeed))
}

// GetMaximumLinearSpeed returns the maximum linear speed
func (w WorldId) GetMaximumLinearSpeed() float32 {
	return float32(C.b2World_GetMaximumLinearSpeed(w.id))
}

// EnableWarmStarting enables/disables warm starting
func (w WorldId) EnableWarmStarting(flag bool) {
	C.b2World_EnableWarmStarting(w.id, C.bool(flag))
}

// IsWarmStartingEnabled returns whether warm starting is enabled
func (w WorldId) IsWarmStartingEnabled() bool {
	return bool(C.b2World_IsWarmStartingEnabled(w.id))
}

// GetAwakeBodyCount returns the number of awake bodies
func (w WorldId) GetAwakeBodyCount() int32 {
	return int32(C.b2World_GetAwakeBodyCount(w.id))
}

// GetProfile returns timing profile information
func (w WorldId) GetProfile() Profile {
	cProfile := C.b2World_GetProfile(w.id)
	return Profile{
		Step:                 float32(cProfile.step),
		Pairs:                float32(cProfile.pairs),
		Collide:              float32(cProfile.collide),
		Solve:                float32(cProfile.solve),
		PrepareStages:        float32(cProfile.prepareStages),
		SolveConstraints:     float32(cProfile.solveConstraints),
		PrepareConstraints:   float32(cProfile.prepareConstraints),
		IntegrateVelocities:  float32(cProfile.integrateVelocities),
		WarmStart:            float32(cProfile.warmStart),
		SolveImpulses:        float32(cProfile.solveImpulses),
		IntegratePositions:   float32(cProfile.integratePositions),
		RelaxImpulses:        float32(cProfile.relaxImpulses),
		ApplyRestitution:     float32(cProfile.applyRestitution),
		StoreImpulses:        float32(cProfile.storeImpulses),
		SplitIslands:         float32(cProfile.splitIslands),
		Transforms:           float32(cProfile.transforms),
		SensorHits:           float32(cProfile.sensorHits),
		JointEvents:          float32(cProfile.jointEvents),
		HitEvents:            float32(cProfile.hitEvents),
	}
}

// GetCounters returns simulation counters
func (w WorldId) GetCounters() Counters {
	cCounters := C.b2World_GetCounters(w.id)
	return Counters{
		BodyCount:        int32(cCounters.bodyCount),
		ShapeCount:       int32(cCounters.shapeCount),
		ContactCount:     int32(cCounters.contactCount),
		JointCount:       int32(cCounters.jointCount),
		IslandCount:      int32(cCounters.islandCount),
		StackUsed:        int32(cCounters.stackUsed),
		StaticTreeHeight: int32(cCounters.staticTreeHeight),
		TreeHeight:       int32(cCounters.treeHeight),
		ByteCount:        int32(cCounters.byteCount),
		TaskCount:        int32(cCounters.taskCount),
	}
}

// DumpMemoryStats dumps memory statistics
func (w WorldId) DumpMemoryStats() {
	C.b2World_DumpMemoryStats(w.id)
}

// RebuildStaticTree rebuilds the static tree
func (w WorldId) RebuildStaticTree() {
	C.b2World_RebuildStaticTree(w.id)
}

// EnableSpeculative enables/disables speculative collision
func (w WorldId) EnableSpeculative(flag bool) {
	C.b2World_EnableSpeculative(w.id, C.bool(flag))
}

// CastRayClosest casts a ray and returns the closest hit
func (w WorldId) CastRayClosest(origin, translation Vec2, filter QueryFilter) RayResult {
	cOrigin := C.b2Vec2{x: C.float(origin.X), y: C.float(origin.Y)}
	cTranslation := C.b2Vec2{x: C.float(translation.X), y: C.float(translation.Y)}
	cFilter := C.b2QueryFilter{
		categoryBits: C.uint64_t(filter.CategoryBits),
		maskBits:     C.uint64_t(filter.MaskBits),
	}
	cResult := C.b2World_CastRayClosest(w.id, cOrigin, cTranslation, cFilter)
	return RayResult{
		ShapeId:    ShapeId{id: cResult.shapeId},
		Point:      Vec2{X: float32(cResult.point.x), Y: float32(cResult.point.y)},
		Normal:     Vec2{X: float32(cResult.normal.x), Y: float32(cResult.normal.y)},
		Fraction:   float32(cResult.fraction),
		NodeVisits: int32(cResult.nodeVisits),
		LeafVisits: int32(cResult.leafVisits),
		Hit:        bool(cResult.hit),
	}
}

// ====================================================================
// Additional Body Functions
// ====================================================================

// IsValid checks if a body ID is valid
func (b BodyId) IsValid() bool {
	return bool(C.b2Body_IsValid(b.id))
}

// SetType sets the type of a body
func (b BodyId) SetType(bodyType BodyType) {
	C.b2Body_SetType(b.id, C.b2BodyType(bodyType))
}

// GetLinearVelocity returns the linear velocity of a body
func (b BodyId) GetLinearVelocity() Vec2 {
	cVel := C.b2Body_GetLinearVelocity(b.id)
	return Vec2{X: float32(cVel.x), Y: float32(cVel.y)}
}

// GetAngularVelocity returns the angular velocity of a body
func (b BodyId) GetAngularVelocity() float32 {
	return float32(C.b2Body_GetAngularVelocity(b.id))
}

// SetAngularVelocity sets the angular velocity of a body
func (b BodyId) SetAngularVelocity(angularVelocity float32) {
	C.b2Body_SetAngularVelocity(b.id, C.float(angularVelocity))
}

// ApplyForce applies a force at a point
func (b BodyId) ApplyForce(force, point Vec2, wake bool) {
	cForce := C.b2Vec2{x: C.float(force.X), y: C.float(force.Y)}
	cPoint := C.b2Vec2{x: C.float(point.X), y: C.float(point.Y)}
	C.b2Body_ApplyForce(b.id, cForce, cPoint, C.bool(wake))
}

// ApplyTorque applies a torque
func (b BodyId) ApplyTorque(torque float32, wake bool) {
	C.b2Body_ApplyTorque(b.id, C.float(torque), C.bool(wake))
}

// ApplyLinearImpulse applies a linear impulse at a point
func (b BodyId) ApplyLinearImpulse(impulse, point Vec2, wake bool) {
	cImpulse := C.b2Vec2{x: C.float(impulse.X), y: C.float(impulse.Y)}
	cPoint := C.b2Vec2{x: C.float(point.X), y: C.float(point.Y)}
	C.b2Body_ApplyLinearImpulse(b.id, cImpulse, cPoint, C.bool(wake))
}

// ApplyLinearImpulseToCenter applies a linear impulse to the center
func (b BodyId) ApplyLinearImpulseToCenter(impulse Vec2, wake bool) {
	cImpulse := C.b2Vec2{x: C.float(impulse.X), y: C.float(impulse.Y)}
	C.b2Body_ApplyLinearImpulseToCenter(b.id, cImpulse, C.bool(wake))
}

// ApplyAngularImpulse applies an angular impulse
func (b BodyId) ApplyAngularImpulse(impulse float32, wake bool) {
	C.b2Body_ApplyAngularImpulse(b.id, C.float(impulse), C.bool(wake))
}

// GetMass returns the mass of a body
func (b BodyId) GetMass() float32 {
	return float32(C.b2Body_GetMass(b.id))
}

// GetRotationalInertia returns the rotational inertia of a body
func (b BodyId) GetRotationalInertia() float32 {
	return float32(C.b2Body_GetRotationalInertia(b.id))
}

// GetLocalCenterOfMass returns the local center of mass
func (b BodyId) GetLocalCenterOfMass() Vec2 {
	cCenter := C.b2Body_GetLocalCenterOfMass(b.id)
	return Vec2{X: float32(cCenter.x), Y: float32(cCenter.y)}
}

// GetWorldCenterOfMass returns the world center of mass
func (b BodyId) GetWorldCenterOfMass() Vec2 {
	cCenter := C.b2Body_GetWorldCenterOfMass(b.id)
	return Vec2{X: float32(cCenter.x), Y: float32(cCenter.y)}
}

// SetMassData sets the mass data
func (b BodyId) SetMassData(massData MassData) {
	cMassData := C.b2MassData{
		mass:              C.float(massData.Mass),
		center:            C.b2Vec2{x: C.float(massData.Center.X), y: C.float(massData.Center.Y)},
		rotationalInertia: C.float(massData.RotationalInertia),
	}
	C.b2Body_SetMassData(b.id, cMassData)
}

// GetMassData returns the mass data
func (b BodyId) GetMassData() MassData {
	cMassData := C.b2Body_GetMassData(b.id)
	return MassData{
		Mass:              float32(cMassData.mass),
		Center:            Vec2{X: float32(cMassData.center.x), Y: float32(cMassData.center.y)},
		RotationalInertia: float32(cMassData.rotationalInertia),
	}
}

// ApplyMassFromShapes recomputes mass from shapes
func (b BodyId) ApplyMassFromShapes() {
	C.b2Body_ApplyMassFromShapes(b.id)
}

// SetLinearDamping sets the linear damping
func (b BodyId) SetLinearDamping(linearDamping float32) {
	C.b2Body_SetLinearDamping(b.id, C.float(linearDamping))
}

// GetLinearDamping returns the linear damping
func (b BodyId) GetLinearDamping() float32 {
	return float32(C.b2Body_GetLinearDamping(b.id))
}

// SetAngularDamping sets the angular damping
func (b BodyId) SetAngularDamping(angularDamping float32) {
	C.b2Body_SetAngularDamping(b.id, C.float(angularDamping))
}

// GetAngularDamping returns the angular damping
func (b BodyId) GetAngularDamping() float32 {
	return float32(C.b2Body_GetAngularDamping(b.id))
}

// SetGravityScale sets the gravity scale
func (b BodyId) SetGravityScale(gravityScale float32) {
	C.b2Body_SetGravityScale(b.id, C.float(gravityScale))
}

// GetGravityScale returns the gravity scale
func (b BodyId) GetGravityScale() float32 {
	return float32(C.b2Body_GetGravityScale(b.id))
}

// IsAwake checks if a body is awake
func (b BodyId) IsAwake() bool {
	return bool(C.b2Body_IsAwake(b.id))
}

// SetAwake sets the awake status
func (b BodyId) SetAwake(awake bool) {
	C.b2Body_SetAwake(b.id, C.bool(awake))
}

// EnableSleep enables/disables sleep for this body
func (b BodyId) EnableSleep(enableSleep bool) {
	C.b2Body_EnableSleep(b.id, C.bool(enableSleep))
}

// IsSleepEnabled checks if sleep is enabled
func (b BodyId) IsSleepEnabled() bool {
	return bool(C.b2Body_IsSleepEnabled(b.id))
}

// SetSleepThreshold sets the sleep threshold
func (b BodyId) SetSleepThreshold(sleepThreshold float32) {
	C.b2Body_SetSleepThreshold(b.id, C.float(sleepThreshold))
}

// GetSleepThreshold returns the sleep threshold
func (b BodyId) GetSleepThreshold() float32 {
	return float32(C.b2Body_GetSleepThreshold(b.id))
}

// IsEnabled checks if a body is enabled
func (b BodyId) IsEnabled() bool {
	return bool(C.b2Body_IsEnabled(b.id))
}

// Disable disables a body
func (b BodyId) Disable() {
	C.b2Body_Disable(b.id)
}

// Enable enables a body
func (b BodyId) Enable() {
	C.b2Body_Enable(b.id)
}

// SetBullet sets the bullet flag (for CCD)
func (b BodyId) SetBullet(flag bool) {
	C.b2Body_SetBullet(b.id, C.bool(flag))
}

// IsBullet checks if the bullet flag is set
func (b BodyId) IsBullet() bool {
	return bool(C.b2Body_IsBullet(b.id))
}

// SetMotionLocks sets motion locks
func (b BodyId) SetMotionLocks(locks MotionLocks) {
	cLocks := C.b2MotionLocks{
		linearX:  C.bool(locks.LinearX),
		linearY:  C.bool(locks.LinearY),
		angularZ: C.bool(locks.AngularZ),
	}
	C.b2Body_SetMotionLocks(b.id, cLocks)
}

// GetMotionLocks returns motion locks
func (b BodyId) GetMotionLocks() MotionLocks {
	cLocks := C.b2Body_GetMotionLocks(b.id)
	return MotionLocks{
		LinearX:  bool(cLocks.linearX),
		LinearY:  bool(cLocks.linearY),
		AngularZ: bool(cLocks.angularZ),
	}
}

// GetRotation returns the rotation of a body
func (b BodyId) GetRotation() Rot {
	cRot := C.b2Body_GetRotation(b.id)
	return Rot{C: float32(cRot.c), S: float32(cRot.s)}
}

// GetTransform returns the transform of a body
func (b BodyId) GetTransform() Transform {
	cTransform := C.b2Body_GetTransform(b.id)
	return Transform{
		Position: Vec2{X: float32(cTransform.p.x), Y: float32(cTransform.p.y)},
		Rotation: Rot{C: float32(cTransform.q.c), S: float32(cTransform.q.s)},
	}
}

// GetLocalPoint transforms a world point to local coordinates
func (b BodyId) GetLocalPoint(worldPoint Vec2) Vec2 {
	cWorldPoint := C.b2Vec2{x: C.float(worldPoint.X), y: C.float(worldPoint.Y)}
	cLocalPoint := C.b2Body_GetLocalPoint(b.id, cWorldPoint)
	return Vec2{X: float32(cLocalPoint.x), Y: float32(cLocalPoint.y)}
}

// GetWorldPoint transforms a local point to world coordinates
func (b BodyId) GetWorldPoint(localPoint Vec2) Vec2 {
	cLocalPoint := C.b2Vec2{x: C.float(localPoint.X), y: C.float(localPoint.Y)}
	cWorldPoint := C.b2Body_GetWorldPoint(b.id, cLocalPoint)
	return Vec2{X: float32(cWorldPoint.x), Y: float32(cWorldPoint.y)}
}

// GetLocalVector transforms a world vector to local coordinates
func (b BodyId) GetLocalVector(worldVector Vec2) Vec2 {
	cWorldVector := C.b2Vec2{x: C.float(worldVector.X), y: C.float(worldVector.Y)}
	cLocalVector := C.b2Body_GetLocalVector(b.id, cWorldVector)
	return Vec2{X: float32(cLocalVector.x), Y: float32(cLocalVector.y)}
}

// GetWorldVector transforms a local vector to world coordinates
func (b BodyId) GetWorldVector(localVector Vec2) Vec2 {
	cLocalVector := C.b2Vec2{x: C.float(localVector.X), y: C.float(localVector.Y)}
	cWorldVector := C.b2Body_GetWorldVector(b.id, cLocalVector)
	return Vec2{X: float32(cWorldVector.x), Y: float32(cWorldVector.y)}
}

// GetContactCapacity returns the contact capacity for this body
func (b BodyId) GetContactCapacity() int32 {
	return int32(C.b2Body_GetContactCapacity(b.id))
}

// GetJointCount returns the number of joints attached to this body
func (b BodyId) GetJointCount() int32 {
	return int32(C.b2Body_GetJointCount(b.id))
}

// GetShapeCount returns the number of shapes attached to this body
func (b BodyId) GetShapeCount() int32 {
	return int32(C.b2Body_GetShapeCount(b.id))
}

// ====================================================================
// Additional Shape Functions
// ====================================================================

// CreateSegmentShape creates a segment (edge) shape
func (b BodyId) CreateSegmentShape(def *ShapeDef, segment *Segment) ShapeId {
	cDef := C.b2DefaultShapeDef()
	cDef.density = C.float(def.Density)
	cDef.material.friction = C.float(def.Friction)
	cDef.material.restitution = C.float(def.Restitution)

	cSegment := C.b2Segment{
		point1: C.b2Vec2{x: C.float(segment.Point1.X), y: C.float(segment.Point1.Y)},
		point2: C.b2Vec2{x: C.float(segment.Point2.X), y: C.float(segment.Point2.Y)},
	}

	shapeId := C.b2CreateSegmentShape(b.id, &cDef, &cSegment)
	return ShapeId{id: shapeId}
}

// CreateCapsuleShape creates a capsule shape
func (b BodyId) CreateCapsuleShape(def *ShapeDef, capsule *Capsule) ShapeId {
	cDef := C.b2DefaultShapeDef()
	cDef.density = C.float(def.Density)
	cDef.material.friction = C.float(def.Friction)
	cDef.material.restitution = C.float(def.Restitution)

	cCapsule := C.b2Capsule{
		center1: C.b2Vec2{x: C.float(capsule.Center1.X), y: C.float(capsule.Center1.Y)},
		center2: C.b2Vec2{x: C.float(capsule.Center2.X), y: C.float(capsule.Center2.Y)},
		radius:  C.float(capsule.Radius),
	}

	shapeId := C.b2CreateCapsuleShape(b.id, &cDef, &cCapsule)
	return ShapeId{id: shapeId}
}

// IsValid checks if a shape ID is valid
func (s ShapeId) IsValid() bool {
	return bool(C.b2Shape_IsValid(s.id))
}

// GetBody returns the body that owns this shape
func (s ShapeId) GetBody() BodyId {
	bodyId := C.b2Shape_GetBody(s.id)
	return BodyId{id: bodyId}
}

// IsSensor checks if a shape is a sensor
func (s ShapeId) IsSensor() bool {
	return bool(C.b2Shape_IsSensor(s.id))
}

// SetDensity sets the density of a shape
func (s ShapeId) SetDensity(density float32, updateBodyMass bool) {
	C.b2Shape_SetDensity(s.id, C.float(density), C.bool(updateBodyMass))
}

// GetDensity returns the density of a shape
func (s ShapeId) GetDensity() float32 {
	return float32(C.b2Shape_GetDensity(s.id))
}

// SetFriction sets the friction of a shape
func (s ShapeId) SetFriction(friction float32) {
	C.b2Shape_SetFriction(s.id, C.float(friction))
}

// GetFriction returns the friction of a shape
func (s ShapeId) GetFriction() float32 {
	return float32(C.b2Shape_GetFriction(s.id))
}

// SetRestitution sets the restitution of a shape
func (s ShapeId) SetRestitution(restitution float32) {
	C.b2Shape_SetRestitution(s.id, C.float(restitution))
}

// GetRestitution returns the restitution of a shape
func (s ShapeId) GetRestitution() float32 {
	return float32(C.b2Shape_GetRestitution(s.id))
}

// GetAABB returns the axis-aligned bounding box
func (s ShapeId) GetAABB() AABB {
	cAABB := C.b2Shape_GetAABB(s.id)
	return AABB{
		LowerBound: Vec2{X: float32(cAABB.lowerBound.x), Y: float32(cAABB.lowerBound.y)},
		UpperBound: Vec2{X: float32(cAABB.upperBound.x), Y: float32(cAABB.upperBound.y)},
	}
}

// GetClosestPoint returns the closest point on the shape to a target
func (s ShapeId) GetClosestPoint(target Vec2) Vec2 {
	cTarget := C.b2Vec2{x: C.float(target.X), y: C.float(target.Y)}
	cClosest := C.b2Shape_GetClosestPoint(s.id, cTarget)
	return Vec2{X: float32(cClosest.x), Y: float32(cClosest.y)}
}

// SetFilter sets the collision filter
func (s ShapeId) SetFilter(filter Filter) {
	cFilter := C.b2Filter{
		categoryBits: C.uint64_t(filter.CategoryBits),
		maskBits:     C.uint64_t(filter.MaskBits),
		groupIndex:   C.int(filter.GroupIndex),
	}
	C.b2Shape_SetFilter(s.id, cFilter)
}

// GetFilter returns the collision filter
func (s ShapeId) GetFilter() Filter {
	cFilter := C.b2Shape_GetFilter(s.id)
	return Filter{
		CategoryBits: uint64(cFilter.categoryBits),
		MaskBits:     uint64(cFilter.maskBits),
		GroupIndex:   int32(cFilter.groupIndex),
	}
}

// EnableSensorEvents enables/disables sensor events
func (s ShapeId) EnableSensorEvents(flag bool) {
	C.b2Shape_EnableSensorEvents(s.id, C.bool(flag))
}

// AreSensorEventsEnabled checks if sensor events are enabled
func (s ShapeId) AreSensorEventsEnabled() bool {
	return bool(C.b2Shape_AreSensorEventsEnabled(s.id))
}

// EnableContactEvents enables/disables contact events
func (s ShapeId) EnableContactEvents(flag bool) {
	C.b2Shape_EnableContactEvents(s.id, C.bool(flag))
}

// AreContactEventsEnabled checks if contact events are enabled
func (s ShapeId) AreContactEventsEnabled() bool {
	return bool(C.b2Shape_AreContactEventsEnabled(s.id))
}

// EnablePreSolveEvents enables/disables pre-solve events
func (s ShapeId) EnablePreSolveEvents(flag bool) {
	C.b2Shape_EnablePreSolveEvents(s.id, C.bool(flag))
}

// ArePreSolveEventsEnabled checks if pre-solve events are enabled
func (s ShapeId) ArePreSolveEventsEnabled() bool {
	return bool(C.b2Shape_ArePreSolveEventsEnabled(s.id))
}

// EnableHitEvents enables/disables hit events
func (s ShapeId) EnableHitEvents(flag bool) {
	C.b2Shape_EnableHitEvents(s.id, C.bool(flag))
}

// AreHitEventsEnabled checks if hit events are enabled
func (s ShapeId) AreHitEventsEnabled() bool {
	return bool(C.b2Shape_AreHitEventsEnabled(s.id))
}

// TestPoint tests if a point is inside the shape
func (s ShapeId) TestPoint(point Vec2) bool {
	cPoint := C.b2Vec2{x: C.float(point.X), y: C.float(point.Y)}
	return bool(C.b2Shape_TestPoint(s.id, cPoint))
}

// GetContactCapacity returns the contact capacity for this shape
func (s ShapeId) GetContactCapacity() int32 {
	return int32(C.b2Shape_GetContactCapacity(s.id))
}

// ====================================================================
// Math and Geometry Helper Functions
// ====================================================================

// MakeRot creates a rotation from an angle
func MakeRot(angle float32) Rot {
	cRot := C.b2MakeRot(C.float(angle))
	return Rot{C: float32(cRot.c), S: float32(cRot.s)}
}

// RotGetAngle gets the angle from a rotation
func RotGetAngle(rot Rot) float32 {
	cRot := C.b2Rot{c: C.float(rot.C), s: C.float(rot.S)}
	return float32(C.b2Rot_GetAngle(cRot))
}

// MakePolygon creates a polygon from an array of vertices
func MakePolygon(vertices []Vec2, radius float32) Polygon {
	if len(vertices) > 8 {
		vertices = vertices[:8] // Box2D supports up to 8 vertices
	}
	
	hull := C.b2Hull{}
	hull.count = C.int(len(vertices))
	for i, v := range vertices {
		hull.points[i] = C.b2Vec2{x: C.float(v.X), y: C.float(v.Y)}
	}
	
	cPoly := C.b2MakePolygon(&hull, C.float(radius))
	return Polygon{cPoly: cPoly}
}

// MakeOffsetBox creates an offset box polygon
func MakeOffsetBox(hx, hy float32, center Vec2, angle float32) Polygon {
	cCenter := C.b2Vec2{x: C.float(center.X), y: C.float(center.Y)}
	cRot := C.b2MakeRot(C.float(angle))
	cPoly := C.b2MakeOffsetBox(C.float(hx), C.float(hy), cCenter, cRot)
	return Polygon{cPoly: cPoly}
}

// MakeSquare creates a square polygon
func MakeSquare(h float32) Polygon {
	cPoly := C.b2MakeSquare(C.float(h))
	return Polygon{cPoly: cPoly}
}

// TransformPoint transforms a point
func TransformPoint(transform Transform, point Vec2) Vec2 {
	cTransform := C.b2Transform{
		p: C.b2Vec2{x: C.float(transform.Position.X), y: C.float(transform.Position.Y)},
		q: C.b2Rot{c: C.float(transform.Rotation.C), s: C.float(transform.Rotation.S)},
	}
	cPoint := C.b2Vec2{x: C.float(point.X), y: C.float(point.Y)}
	cResult := C.b2TransformPoint(cTransform, cPoint)
	return Vec2{X: float32(cResult.x), Y: float32(cResult.y)}
}

// ====================================================================
// Contact Functions
// ====================================================================

// IsValid checks if a contact ID is valid
func (c ContactId) IsValid() bool {
	return bool(C.b2Contact_IsValid(c.id))
}

// ====================================================================
// Joint Type Definition and Common Joint Functions
// ====================================================================

// JointType represents the type of a joint
type JointType int32

const (
DistanceJoint  JointType = C.b2_distanceJoint
FilterJoint    JointType = C.b2_filterJoint
MotorJoint     JointType = C.b2_motorJoint
PrismaticJoint JointType = C.b2_prismaticJoint
RevoluteJoint  JointType = C.b2_revoluteJoint
WeldJoint      JointType = C.b2_weldJoint
WheelJoint     JointType = C.b2_wheelJoint
)

// DistanceJointDef defines a distance joint
type DistanceJointDef struct {
BodyIdA            BodyId
BodyIdB            BodyId
LocalAnchorA       Vec2
LocalAnchorB       Vec2
Length             float32
EnableSpring       bool
LowerSpringForce   float32
UpperSpringForce   float32
Hertz              float32
DampingRatio       float32
EnableLimit        bool
MinLength          float32
MaxLength          float32
EnableMotor        bool
MaxMotorForce      float32
MotorSpeed         float32
CollideConnected   bool
}

// RevoluteJointDef defines a revolute (hinge) joint
type RevoluteJointDef struct {
BodyIdA          BodyId
BodyIdB          BodyId
LocalAnchorA     Vec2
LocalAnchorB     Vec2
ReferenceAngle   float32
EnableLimit      bool
LowerAngle       float32
UpperAngle       float32
EnableMotor      bool
MotorSpeed       float32
MaxMotorTorque   float32
DrawSize         float32
CollideConnected bool
}

// PrismaticJointDef defines a prismatic (slider) joint
type PrismaticJointDef struct {
BodyIdA          BodyId
BodyIdB          BodyId
LocalAnchorA     Vec2
LocalAnchorB     Vec2
LocalAxisA       Vec2
ReferenceAngle   float32
EnableLimit      bool
LowerTranslation float32
UpperTranslation float32
EnableMotor      bool
MotorSpeed       float32
MaxMotorForce    float32
CollideConnected bool
}

// WeldJointDef defines a weld joint
type WeldJointDef struct {
BodyIdA          BodyId
BodyIdB          BodyId
LocalAnchorA     Vec2
LocalAnchorB     Vec2
ReferenceAngle   float32
LinearHertz      float32
AngularHertz     float32
LinearDampingRatio float32
AngularDampingRatio float32
CollideConnected bool
}

// WheelJointDef defines a wheel joint
type WheelJointDef struct {
BodyIdA          BodyId
BodyIdB          BodyId
LocalAnchorA     Vec2
LocalAnchorB     Vec2
LocalAxisA       Vec2
EnableSpring     bool
Hertz            float32
DampingRatio     float32
EnableLimit      bool
LowerTranslation float32
UpperTranslation float32
EnableMotor      bool
MotorSpeed       float32
MaxMotorTorque   float32
CollideConnected bool
}

// MotorJointDef defines a motor joint
type MotorJointDef struct {
BodyIdA            BodyId
BodyIdB            BodyId
LinearOffset       Vec2
AngularOffset      float32
MaxForce           float32
MaxTorque          float32
CorrectionFactor   float32
CollideConnected   bool
}

// DefaultDistanceJointDef returns a distance joint definition with default values
func DefaultDistanceJointDef() DistanceJointDef {
return DistanceJointDef{
Length:             1.0,
EnableSpring:       false,
Hertz:              0.0,
DampingRatio:       0.0,
EnableLimit:        false,
MinLength:          0.0,
MaxLength:          1000000.0,
EnableMotor:        false,
MaxMotorForce:      0.0,
MotorSpeed:         0.0,
CollideConnected:   false,
}
}

// CreateDistanceJoint creates a distance joint
func DestroyJoint(jointId JointId) {
C.b2DestroyJoint(jointId.id, C.bool(true))
}

// IsValid checks if a joint ID is valid
func (j JointId) IsValid() bool {
return bool(C.b2Joint_IsValid(j.id))
}

// GetType returns the type of a joint
func (j JointId) GetType() JointType {
return JointType(C.b2Joint_GetType(j.id))
}

// GetBodyA returns body A of a joint
func (j JointId) GetBodyA() BodyId {
bodyId := C.b2Joint_GetBodyA(j.id)
return BodyId{id: bodyId}
}

// GetBodyB returns body B of a joint
func (j JointId) GetBodyB() BodyId {
bodyId := C.b2Joint_GetBodyB(j.id)
return BodyId{id: bodyId}
}



// SetCollideConnected sets whether connected bodies should collide
func (j JointId) SetCollideConnected(shouldCollide bool) {
C.b2Joint_SetCollideConnected(j.id, C.bool(shouldCollide))
}

// GetCollideConnected returns whether connected bodies collide
func (j JointId) GetCollideConnected() bool {
return bool(C.b2Joint_GetCollideConnected(j.id))
}

// WakeBodies wakes up the bodies connected to this joint
func (j JointId) WakeBodies() {
C.b2Joint_WakeBodies(j.id)
}

// ====================================================================
// Distance Joint Specific Functions
// ====================================================================

// SetLength sets the length of a distance joint
func (j JointId) SetLength(length float32) {
C.b2DistanceJoint_SetLength(j.id, C.float(length))
}

// GetLength returns the length of a distance joint
func (j JointId) GetLength() float32 {
return float32(C.b2DistanceJoint_GetLength(j.id))
}

// EnableSpring enables/disables the spring on a distance joint
func (j JointId) EnableSpring(enableSpring bool) {
C.b2DistanceJoint_EnableSpring(j.id, C.bool(enableSpring))
}

// IsSpringEnabled checks if the spring is enabled on a distance joint
func (j JointId) IsSpringEnabled() bool {
return bool(C.b2DistanceJoint_IsSpringEnabled(j.id))
}

// SetSpringHertz sets the spring frequency for a distance joint
func (j JointId) SetSpringHertz(hertz float32) {
C.b2DistanceJoint_SetSpringHertz(j.id, C.float(hertz))
}

// GetSpringHertz returns the spring frequency of a distance joint
func (j JointId) GetSpringHertz() float32 {
return float32(C.b2DistanceJoint_GetSpringHertz(j.id))
}

// SetSpringDampingRatio sets the spring damping ratio for a distance joint
func (j JointId) SetSpringDampingRatio(dampingRatio float32) {
C.b2DistanceJoint_SetSpringDampingRatio(j.id, C.float(dampingRatio))
}

// GetSpringDampingRatio returns the spring damping ratio of a distance joint
func (j JointId) GetSpringDampingRatio() float32 {
return float32(C.b2DistanceJoint_GetSpringDampingRatio(j.id))
}

// EnableLimit enables/disables the limit on a distance joint
func (j JointId) EnableLimit(enableLimit bool) {
C.b2DistanceJoint_EnableLimit(j.id, C.bool(enableLimit))
}

// IsLimitEnabled checks if the limit is enabled on a distance joint
func (j JointId) IsLimitEnabled() bool {
return bool(C.b2DistanceJoint_IsLimitEnabled(j.id))
}

// SetLengthRange sets the length range for a distance joint
func (j JointId) SetLengthRange(minLength, maxLength float32) {
C.b2DistanceJoint_SetLengthRange(j.id, C.float(minLength), C.float(maxLength))
}

// GetMinLength returns the minimum length of a distance joint
func (j JointId) GetMinLength() float32 {
return float32(C.b2DistanceJoint_GetMinLength(j.id))
}

// GetMaxLength returns the maximum length of a distance joint
func (j JointId) GetMaxLength() float32 {
return float32(C.b2DistanceJoint_GetMaxLength(j.id))
}

// GetCurrentLength returns the current length of a distance joint
func (j JointId) GetCurrentLength() float32 {
return float32(C.b2DistanceJoint_GetCurrentLength(j.id))
}

// EnableMotor enables/disables the motor on a distance joint
func (j JointId) EnableMotor(enableMotor bool) {
C.b2DistanceJoint_EnableMotor(j.id, C.bool(enableMotor))
}

// IsMotorEnabled checks if the motor is enabled on a distance joint
func (j JointId) IsMotorEnabled() bool {
return bool(C.b2DistanceJoint_IsMotorEnabled(j.id))
}

// SetMotorSpeed sets the motor speed for a distance joint
func (j JointId) SetMotorSpeed(motorSpeed float32) {
C.b2DistanceJoint_SetMotorSpeed(j.id, C.float(motorSpeed))
}

// GetMotorSpeed returns the motor speed of a distance joint
func (j JointId) GetMotorSpeed() float32 {
return float32(C.b2DistanceJoint_GetMotorSpeed(j.id))
}

// SetMaxMotorForce sets the maximum motor force for a distance joint
func (j JointId) SetMaxMotorForce(force float32) {
C.b2DistanceJoint_SetMaxMotorForce(j.id, C.float(force))
}

// GetMaxMotorForce returns the maximum motor force of a distance joint
func (j JointId) GetMaxMotorForce() float32 {
return float32(C.b2DistanceJoint_GetMaxMotorForce(j.id))
}

// GetMotorForce returns the current motor force of a distance joint
func (j JointId) GetMotorForce() float32 {
return float32(C.b2DistanceJoint_GetMotorForce(j.id))
}

// ====================================================================
// Revolute Joint Specific Functions
// ====================================================================

// EnableRevoluteLimit enables/disables the limit on a revolute joint
func (j JointId) EnableRevoluteLimit(enableLimit bool) {
C.b2RevoluteJoint_EnableLimit(j.id, C.bool(enableLimit))
}

// IsRevoluteLimitEnabled checks if the limit is enabled on a revolute joint
func (j JointId) IsRevoluteLimitEnabled() bool {
return bool(C.b2RevoluteJoint_IsLimitEnabled(j.id))
}

// GetRevoluteAngle returns the current angle of a revolute joint
func (j JointId) GetRevoluteAngle() float32 {
return float32(C.b2RevoluteJoint_GetAngle(j.id))
}

// SetRevoluteLimits sets the angle limits for a revolute joint
func (j JointId) SetRevoluteLimits(lower, upper float32) {
C.b2RevoluteJoint_SetLimits(j.id, C.float(lower), C.float(upper))
}

// GetRevoluteLowerLimit returns the lower angle limit of a revolute joint
func (j JointId) GetRevoluteLowerLimit() float32 {
return float32(C.b2RevoluteJoint_GetLowerLimit(j.id))
}

// GetRevoluteUpperLimit returns the upper angle limit of a revolute joint
func (j JointId) GetRevoluteUpperLimit() float32 {
return float32(C.b2RevoluteJoint_GetUpperLimit(j.id))
}

// EnableRevoluteMotor enables/disables the motor on a revolute joint
func (j JointId) EnableRevoluteMotor(enableMotor bool) {
C.b2RevoluteJoint_EnableMotor(j.id, C.bool(enableMotor))
}

// IsRevoluteMotorEnabled checks if the motor is enabled on a revolute joint
func (j JointId) IsRevoluteMotorEnabled() bool {
return bool(C.b2RevoluteJoint_IsMotorEnabled(j.id))
}

// SetRevoluteMotorSpeed sets the motor speed for a revolute joint
func (j JointId) SetRevoluteMotorSpeed(motorSpeed float32) {
C.b2RevoluteJoint_SetMotorSpeed(j.id, C.float(motorSpeed))
}

// GetRevoluteMotorSpeed returns the motor speed of a revolute joint
func (j JointId) GetRevoluteMotorSpeed() float32 {
return float32(C.b2RevoluteJoint_GetMotorSpeed(j.id))
}

// GetRevoluteMotorTorque returns the motor torque of a revolute joint
func (j JointId) GetRevoluteMotorTorque() float32 {
return float32(C.b2RevoluteJoint_GetMotorTorque(j.id))
}

// SetRevoluteMaxMotorTorque sets the maximum motor torque for a revolute joint
func (j JointId) SetRevoluteMaxMotorTorque(torque float32) {
C.b2RevoluteJoint_SetMaxMotorTorque(j.id, C.float(torque))
}

// GetRevoluteMaxMotorTorque returns the maximum motor torque of a revolute joint
func (j JointId) GetRevoluteMaxMotorTorque() float32 {
return float32(C.b2RevoluteJoint_GetMaxMotorTorque(j.id))
}

// ====================================================================
// Prismatic Joint Specific Functions
// ====================================================================

// EnablePrismaticLimit enables/disables the limit on a prismatic joint
func (j JointId) EnablePrismaticLimit(enableLimit bool) {
C.b2PrismaticJoint_EnableLimit(j.id, C.bool(enableLimit))
}

// IsPrismaticLimitEnabled checks if the limit is enabled on a prismatic joint
func (j JointId) IsPrismaticLimitEnabled() bool {
return bool(C.b2PrismaticJoint_IsLimitEnabled(j.id))
}

// GetPrismaticTranslation returns the current translation of a prismatic joint
func (j JointId) GetPrismaticTranslation() float32 {
return float32(C.b2PrismaticJoint_GetTranslation(j.id))
}

// SetPrismaticLimits sets the translation limits for a prismatic joint
func (j JointId) SetPrismaticLimits(lower, upper float32) {
C.b2PrismaticJoint_SetLimits(j.id, C.float(lower), C.float(upper))
}

// GetPrismaticLowerLimit returns the lower translation limit of a prismatic joint
func (j JointId) GetPrismaticLowerLimit() float32 {
return float32(C.b2PrismaticJoint_GetLowerLimit(j.id))
}

// GetPrismaticUpperLimit returns the upper translation limit of a prismatic joint
func (j JointId) GetPrismaticUpperLimit() float32 {
return float32(C.b2PrismaticJoint_GetUpperLimit(j.id))
}

// EnablePrismaticMotor enables/disables the motor on a prismatic joint
func (j JointId) EnablePrismaticMotor(enableMotor bool) {
C.b2PrismaticJoint_EnableMotor(j.id, C.bool(enableMotor))
}

// IsPrismaticMotorEnabled checks if the motor is enabled on a prismatic joint
func (j JointId) IsPrismaticMotorEnabled() bool {
return bool(C.b2PrismaticJoint_IsMotorEnabled(j.id))
}

// SetPrismaticMotorSpeed sets the motor speed for a prismatic joint
func (j JointId) SetPrismaticMotorSpeed(motorSpeed float32) {
C.b2PrismaticJoint_SetMotorSpeed(j.id, C.float(motorSpeed))
}

// GetPrismaticMotorSpeed returns the motor speed of a prismatic joint
func (j JointId) GetPrismaticMotorSpeed() float32 {
return float32(C.b2PrismaticJoint_GetMotorSpeed(j.id))
}

// SetPrismaticMaxMotorForce sets the maximum motor force for a prismatic joint
func (j JointId) SetPrismaticMaxMotorForce(force float32) {
C.b2PrismaticJoint_SetMaxMotorForce(j.id, C.float(force))
}

// GetPrismaticMaxMotorForce returns the maximum motor force of a prismatic joint
func (j JointId) GetPrismaticMaxMotorForce() float32 {
return float32(C.b2PrismaticJoint_GetMaxMotorForce(j.id))
}

// GetPrismaticMotorForce returns the motor force of a prismatic joint
func (j JointId) GetPrismaticMotorForce() float32 {
return float32(C.b2PrismaticJoint_GetMotorForce(j.id))
}

// ====================================================================
// Weld Joint Specific Functions
// ====================================================================

// SetWeldLinearHertz sets the linear frequency for a weld joint
func (j JointId) SetWeldLinearHertz(hertz float32) {
C.b2WeldJoint_SetLinearHertz(j.id, C.float(hertz))
}

// GetWeldLinearHertz returns the linear frequency of a weld joint
func (j JointId) GetWeldLinearHertz() float32 {
return float32(C.b2WeldJoint_GetLinearHertz(j.id))
}

// SetWeldLinearDampingRatio sets the linear damping ratio for a weld joint
func (j JointId) SetWeldLinearDampingRatio(dampingRatio float32) {
C.b2WeldJoint_SetLinearDampingRatio(j.id, C.float(dampingRatio))
}

// GetWeldLinearDampingRatio returns the linear damping ratio of a weld joint
func (j JointId) GetWeldLinearDampingRatio() float32 {
return float32(C.b2WeldJoint_GetLinearDampingRatio(j.id))
}

// SetWeldAngularHertz sets the angular frequency for a weld joint
func (j JointId) SetWeldAngularHertz(hertz float32) {
C.b2WeldJoint_SetAngularHertz(j.id, C.float(hertz))
}

// GetWeldAngularHertz returns the angular frequency of a weld joint
func (j JointId) GetWeldAngularHertz() float32 {
return float32(C.b2WeldJoint_GetAngularHertz(j.id))
}

// SetWeldAngularDampingRatio sets the angular damping ratio for a weld joint
func (j JointId) SetWeldAngularDampingRatio(dampingRatio float32) {
C.b2WeldJoint_SetAngularDampingRatio(j.id, C.float(dampingRatio))
}

// GetWeldAngularDampingRatio returns the angular damping ratio of a weld joint
func (j JointId) GetWeldAngularDampingRatio() float32 {
return float32(C.b2WeldJoint_GetAngularDampingRatio(j.id))
}

// ====================================================================
// Wheel Joint Specific Functions
// ====================================================================

// EnableWheelSpring enables/disables the spring on a wheel joint
func (j JointId) EnableWheelSpring(enableSpring bool) {
C.b2WheelJoint_EnableSpring(j.id, C.bool(enableSpring))
}

// IsWheelSpringEnabled checks if the spring is enabled on a wheel joint
func (j JointId) IsWheelSpringEnabled() bool {
return bool(C.b2WheelJoint_IsSpringEnabled(j.id))
}

// SetWheelSpringHertz sets the spring frequency for a wheel joint
func (j JointId) SetWheelSpringHertz(hertz float32) {
C.b2WheelJoint_SetSpringHertz(j.id, C.float(hertz))
}

// GetWheelSpringHertz returns the spring frequency of a wheel joint
func (j JointId) GetWheelSpringHertz() float32 {
return float32(C.b2WheelJoint_GetSpringHertz(j.id))
}

// SetWheelSpringDampingRatio sets the spring damping ratio for a wheel joint
func (j JointId) SetWheelSpringDampingRatio(dampingRatio float32) {
C.b2WheelJoint_SetSpringDampingRatio(j.id, C.float(dampingRatio))
}

// GetWheelSpringDampingRatio returns the spring damping ratio of a wheel joint
func (j JointId) GetWheelSpringDampingRatio() float32 {
return float32(C.b2WheelJoint_GetSpringDampingRatio(j.id))
}

// EnableWheelLimit enables/disables the limit on a wheel joint
func (j JointId) EnableWheelLimit(enableLimit bool) {
C.b2WheelJoint_EnableLimit(j.id, C.bool(enableLimit))
}

// IsWheelLimitEnabled checks if the limit is enabled on a wheel joint
func (j JointId) IsWheelLimitEnabled() bool {
return bool(C.b2WheelJoint_IsLimitEnabled(j.id))
}

// SetWheelLimits sets the translation limits for a wheel joint
func (j JointId) SetWheelLimits(lower, upper float32) {
C.b2WheelJoint_SetLimits(j.id, C.float(lower), C.float(upper))
}

// GetWheelLowerLimit returns the lower translation limit of a wheel joint
func (j JointId) GetWheelLowerLimit() float32 {
return float32(C.b2WheelJoint_GetLowerLimit(j.id))
}

// GetWheelUpperLimit returns the upper translation limit of a wheel joint
func (j JointId) GetWheelUpperLimit() float32 {
return float32(C.b2WheelJoint_GetUpperLimit(j.id))
}

// EnableWheelMotor enables/disables the motor on a wheel joint
func (j JointId) EnableWheelMotor(enableMotor bool) {
C.b2WheelJoint_EnableMotor(j.id, C.bool(enableMotor))
}

// IsWheelMotorEnabled checks if the motor is enabled on a wheel joint
func (j JointId) IsWheelMotorEnabled() bool {
return bool(C.b2WheelJoint_IsMotorEnabled(j.id))
}

// SetWheelMotorSpeed sets the motor speed for a wheel joint
func (j JointId) SetWheelMotorSpeed(motorSpeed float32) {
C.b2WheelJoint_SetMotorSpeed(j.id, C.float(motorSpeed))
}

// GetWheelMotorSpeed returns the motor speed of a wheel joint
func (j JointId) GetWheelMotorSpeed() float32 {
return float32(C.b2WheelJoint_GetMotorSpeed(j.id))
}

// SetWheelMaxMotorTorque sets the maximum motor torque for a wheel joint
func (j JointId) SetWheelMaxMotorTorque(torque float32) {
C.b2WheelJoint_SetMaxMotorTorque(j.id, C.float(torque))
}

// GetWheelMaxMotorTorque returns the maximum motor torque of a wheel joint
func (j JointId) GetWheelMaxMotorTorque() float32 {
return float32(C.b2WheelJoint_GetMaxMotorTorque(j.id))
}

// GetWheelMotorTorque returns the motor torque of a wheel joint
func (j JointId) GetWheelMotorTorque() float32 {
return float32(C.b2WheelJoint_GetMotorTorque(j.id))
}

// ====================================================================
// Motor Joint Specific Functions
// ====================================================================

// SetMotorJointLinearOffset sets the linear offset for a motor joint
func (j JointId) SetMotorJointLinearOffset(linearOffset Vec2) {
cOffset := C.b2Vec2{x: C.float(linearOffset.X), y: C.float(linearOffset.Y)}
C.b2MotorJoint_SetLinearVelocity(j.id, cOffset)
}

// GetMotorJointLinearOffset returns the linear offset of a motor joint
func (j JointId) GetMotorJointLinearOffset() Vec2 {
cOffset := C.b2MotorJoint_GetLinearVelocity(j.id)
return Vec2{X: float32(cOffset.x), Y: float32(cOffset.y)}
}

// SetMotorJointAngularOffset sets the angular offset for a motor joint
func (j JointId) SetMotorJointAngularOffset(angularOffset float32) {
C.b2MotorJoint_SetAngularVelocity(j.id, C.float(angularOffset))
}

// GetMotorJointAngularOffset returns the angular offset of a motor joint
func (j JointId) GetMotorJointAngularOffset() float32 {
return float32(C.b2MotorJoint_GetAngularVelocity(j.id))
}

// SetMotorJointMaxForce sets the maximum force for a motor joint
func (j JointId) SetMotorJointMaxForce(force float32) {
C.b2MotorJoint_SetMaxVelocityForce(j.id, C.float(force))
}

// GetMotorJointMaxForce returns the maximum force of a motor joint
func (j JointId) GetMotorJointMaxForce() float32 {
return float32(C.b2MotorJoint_GetMaxVelocityForce(j.id))
}

// SetMotorJointMaxTorque sets the maximum torque for a motor joint
func (j JointId) SetMotorJointMaxTorque(torque float32) {
C.b2MotorJoint_SetMaxVelocityTorque(j.id, C.float(torque))
}

// GetMotorJointMaxTorque returns the maximum torque of a motor joint
func (j JointId) GetMotorJointMaxTorque() float32 {
return float32(C.b2MotorJoint_GetMaxVelocityTorque(j.id))
}

// SetMotorJointCorrectionFactor sets the correction factor for a motor joint
func (j JointId) SetMotorJointCorrectionFactor(factor float32) {
C.b2MotorJoint_SetLinearHertz(j.id, C.float(factor))
}

// GetMotorJointCorrectionFactor returns the correction factor of a motor joint
func (j JointId) GetMotorJointCorrectionFactor() float32 {
return float32(C.b2MotorJoint_GetLinearHertz(j.id))
}

// ====================================================================
// Additional Math Functions
// ====================================================================

// Vec2Add adds two vectors
func Vec2Add(a, b Vec2) Vec2 {
cA := C.b2Vec2{x: C.float(a.X), y: C.float(a.Y)}
cB := C.b2Vec2{x: C.float(b.X), y: C.float(b.Y)}
cResult := C.b2Add(cA, cB)
return Vec2{X: float32(cResult.x), Y: float32(cResult.y)}
}

// Vec2Sub subtracts two vectors
func Vec2Sub(a, b Vec2) Vec2 {
cA := C.b2Vec2{x: C.float(a.X), y: C.float(a.Y)}
cB := C.b2Vec2{x: C.float(b.X), y: C.float(b.Y)}
cResult := C.b2Sub(cA, cB)
return Vec2{X: float32(cResult.x), Y: float32(cResult.y)}
}

// Vec2Length returns the length of a vector
func Vec2Length(v Vec2) float32 {
cV := C.b2Vec2{x: C.float(v.X), y: C.float(v.Y)}
return float32(C.b2Length(cV))
}

// Vec2Distance returns the distance between two points
func Vec2Distance(a, b Vec2) float32 {
cA := C.b2Vec2{x: C.float(a.X), y: C.float(a.Y)}
cB := C.b2Vec2{x: C.float(b.X), y: C.float(b.Y)}
return float32(C.b2Distance(cA, cB))
}

// Vec2Normalize normalizes a vector
func Vec2Normalize(v Vec2) Vec2 {
cV := C.b2Vec2{x: C.float(v.X), y: C.float(v.Y)}
cResult := C.b2Normalize(cV)
return Vec2{X: float32(cResult.x), Y: float32(cResult.y)}
}

// Vec2Dot returns the dot product of two vectors
func Vec2Dot(a, b Vec2) float32 {
cA := C.b2Vec2{x: C.float(a.X), y: C.float(a.Y)}
cB := C.b2Vec2{x: C.float(b.X), y: C.float(b.Y)}
return float32(C.b2Dot(cA, cB))
}

// Vec2Cross returns the cross product of two vectors (scalar result)
func Vec2Cross(a, b Vec2) float32 {
cA := C.b2Vec2{x: C.float(a.X), y: C.float(a.Y)}
cB := C.b2Vec2{x: C.float(b.X), y: C.float(b.Y)}
return float32(C.b2Cross(cA, cB))
}
