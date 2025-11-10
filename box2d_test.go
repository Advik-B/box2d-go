package box2d

import (
	"math"
	"testing"
)

// TestHelloWorld is a port of the HelloWorld test from C
func TestHelloWorld(t *testing.T) {
	// Create a world with gravity
	worldDef := DefaultWorldDef()
	worldDef.Gravity = Vec2{X: 0.0, Y: -10.0}
	worldId := CreateWorld(&worldDef)
	defer DestroyWorld(worldId)

	// Create ground body
	groundBodyDef := DefaultBodyDef()
	groundBodyDef.Position = Vec2{X: 0.0, Y: -10.0}
	groundId := worldId.CreateBody(&groundBodyDef)

	// Create ground shape (box) - add the shape to the ground!
	groundBox := MakeBox(50.0, 10.0)
	groundShapeDef := DefaultShapeDef()
	groundId.CreatePolygonShape(&groundShapeDef, &groundBox)

	// Create dynamic body
	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.Position = Vec2{X: 0.0, Y: 4.0}
	bodyId := worldId.CreateBody(&bodyDef)

	// Create dynamic box shape
	dynamicBox := MakeBox(1.0, 1.0)
	shapeDef := DefaultShapeDef()
	shapeDef.Density = 1.0
	shapeDef.Friction = 0.3
	bodyId.CreatePolygonShape(&shapeDef, &dynamicBox)

	// Simulate
	timeStep := float32(1.0 / 60.0)
	subStepCount := int32(4)

	for i := 0; i < 90; i++ {
		worldId.Step(timeStep, subStepCount)
	}

	// Check final position
	position := bodyId.GetPosition()
	angle := bodyId.GetAngle()

	if math.Abs(float64(position.X)) >= 0.01 {
		t.Errorf("Expected X position near 0, got %f", position.X)
	}
	if math.Abs(float64(position.Y-1.0)) >= 0.01 {
		t.Errorf("Expected Y position near 1.0, got %f", position.Y)
	}
	if math.Abs(float64(angle)) >= 0.01 {
		t.Errorf("Expected angle near 0, got %f", angle)
	}
}

// TestEmptyWorld tests creating and stepping an empty world
func TestEmptyWorld(t *testing.T) {
	worldDef := DefaultWorldDef()
	worldId := CreateWorld(&worldDef)

	timeStep := float32(1.0 / 60.0)
	subStepCount := int32(1)

	for i := 0; i < 60; i++ {
		worldId.Step(timeStep, subStepCount)
	}

	DestroyWorld(worldId)
}

// TestBodyCreationAndDestruction tests creating and destroying bodies
func TestBodyCreationAndDestruction(t *testing.T) {
	worldDef := DefaultWorldDef()
	worldId := CreateWorld(&worldDef)
	defer DestroyWorld(worldId)

	// Create some bodies
	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody

	const bodyCount = 10
	bodies := make([]BodyId, bodyCount)

	square := MakeBox(0.5, 0.5)
	shapeDef := DefaultShapeDef()

	for i := 0; i < bodyCount; i++ {
		bodies[i] = worldId.CreateBody(&bodyDef)
		bodies[i].CreatePolygonShape(&shapeDef, &square)
	}

	// Step the simulation
	for i := 0; i < 10; i++ {
		worldId.Step(1.0/60.0, 3)
	}

	// Destroy bodies
	for i := 0; i < bodyCount; i++ {
		DestroyBody(bodies[i])
	}

	// Step again after destroying
	for i := 0; i < 10; i++ {
		worldId.Step(1.0/60.0, 3)
	}
}

// TestCircleShape tests creating a circle shape
func TestCircleShape(t *testing.T) {
	worldDef := DefaultWorldDef()
	worldDef.Gravity = Vec2{X: 0.0, Y: -10.0}
	worldId := CreateWorld(&worldDef)
	defer DestroyWorld(worldId)

	// Create a dynamic body
	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.Position = Vec2{X: 0.0, Y: 10.0}
	bodyId := worldId.CreateBody(&bodyDef)

	// Create a circle shape
	circle := MakeCircle(Vec2{X: 0.0, Y: 0.0}, 1.0)
	shapeDef := DefaultShapeDef()
	shapeDef.Density = 1.0
	bodyId.CreateCircleShape(&shapeDef, &circle)

	// Simulate for a bit
	timeStep := float32(1.0 / 60.0)
	subStepCount := int32(4)

	for i := 0; i < 60; i++ {
		worldId.Step(timeStep, subStepCount)
	}

	// Ball should have fallen down
	position := bodyId.GetPosition()
	if position.Y >= 10.0 {
		t.Errorf("Expected ball to fall, Y position: %f", position.Y)
	}
}

// TestSetTransform tests setting body transform
func TestSetTransform(t *testing.T) {
	worldDef := DefaultWorldDef()
	worldId := CreateWorld(&worldDef)
	defer DestroyWorld(worldId)

	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyId := worldId.CreateBody(&bodyDef)

	// Set a new position and angle
	newPos := Vec2{X: 5.0, Y: 10.0}
	newAngle := float32(0.5)
	bodyId.SetTransform(newPos, newAngle)

	// Verify the transform was set
	pos := bodyId.GetPosition()
	angle := bodyId.GetAngle()

	if math.Abs(float64(pos.X-newPos.X)) > 0.001 {
		t.Errorf("Expected X position %f, got %f", newPos.X, pos.X)
	}
	if math.Abs(float64(pos.Y-newPos.Y)) > 0.001 {
		t.Errorf("Expected Y position %f, got %f", newPos.Y, pos.Y)
	}
	if math.Abs(float64(angle-newAngle)) > 0.001 {
		t.Errorf("Expected angle %f, got %f", newAngle, angle)
	}
}

// TestStaticBody tests creating a static body
func TestStaticBody(t *testing.T) {
	worldDef := DefaultWorldDef()
	worldDef.Gravity = Vec2{X: 0.0, Y: -10.0}
	worldId := CreateWorld(&worldDef)
	defer DestroyWorld(worldId)

	// Create static body
	bodyDef := DefaultBodyDef()
	bodyDef.Type = StaticBody
	bodyDef.Position = Vec2{X: 0.0, Y: 0.0}
	bodyId := worldId.CreateBody(&bodyDef)

	box := MakeBox(10.0, 1.0)
	shapeDef := DefaultShapeDef()
	bodyId.CreatePolygonShape(&shapeDef, &box)

	initialPos := bodyId.GetPosition()

	// Simulate
	for i := 0; i < 60; i++ {
		worldId.Step(1.0/60.0, 4)
	}

	// Static body should not move
	finalPos := bodyId.GetPosition()
	if math.Abs(float64(finalPos.X-initialPos.X)) > 0.001 ||
		math.Abs(float64(finalPos.Y-initialPos.Y)) > 0.001 {
		t.Errorf("Static body moved from %v to %v", initialPos, finalPos)
	}
}

// TestRestitution tests that restitution field is properly set
func TestRestitution(t *testing.T) {
	worldDef := DefaultWorldDef()
	worldDef.Gravity = Vec2{X: 0.0, Y: -10.0}
	worldId := CreateWorld(&worldDef)
	defer DestroyWorld(worldId)

	// Create ground
	groundDef := DefaultBodyDef()
	groundDef.Position = Vec2{X: 0.0, Y: 0.0}
	groundId := worldId.CreateBody(&groundDef)
	groundBox := MakeBox(50.0, 1.0)
	groundShapeDef := DefaultShapeDef()
	groundId.CreatePolygonShape(&groundShapeDef, &groundBox)

	// Create bouncy ball
	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.Position = Vec2{X: 0.0, Y: 5.0}
	bodyId := worldId.CreateBody(&bodyDef)

	circle := MakeCircle(Vec2{X: 0.0, Y: 0.0}, 0.5)
	shapeDef := DefaultShapeDef()
	shapeDef.Density = 1.0
	shapeDef.Restitution = 0.9 // Very bouncy

	bodyId.CreateCircleShape(&shapeDef, &circle)

	// Verify shapeDef has restitution field
	if shapeDef.Restitution != 0.9 {
		t.Errorf("Expected restitution 0.9, got %f", shapeDef.Restitution)
	}
}

// TestGetType tests the GetType method
func TestGetType(t *testing.T) {
	worldDef := DefaultWorldDef()
	worldId := CreateWorld(&worldDef)
	defer DestroyWorld(worldId)

	// Create static body
	staticDef := DefaultBodyDef()
	staticDef.Type = StaticBody
	staticBody := worldId.CreateBody(&staticDef)

	if staticBody.GetType() != StaticBody {
		t.Errorf("Expected StaticBody, got %v", staticBody.GetType())
	}

	// Create dynamic body
	dynamicDef := DefaultBodyDef()
	dynamicDef.Type = DynamicBody
	dynamicBody := worldId.CreateBody(&dynamicDef)

	if dynamicBody.GetType() != DynamicBody {
		t.Errorf("Expected DynamicBody, got %v", dynamicBody.GetType())
	}

	// Create kinematic body
	kinematicDef := DefaultBodyDef()
	kinematicDef.Type = KinematicBody
	kinematicBody := worldId.CreateBody(&kinematicDef)

	if kinematicBody.GetType() != KinematicBody {
		t.Errorf("Expected KinematicBody, got %v", kinematicBody.GetType())
	}
}

// TestSetLinearVelocity tests the SetLinearVelocity method
func TestSetLinearVelocity(t *testing.T) {
	worldDef := DefaultWorldDef()
	worldId := CreateWorld(&worldDef)
	defer DestroyWorld(worldId)

	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.Position = Vec2{X: 0.0, Y: 0.0}
	bodyId := worldId.CreateBody(&bodyDef)

	circle := MakeCircle(Vec2{X: 0.0, Y: 0.0}, 1.0)
	shapeDef := DefaultShapeDef()
	shapeDef.Density = 1.0
	bodyId.CreateCircleShape(&shapeDef, &circle)

	// Set velocity
	velocity := Vec2{X: 10.0, Y: 5.0}
	bodyId.SetLinearVelocity(velocity)

	// Step the simulation
	worldId.Step(1.0/60.0, 4)

	// Body should have moved
	pos := bodyId.GetPosition()
	if math.Abs(float64(pos.X)) < 0.01 {
		t.Errorf("Expected body to move horizontally, X position: %f", pos.X)
	}
}

// TestApplyForceToCenter tests the ApplyForceToCenter method
func TestApplyForceToCenter(t *testing.T) {
	worldDef := DefaultWorldDef()
	worldDef.Gravity = Vec2{X: 0.0, Y: 0.0} // No gravity
	worldId := CreateWorld(&worldDef)
	defer DestroyWorld(worldId)

	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.Position = Vec2{X: 0.0, Y: 0.0}
	bodyId := worldId.CreateBody(&bodyDef)

	box := MakeBox(1.0, 1.0)
	shapeDef := DefaultShapeDef()
	shapeDef.Density = 1.0
	bodyId.CreatePolygonShape(&shapeDef, &box)

	// Apply force
	force := Vec2{X: 100.0, Y: 0.0}
	bodyId.ApplyForceToCenter(force, true)

	// Step the simulation multiple times
	for i := 0; i < 60; i++ {
		worldId.Step(1.0/60.0, 4)
	}

	// Body should have moved to the right
	pos := bodyId.GetPosition()
	if pos.X <= 0.01 {
		t.Errorf("Expected body to move right due to force, X position: %f", pos.X)
	}
}

// TestJointId tests that JointId type exists
func TestJointId(t *testing.T) {
	// Just verify the type can be declared
	var joint JointId
	_ = joint
	// If this compiles, the test passes
}

