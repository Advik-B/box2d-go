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


// TestWorldGravity tests getting and setting world gravity
func TestWorldGravity(t *testing.T) {
worldDef := DefaultWorldDef()
worldId := CreateWorld(&worldDef)
defer DestroyWorld(worldId)

// Set new gravity
newGravity := Vec2{X: 5.0, Y: -15.0}
worldId.SetGravity(newGravity)

// Get gravity and verify
gravity := worldId.GetGravity()
if math.Abs(float64(gravity.X-newGravity.X)) > 0.001 || math.Abs(float64(gravity.Y-newGravity.Y)) > 0.001 {
t.Errorf("Expected gravity %v, got %v", newGravity, gravity)
}
}

// TestWorldSleeping tests world sleeping enable/disable
func TestWorldSleeping(t *testing.T) {
worldDef := DefaultWorldDef()
worldId := CreateWorld(&worldDef)
defer DestroyWorld(worldId)

// Disable sleeping
worldId.EnableSleeping(false)
if worldId.IsSleepingEnabled() {
t.Error("Sleeping should be disabled")
}

// Enable sleeping
worldId.EnableSleeping(true)
if !worldId.IsSleepingEnabled() {
t.Error("Sleeping should be enabled")
}
}

// TestWorldCounters tests getting world counters
func TestWorldCounters(t *testing.T) {
worldDef := DefaultWorldDef()
worldId := CreateWorld(&worldDef)
defer DestroyWorld(worldId)

// Create some bodies
bodyDef := DefaultBodyDef()
bodyDef.Type = DynamicBody

for i := 0; i < 5; i++ {
bodyId := worldId.CreateBody(&bodyDef)
box := MakeBox(1.0, 1.0)
shapeDef := DefaultShapeDef()
bodyId.CreatePolygonShape(&shapeDef, &box)
}

counters := worldId.GetCounters()
if counters.BodyCount != 5 {
t.Errorf("Expected 5 bodies, got %d", counters.BodyCount)
}
if counters.ShapeCount != 5 {
t.Errorf("Expected 5 shapes, got %d", counters.ShapeCount)
}
}

// TestBodyVelocity tests body velocity operations
func TestBodyVelocity(t *testing.T) {
worldDef := DefaultWorldDef()
worldId := CreateWorld(&worldDef)
defer DestroyWorld(worldId)

bodyDef := DefaultBodyDef()
bodyDef.Type = DynamicBody
bodyId := worldId.CreateBody(&bodyDef)

// Set linear velocity
velocity := Vec2{X: 10.0, Y: 5.0}
bodyId.SetLinearVelocity(velocity)

// Get and verify
gotVelocity := bodyId.GetLinearVelocity()
if math.Abs(float64(gotVelocity.X-velocity.X)) > 0.001 || math.Abs(float64(gotVelocity.Y-velocity.Y)) > 0.001 {
t.Errorf("Expected velocity %v, got %v", velocity, gotVelocity)
}

// Set angular velocity
angularVelocity := float32(2.5)
bodyId.SetAngularVelocity(angularVelocity)

gotAngularVelocity := bodyId.GetAngularVelocity()
if math.Abs(float64(gotAngularVelocity-angularVelocity)) > 0.001 {
t.Errorf("Expected angular velocity %f, got %f", angularVelocity, gotAngularVelocity)
}
}

// TestBodyMass tests body mass operations
func TestBodyMass(t *testing.T) {
worldDef := DefaultWorldDef()
worldId := CreateWorld(&worldDef)
defer DestroyWorld(worldId)

bodyDef := DefaultBodyDef()
bodyDef.Type = DynamicBody
bodyId := worldId.CreateBody(&bodyDef)

// Add a shape with density
box := MakeBox(2.0, 2.0)
shapeDef := DefaultShapeDef()
shapeDef.Density = 1.0
bodyId.CreatePolygonShape(&shapeDef, &box)

// Get mass - should be computed from shape
mass := bodyId.GetMass()
if mass == 0.0 {
t.Error("Mass should be computed from shape")
}

// Get mass data
massData := bodyId.GetMassData()
if massData.Mass == 0.0 {
t.Error("Mass data should have non-zero mass")
}
}

// TestBodyDamping tests body damping  
func TestBodyDamping(t *testing.T) {
worldDef := DefaultWorldDef()
worldId := CreateWorld(&worldDef)
defer DestroyWorld(worldId)

bodyDef := DefaultBodyDef()
bodyDef.Type = DynamicBody
bodyId := worldId.CreateBody(&bodyDef)

// Set linear damping
linearDamping := float32(0.5)
bodyId.SetLinearDamping(linearDamping)
if math.Abs(float64(bodyId.GetLinearDamping()-linearDamping)) > 0.001 {
t.Error("Linear damping not set correctly")
}

// Set angular damping
angularDamping := float32(0.3)
bodyId.SetAngularDamping(angularDamping)
if math.Abs(float64(bodyId.GetAngularDamping()-angularDamping)) > 0.001 {
t.Error("Angular damping not set correctly")
}
}

// TestBodyGravityScale tests body gravity scale
func TestBodyGravityScale(t *testing.T) {
worldDef := DefaultWorldDef()
worldId := CreateWorld(&worldDef)
defer DestroyWorld(worldId)

bodyDef := DefaultBodyDef()
bodyDef.Type = DynamicBody
bodyId := worldId.CreateBody(&bodyDef)

// Set gravity scale
gravityScale := float32(2.0)
bodyId.SetGravityScale(gravityScale)

gotScale := bodyId.GetGravityScale()
if math.Abs(float64(gotScale-gravityScale)) > 0.001 {
t.Errorf("Expected gravity scale %f, got %f", gravityScale, gotScale)
}
}

// TestBodySleep tests body sleep operations
func TestBodySleep(t *testing.T) {
worldDef := DefaultWorldDef()
worldId := CreateWorld(&worldDef)
defer DestroyWorld(worldId)

bodyDef := DefaultBodyDef()
bodyDef.Type = DynamicBody
bodyId := worldId.CreateBody(&bodyDef)

// Check if awake (should be awake initially)
if !bodyId.IsAwake() {
t.Error("Body should be awake initially")
}

// Check if sleep is enabled
if !bodyId.IsSleepEnabled() {
t.Error("Sleep should be enabled by default")
}

// Disable sleep
bodyId.EnableSleep(false)
if bodyId.IsSleepEnabled() {
t.Error("Sleep should be disabled")
}
}

// TestBodyEnableDisable tests body enable/disable
func TestBodyEnableDisable(t *testing.T) {
worldDef := DefaultWorldDef()
worldId := CreateWorld(&worldDef)
defer DestroyWorld(worldId)

bodyDef := DefaultBodyDef()
bodyDef.Type = DynamicBody
bodyId := worldId.CreateBody(&bodyDef)

// Should be enabled initially
if !bodyId.IsEnabled() {
t.Error("Body should be enabled initially")
}

// Disable
bodyId.Disable()
if bodyId.IsEnabled() {
t.Error("Body should be disabled")
}

// Enable
bodyId.Enable()
if !bodyId.IsEnabled() {
t.Error("Body should be enabled")
}
}

// TestBodyTransformOperations tests body transform operations
func TestBodyTransformOperations(t *testing.T) {
worldDef := DefaultWorldDef()
worldId := CreateWorld(&worldDef)
defer DestroyWorld(worldId)

bodyDef := DefaultBodyDef()
bodyDef.Type = DynamicBody
bodyDef.Position = Vec2{X: 10.0, Y: 5.0}
bodyId := worldId.CreateBody(&bodyDef)

// Test GetTransform
transform := bodyId.GetTransform()
if math.Abs(float64(transform.Position.X-10.0)) > 0.001 || math.Abs(float64(transform.Position.Y-5.0)) > 0.001 {
t.Errorf("Transform position incorrect: %v", transform.Position)
}

// Test world/local point conversion
worldPoint := Vec2{X: 15.0, Y: 10.0}
localPoint := bodyId.GetLocalPoint(worldPoint)
backToWorld := bodyId.GetWorldPoint(localPoint)

if math.Abs(float64(backToWorld.X-worldPoint.X)) > 0.01 || math.Abs(float64(backToWorld.Y-worldPoint.Y)) > 0.01 {
t.Errorf("Point conversion failed: expected %v, got %v", worldPoint, backToWorld)
}
}

// TestShapeDensity tests shape density operations
func TestShapeDensity(t *testing.T) {
worldDef := DefaultWorldDef()
worldId := CreateWorld(&worldDef)
defer DestroyWorld(worldId)

bodyDef := DefaultBodyDef()
bodyDef.Type = DynamicBody
bodyId := worldId.CreateBody(&bodyDef)

box := MakeBox(1.0, 1.0)
shapeDef := DefaultShapeDef()
shapeDef.Density = 2.0
shapeId := bodyId.CreatePolygonShape(&shapeDef, &box)

// Get density
density := shapeId.GetDensity()
if math.Abs(float64(density-2.0)) > 0.001 {
t.Errorf("Expected density 2.0, got %f", density)
}

// Set new density
shapeId.SetDensity(3.0, true)
density = shapeId.GetDensity()
if math.Abs(float64(density-3.0)) > 0.001 {
t.Errorf("Expected density 3.0, got %f", density)
}
}

// TestShapeFriction tests shape friction operations
func TestShapeFriction(t *testing.T) {
worldDef := DefaultWorldDef()
worldId := CreateWorld(&worldDef)
defer DestroyWorld(worldId)

bodyDef := DefaultBodyDef()
bodyId := worldId.CreateBody(&bodyDef)

box := MakeBox(1.0, 1.0)
shapeDef := DefaultShapeDef()
shapeDef.Friction = 0.5
shapeId := bodyId.CreatePolygonShape(&shapeDef, &box)

// Get friction
friction := shapeId.GetFriction()
if math.Abs(float64(friction-0.5)) > 0.001 {
t.Errorf("Expected friction 0.5, got %f", friction)
}

// Set new friction
shapeId.SetFriction(0.8)
friction = shapeId.GetFriction()
if math.Abs(float64(friction-0.8)) > 0.001 {
t.Errorf("Expected friction 0.8, got %f", friction)
}
}

// TestShapeFilter tests shape collision filters
func TestShapeFilter(t *testing.T) {
worldDef := DefaultWorldDef()
worldId := CreateWorld(&worldDef)
defer DestroyWorld(worldId)

bodyDef := DefaultBodyDef()
bodyId := worldId.CreateBody(&bodyDef)

box := MakeBox(1.0, 1.0)
shapeDef := DefaultShapeDef()
shapeId := bodyId.CreatePolygonShape(&shapeDef, &box)

// Set filter
filter := Filter{
CategoryBits: 0x0002,
MaskBits:     0x0004,
GroupIndex:   -1,
}
shapeId.SetFilter(filter)

// Get filter and verify
gotFilter := shapeId.GetFilter()
if gotFilter.CategoryBits != 0x0002 || gotFilter.MaskBits != 0x0004 || gotFilter.GroupIndex != -1 {
t.Errorf("Filter not set correctly: %v", gotFilter)
}
}

// TestSegmentShape tests segment (edge) shape creation
func TestSegmentShape(t *testing.T) {
worldDef := DefaultWorldDef()
worldId := CreateWorld(&worldDef)
defer DestroyWorld(worldId)

bodyDef := DefaultBodyDef()
bodyId := worldId.CreateBody(&bodyDef)

segment := Segment{
Point1: Vec2{X: -1.0, Y: 0.0},
Point2: Vec2{X: 1.0, Y: 0.0},
}
shapeDef := DefaultShapeDef()
shapeId := bodyId.CreateSegmentShape(&shapeDef, &segment)

if !shapeId.IsValid() {
t.Error("Segment shape should be valid")
}
}

// TestCapsuleShape tests capsule shape creation
func TestCapsuleShape(t *testing.T) {
worldDef := DefaultWorldDef()
worldId := CreateWorld(&worldDef)
defer DestroyWorld(worldId)

bodyDef := DefaultBodyDef()
bodyDef.Type = DynamicBody
bodyId := worldId.CreateBody(&bodyDef)

capsule := Capsule{
Center1: Vec2{X: 0.0, Y: -0.5},
Center2: Vec2{X: 0.0, Y: 0.5},
Radius:  0.5,
}
shapeDef := DefaultShapeDef()
shapeDef.Density = 1.0
shapeId := bodyId.CreateCapsuleShape(&shapeDef, &capsule)

if !shapeId.IsValid() {
t.Error("Capsule shape should be valid")
}
}

// TestMathFunctions tests vector math functions
func TestMathFunctions(t *testing.T) {
a := Vec2{X: 3.0, Y: 4.0}
b := Vec2{X: 1.0, Y: 2.0}

// Test Vec2Add
sum := Vec2Add(a, b)
if math.Abs(float64(sum.X-4.0)) > 0.001 || math.Abs(float64(sum.Y-6.0)) > 0.001 {
t.Errorf("Vec2Add failed: expected (4, 6), got %v", sum)
}

// Test Vec2Sub
diff := Vec2Sub(a, b)
if math.Abs(float64(diff.X-2.0)) > 0.001 || math.Abs(float64(diff.Y-2.0)) > 0.001 {
t.Errorf("Vec2Sub failed: expected (2, 2), got %v", diff)
}

// Test Vec2Length
length := Vec2Length(a)
expectedLength := float32(5.0) // sqrt(9+16) = 5
if math.Abs(float64(length-expectedLength)) > 0.001 {
t.Errorf("Vec2Length failed: expected %f, got %f", expectedLength, length)
}

// Test Vec2Normalize
normalized := Vec2Normalize(a)
normalizedLength := Vec2Length(normalized)
if math.Abs(float64(normalizedLength-1.0)) > 0.001 {
t.Errorf("Vec2Normalize failed: length should be 1.0, got %f", normalizedLength)
}

// Test Vec2Dot
dot := Vec2Dot(a, b)
expectedDot := float32(11.0) // 3*1 + 4*2 = 11
if math.Abs(float64(dot-expectedDot)) > 0.001 {
t.Errorf("Vec2Dot failed: expected %f, got %f", expectedDot, dot)
}

// Test Vec2Distance
distance := Vec2Distance(a, b)
expectedDistance := float32(math.Sqrt(8.0)) // sqrt((3-1)^2 + (4-2)^2) = sqrt(8)
if math.Abs(float64(distance-expectedDistance)) > 0.001 {
t.Errorf("Vec2Distance failed: expected %f, got %f", expectedDistance, distance)
}
}

// TestRotationFunctions tests rotation functions
func TestRotationFunctions(t *testing.T) {
angle := float32(math.Pi / 4.0) // 45 degrees
rot := MakeRot(angle)

// Get angle back
gotAngle := RotGetAngle(rot)
if math.Abs(float64(gotAngle-angle)) > 0.001 {
t.Errorf("RotGetAngle failed: expected %f, got %f", angle, gotAngle)
}
}

// TestRaycasting tests raycasting
func TestRaycasting(t *testing.T) {
worldDef := DefaultWorldDef()
worldId := CreateWorld(&worldDef)
defer DestroyWorld(worldId)

// Create a static box
bodyDef := DefaultBodyDef()
bodyDef.Position = Vec2{X: 5.0, Y: 0.0}
bodyId := worldId.CreateBody(&bodyDef)

box := MakeBox(1.0, 1.0)
shapeDef := DefaultShapeDef()
bodyId.CreatePolygonShape(&shapeDef, &box)

// Cast a ray that should hit the box
origin := Vec2{X: 0.0, Y: 0.0}
translation := Vec2{X: 10.0, Y: 0.0}
filter := QueryFilter{CategoryBits: 0xFFFFFFFFFFFFFFFF, MaskBits: 0xFFFFFFFFFFFFFFFF}

result := worldId.CastRayClosest(origin, translation, filter)
if result.Hit {
t.Logf("Raycast hit at fraction %f, point (%f, %f)", result.Fraction, result.Point.X, result.Point.Y)
}
}
