package main

import (
	"fmt"

	box2d "github.com/Advik-B/box2d-go"
)

// This example demonstrates the correct usage of the box2d-go API
// addressing the build errors from the issue.

const (
	screenWidth  = 1200
	screenHeight = 800
	worldScale   = 20.0 // pixels per meter
)

// Helper function to convert screen coordinates to world coordinates
// Note: Use float32, not float64, as Vec2 expects float32 values
func screenToWorld(x, y float32) box2d.Vec2 {
	return box2d.Vec2{
		X: (x - screenWidth/2) / worldScale,  // Direct float32 calculation
		Y: (screenHeight/2 - y) / worldScale, // Direct float32 calculation
	}
}

func main() {
	fmt.Println("Box2D Go - Correct API Usage Example")
	fmt.Println("=====================================")

	// Create Box2D world with gravity
	worldDef := box2d.DefaultWorldDef()
	worldDef.Gravity = box2d.Vec2{X: 0.0, Y: -10.0}
	worldId := box2d.CreateWorld(&worldDef)
	defer box2d.DestroyWorld(worldId)

	// Create ground (static body)
	groundBodyDef := box2d.DefaultBodyDef()
	groundBodyDef.Position = box2d.Vec2{X: 0.0, Y: -12.0}
	groundId := worldId.CreateBody(&groundBodyDef)
	groundBox := box2d.MakeBox(25.0, 1.0)
	groundShapeDef := box2d.DefaultShapeDef()
	groundShapeDef.Friction = 0.5
	groundId.CreatePolygonShape(&groundShapeDef, &groundBox)

	fmt.Println("\n1. Creating pyramid of boxes with restitution...")
	pyramidRows := 5
	for row := 0; row < pyramidRows; row++ {
		numBoxes := pyramidRows - row
		startX := -float32(numBoxes-1) * 0.65

		for i := 0; i < numBoxes; i++ {
			bodyDef := box2d.DefaultBodyDef()
			bodyDef.Type = box2d.DynamicBody
			// Correct: Use float32 directly, not float64 conversion
			bodyDef.Position = box2d.Vec2{
				X: startX + float32(i)*1.3,
				Y: -10.0 + float32(row)*1.1,
			}

			bodyId := worldId.CreateBody(&bodyDef)
			boxShape := box2d.MakeBox(0.5, 0.5)
			shapeDef := box2d.DefaultShapeDef()
			shapeDef.Density = 1.0
			shapeDef.Friction = 0.3
			shapeDef.Restitution = 0.1 // Now supported!
			bodyId.CreatePolygonShape(&shapeDef, &boxShape)
		}
	}
	fmt.Printf("   Created %d boxes\n", pyramidRows*(pyramidRows+1)/2)

	fmt.Println("\n2. Creating bouncing balls with restitution...")
	for i := 0; i < 4; i++ {
		bodyDef := box2d.DefaultBodyDef()
		bodyDef.Type = box2d.DynamicBody
		// Correct: Use float32 directly
		bodyDef.Position = box2d.Vec2{
			X: -15.0 + float32(i)*4.0,
			Y: 8.0,
		}

		bodyId := worldId.CreateBody(&bodyDef)
		circle := box2d.MakeCircle(box2d.Vec2{X: 0, Y: 0}, 0.4)
		shapeDef := box2d.DefaultShapeDef()
		shapeDef.Density = 0.8
		shapeDef.Friction = 0.2
		shapeDef.Restitution = 0.7 // Bouncy! (now supported)
		bodyId.CreateCircleShape(&shapeDef, &circle)
	}
	fmt.Println("   Created 4 bouncing balls")

	fmt.Println("\n3. Creating a wrecking ball with velocity...")
	worldMousePos := box2d.Vec2{X: 5.0, Y: 10.0}
	bodyDef := box2d.DefaultBodyDef()
	bodyDef.Type = box2d.DynamicBody
	bodyDef.Position = worldMousePos

	bodyId := worldId.CreateBody(&bodyDef)
	circle := box2d.MakeCircle(box2d.Vec2{X: 0, Y: 0}, 0.8)
	shapeDef := box2d.DefaultShapeDef()
	shapeDef.Density = 5.0
	shapeDef.Friction = 0.3
	shapeDef.Restitution = 0.5
	bodyId.CreateCircleShape(&shapeDef, &circle)

	// Use SetLinearVelocity (now supported)
	bodyId.SetLinearVelocity(box2d.Vec2{X: 0, Y: -15.0})
	fmt.Println("   Created wrecking ball with initial velocity")

	fmt.Println("\n4. Testing body type check...")
	if bodyId.GetType() == box2d.DynamicBody {
		fmt.Println("   ✓ Body is dynamic (using GetType method)")
	}

	fmt.Println("\n5. Applying force to a random object...")
	// Apply force using ApplyForceToCenter (now supported)
	force := box2d.Vec2{X: 50.0, Y: 100.0}
	bodyId.ApplyForceToCenter(force, true)
	fmt.Printf("   Applied force (%.1f, %.1f) to center\n", force.X, force.Y)

	fmt.Println("\n6. Testing JointId type...")
	var mouseJoint box2d.JointId // JointId type now available
	_ = mouseJoint
	fmt.Println("   ✓ JointId type is available")

	// Run simulation
	fmt.Println("\n7. Running physics simulation...")
	timeStep := float32(1.0 / 60.0)
	subStepCount := int32(4)

	for i := 0; i < 60; i++ {
		worldId.Step(timeStep, subStepCount)
	}

	pos := bodyId.GetPosition()
	fmt.Printf("   Wrecking ball position after 1 second: (%.2f, %.2f)\n", pos.X, pos.Y)

	fmt.Println("\n✓ All API features work correctly!")
	fmt.Println("\nKey points:")
	fmt.Println("  - Vec2 uses float32, not float64")
	fmt.Println("  - ShapeDef now supports Restitution field")
	fmt.Println("  - BodyId has GetType(), SetLinearVelocity(), ApplyForceToCenter() methods")
	fmt.Println("  - JointId type is now available")
}
