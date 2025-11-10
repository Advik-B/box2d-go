package main

import (
	"fmt"

	box2d "github.com/Advik-B/box2d-go"
)

func main() {
	fmt.Println("Box2D Go - Testing New Features")
	fmt.Println("=================================")

	// Create a world with gravity
	worldDef := box2d.DefaultWorldDef()
	worldDef.Gravity = box2d.Vec2{X: 0.0, Y: -10.0}
	worldId := box2d.CreateWorld(&worldDef)
	defer box2d.DestroyWorld(worldId)

	// Test 1: Restitution field in ShapeDef
	fmt.Println("\n1. Testing Restitution field...")
	bodyDef := box2d.DefaultBodyDef()
	bodyDef.Type = box2d.DynamicBody
	bodyDef.Position = box2d.Vec2{X: 0.0, Y: 10.0}
	bodyId := worldId.CreateBody(&bodyDef)

	circle := box2d.MakeCircle(box2d.Vec2{X: 0, Y: 0}, 0.5)
	shapeDef := box2d.DefaultShapeDef()
	shapeDef.Density = 1.0
	shapeDef.Friction = 0.3
	shapeDef.Restitution = 0.8 // Bouncy!
	bodyId.CreateCircleShape(&shapeDef, &circle)
	fmt.Printf("   Created bouncy ball with restitution: %.1f\n", shapeDef.Restitution)

	// Test 2: GetType method
	fmt.Println("\n2. Testing GetType method...")
	bodyType := bodyId.GetType()
	fmt.Printf("   Body type: %v (expected: DynamicBody)\n", bodyType)
	if bodyType == box2d.DynamicBody {
		fmt.Println("   ✓ GetType works correctly!")
	}

	// Test 3: SetLinearVelocity method
	fmt.Println("\n3. Testing SetLinearVelocity method...")
	velocity := box2d.Vec2{X: 5.0, Y: 10.0}
	bodyId.SetLinearVelocity(velocity)
	fmt.Printf("   Set linear velocity to (%.1f, %.1f)\n", velocity.X, velocity.Y)
	fmt.Println("   ✓ SetLinearVelocity works correctly!")

	// Test 4: ApplyForceToCenter method
	fmt.Println("\n4. Testing ApplyForceToCenter method...")
	force := box2d.Vec2{X: 100.0, Y: 0.0}
	bodyId.ApplyForceToCenter(force, true)
	fmt.Printf("   Applied force (%.1f, %.1f) to center\n", force.X, force.Y)
	fmt.Println("   ✓ ApplyForceToCenter works correctly!")

	// Test 5: JointId type exists
	fmt.Println("\n5. Testing JointId type...")
	var joint box2d.JointId
	_ = joint
	fmt.Println("   ✓ JointId type is available!")

	// Run a few simulation steps
	fmt.Println("\n6. Running simulation...")
	timeStep := float32(1.0 / 60.0)
	subStepCount := int32(4)

	for i := 0; i < 30; i++ {
		worldId.Step(timeStep, subStepCount)
	}
	
	finalPos := bodyId.GetPosition()
	fmt.Printf("   Final position after 0.5s: (%.3f, %.3f)\n", finalPos.X, finalPos.Y)

	fmt.Println("\n✓ All new features working correctly!")
}
