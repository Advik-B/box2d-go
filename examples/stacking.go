package main

import (
	"fmt"

	box2d "github.com/Advik-B/box2d-go"
)

func main() {
	fmt.Println("Box2D Go Example - Stacking Boxes")
	fmt.Println("==================================")

	// Create a world with gravity
	worldDef := box2d.DefaultWorldDef()
	worldDef.Gravity = box2d.Vec2{X: 0.0, Y: -10.0}
	worldId := box2d.CreateWorld(&worldDef)
	defer box2d.DestroyWorld(worldId)

	// Create ground body
	groundBodyDef := box2d.DefaultBodyDef()
	groundBodyDef.Position = box2d.Vec2{X: 0.0, Y: 0.0}
	groundId := worldId.CreateBody(&groundBodyDef)

	// Create ground shape (wide box)
	groundBox := box2d.MakeBox(50.0, 1.0)
	groundShapeDef := box2d.DefaultShapeDef()
	groundId.CreatePolygonShape(&groundShapeDef, &groundBox)

	fmt.Println("Creating a stack of 5 boxes...")

	// Create a stack of boxes
	boxCount := 5
	bodies := make([]box2d.BodyId, boxCount)

	for i := 0; i < boxCount; i++ {
		bodyDef := box2d.DefaultBodyDef()
		bodyDef.Type = box2d.DynamicBody
		bodyDef.Position = box2d.Vec2{X: 0.0, Y: float32(2.0 + i*2.0)}
		bodies[i] = worldId.CreateBody(&bodyDef)

		// Create box shape
		boxShape := box2d.MakeBox(0.5, 0.5)
		shapeDef := box2d.DefaultShapeDef()
		shapeDef.Density = 1.0
		shapeDef.Friction = 0.3
		bodies[i].CreatePolygonShape(&shapeDef, &boxShape)
	}

	fmt.Println("\nSimulating stack settling...")
	fmt.Println()

	// Simulate
	timeStep := float32(1.0 / 60.0)
	subStepCount := int32(4)

	for step := 0; step < 180; step++ {
		worldId.Step(timeStep, subStepCount)

		// Print positions every 30 steps (half second)
		if step%30 == 0 {
			fmt.Printf("t=%.1fs:\n", float32(step)*timeStep)
			for i := 0; i < boxCount; i++ {
				pos := bodies[i].GetPosition()
				fmt.Printf("  Box %d: y=%.3f\n", i+1, pos.Y)
			}
			fmt.Println()
		}
	}

	fmt.Println("Stack has settled!")
}
