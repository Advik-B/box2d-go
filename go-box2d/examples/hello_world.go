package main

import (
	"fmt"

	box2d "github.com/Advik-B/box2d-go"
)

func main() {
	fmt.Println("Box2D Go Example - Falling Box")
	fmt.Println("================================")

	// Create a world with gravity
	worldDef := box2d.DefaultWorldDef()
	worldDef.Gravity = box2d.Vec2{X: 0.0, Y: -10.0}
	worldId := box2d.CreateWorld(&worldDef)
	defer box2d.DestroyWorld(worldId)

	// Create ground body
	groundBodyDef := box2d.DefaultBodyDef()
	groundBodyDef.Position = box2d.Vec2{X: 0.0, Y: -10.0}
	groundId := worldId.CreateBody(&groundBodyDef)

	// Create ground shape (box)
	groundBox := box2d.MakeBox(50.0, 10.0)
	groundShapeDef := box2d.DefaultShapeDef()
	groundId.CreatePolygonShape(&groundShapeDef, &groundBox)

	// Create dynamic body
	bodyDef := box2d.DefaultBodyDef()
	bodyDef.Type = box2d.DynamicBody
	bodyDef.Position = box2d.Vec2{X: 0.0, Y: 4.0}
	bodyId := worldId.CreateBody(&bodyDef)

	// Create dynamic box shape
	dynamicBox := box2d.MakeBox(1.0, 1.0)
	shapeDef := box2d.DefaultShapeDef()
	shapeDef.Density = 1.0
	shapeDef.Friction = 0.3
	bodyId.CreatePolygonShape(&shapeDef, &dynamicBox)

	fmt.Println("\nSimulating a 2x2 box falling from height 4.0...")
	fmt.Println("Time step: 1/60s, Sub-steps: 4")
	fmt.Println()

	// Simulate
	timeStep := float32(1.0 / 60.0)
	subStepCount := int32(4)

	for i := 0; i < 90; i++ {
		worldId.Step(timeStep, subStepCount)
		position := bodyId.GetPosition()
		angle := bodyId.GetAngle()

		// Print every 15 steps (quarter second)
		if i%15 == 0 {
			fmt.Printf("t=%.2fs: pos=(%.3f, %.3f) angle=%.3f rad\n",
				float32(i)*timeStep, position.X, position.Y, angle)
		}
	}

	// Final position
	position := bodyId.GetPosition()
	angle := bodyId.GetAngle()
	fmt.Printf("\nFinal position: (%.3f, %.3f) angle=%.3f rad\n", position.X, position.Y, angle)
	fmt.Println("\nBox has landed on the ground!")
}
