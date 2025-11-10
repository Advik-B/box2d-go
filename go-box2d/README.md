# Box2D Go Library

Go bindings for the Box2D physics engine using CGO.

## Overview

This library provides Go bindings to Box2D v3.2.0, a 2D physics engine for games. The bindings use CGO to interface with the native C library.

## Features

- World creation and simulation
- Dynamic, static, and kinematic bodies
- Circle and polygon (box) shapes
- Physics stepping with configurable substeps
- Collision detection and response

## Building

The library requires CGO to be enabled and a C compiler to be available. The Box2D C library is built automatically using CMake.

### Prerequisites

- Go 1.16 or later
- CMake 3.22 or later
- A C compiler (gcc, clang, etc.)

### Installation

```bash
cd go-box2d/clib
cmake .
make
cd ..
go build
```

## Usage

Here's a simple example that creates a world, adds a ground and a dynamic box, and runs the simulation:

```go
package main

import (
    "fmt"
    box2d "github.com/Advik-B/box2d-go"
)

func main() {
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

    // Simulate
    timeStep := float32(1.0 / 60.0)
    subStepCount := int32(4)

    for i := 0; i < 60; i++ {
        worldId.Step(timeStep, subStepCount)
        position := bodyId.GetPosition()
        angle := bodyId.GetAngle()
        fmt.Printf("Step %d: pos=(%.2f, %.2f) angle=%.2f\n", i, position.X, position.Y, angle)
    }
}
```

## Testing

Run the tests with:

```bash
go test -v
```

## API Overview

### Types

- `WorldId` - Handle to a physics world
- `BodyId` - Handle to a rigid body
- `ShapeId` - Handle to a shape
- `Vec2` - 2D vector with X and Y components
- `WorldDef` - World definition with gravity
- `BodyDef` - Body definition with type, position, and angle
- `ShapeDef` - Shape definition with density and friction
- `BodyType` - Body type (StaticBody, KinematicBody, DynamicBody)

### Key Functions

- `CreateWorld(def *WorldDef) WorldId` - Create a physics world
- `DestroyWorld(worldId WorldId)` - Destroy a world
- `(w WorldId) Step(timeStep float32, subStepCount int32)` - Step the simulation
- `(w WorldId) CreateBody(def *BodyDef) BodyId` - Create a body
- `DestroyBody(bodyId BodyId)` - Destroy a body
- `MakeBox(hx, hy float32) Polygon` - Create a box polygon
- `MakeCircle(center Vec2, radius float32) Circle` - Create a circle
- `(b BodyId) CreatePolygonShape(def *ShapeDef, polygon *Polygon) ShapeId` - Attach polygon to body
- `(b BodyId) CreateCircleShape(def *ShapeDef, circle *Circle) ShapeId` - Attach circle to body

## License

This Go binding is provided under the MIT license. Box2D itself is also MIT licensed.

## Credits

- Box2D physics engine by Erin Catto: https://box2d.org/
- Go bindings by the Advik-B/box2d-go contributors
