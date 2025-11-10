# Box2D Go Bindings

This repository provides Go bindings for the [Box2D](https://github.com/erincatto/box2d) physics engine using CGO.

## Structure

- `box2d/` - Git submodule pointing to the upstream Box2D C library (for reference)
- `go-box2d/` - Go bindings and wrapper library with vendored Box2D C source

## Box2D

Box2D is a 2D physics engine for games developed by Erin Catto.

[![Box2D Version 3.0 Release Demo](https://img.youtube.com/vi/dAoM-xjOWtA/0.jpg)](https://www.youtube.com/watch?v=dAoM-xjOWtA)

### Features

#### Collision
- Continuous collision detection
- Contact events
- Convex polygons, capsules, circles, rounded polygons, segments, and chains
- Multiple shapes per body
- Collision filtering
- Ray casts, shape casts, and overlap queries
- Sensor system

#### Physics
- Robust _Soft Step_ rigid body solver
- Continuous physics for fast translations and rotations
- Island based sleep
- Revolute, prismatic, distance, mouse joint, weld, and wheel joints
- Joint limits, motors, springs, and friction
- Joint and contact forces
- Body movement events and sleep notification

#### System
- Data-oriented design
- Written in portable C17
- Extensive multithreading and SIMD
- Optimized for large piles of bodies

## Go Library

This repository provides Go bindings for Box2D using CGO. The Go library is located in the `go-box2d/` directory.

### Installation

Simply import the library and build your project:

```bash
go get github.com/Advik-B/box2d-go
```

That's it! The Box2D C library is automatically compiled when you build your Go project.

### Prerequisites

- Go 1.16 or later
- CGO enabled (default)
- A C compiler (gcc, clang, msvc, etc.)

### Quick Example

```go
import box2d "github.com/Advik-B/box2d-go"

worldDef := box2d.DefaultWorldDef()
worldDef.Gravity = box2d.Vec2{X: 0.0, Y: -10.0}
worldId := box2d.CreateWorld(&worldDef)
defer box2d.DestroyWorld(worldId)

// Create bodies and shapes...
worldId.Step(1.0/60.0, 4)
```

See [go-box2d/README.md](go-box2d/README.md) for detailed documentation and examples.

## Documentation

- [Box2D Manual](https://box2d.org/documentation/)
- [Box2D Migration Guide](https://github.com/erincatto/box2d/blob/main/docs/migration.md)
- [Go Bindings Documentation](go-box2d/README.md)

## Community

- [Discord](https://discord.gg/NKYgCBP)
- [GitHub Discussions](https://github.com/erincatto/box2d/discussions)

## License

Box2D is developed by Erin Catto and uses the [MIT license](https://en.wikipedia.org/wiki/MIT_License).

The Go bindings are also provided under the MIT license.

## Credits

- Box2D physics engine: [Erin Catto](https://github.com/erincatto)
- Go bindings: Contributors to this repository
