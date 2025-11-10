# Box2D Go Library

Go bindings for the Box2D physics engine using CGO.

## Overview

This library provides Go bindings to Box2D v3.2.0, a 2D physics engine for games. The bindings use CGO to interface with the native C library, which is automatically compiled when you build your project.

## Features

- World creation and simulation
- Dynamic, static, and kinematic bodies
- Circle and polygon (box) shapes
- Physics stepping with configurable substeps
- Collision detection and response

## Installation

Simply use `go get`:

```bash
go get github.com/Advik-B/box2d-go
```

The Box2D C library is automatically compiled when you build your Go project.

### Prerequisites

- Go 1.16 or later
- CGO enabled (default)
- A C compiler (gcc, clang, msvc, etc.)

No need for CMake or manual build steps!

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

## Examples

The `examples/` directory contains sample programs demonstrating the library:

### Hello World Example

A simple falling box simulation:

```bash
go run examples/hello_world.go
```

### Stacking Example

A stack of boxes settling under gravity:

```bash
go run examples/stacking.go
```

## API Overview

This library provides comprehensive Go bindings for Box2D v3.2.0 with **218+ functions** covering all major physics engine features.

### Types

#### Core Types
- `WorldId` - Handle to a physics world
- `BodyId` - Handle to a rigid body
- `ShapeId` - Handle to a shape
- `JointId` - Handle to a joint
- `ChainId` - Handle to a chain shape
- `ContactId` - Handle to a contact

#### Data Types
- `Vec2` - 2D vector with X and Y components
- `Rot` - 2D rotation (cosine and sine)
- `Transform` - 2D transform (position and rotation)
- `AABB` - Axis-aligned bounding box
- `MassData` - Mass properties (mass, center, rotational inertia)
- `MotionLocks` - Restrict body movement on specific axes

#### Definition Types
- `WorldDef` - World definition with gravity and physics settings
- `BodyDef` - Body definition with type, position, and angle
- `ShapeDef` - Shape definition with density, friction, and restitution
- `Filter` - Collision filtering with category/mask bits
- `QueryFilter` - Query filtering for raycasts and overlap tests

#### Joint Definitions
- `DistanceJointDef` - Distance (spring) joint
- `RevoluteJointDef` - Revolute (hinge) joint
- `PrismaticJointDef` - Prismatic (slider) joint
- `WeldJointDef` - Weld joint
- `WheelJointDef` - Wheel/suspension joint
- `MotorJointDef` - Motor joint

#### Shape Types
- `Circle` - Circle shape
- `Polygon` - Convex polygon shape
- `Segment` - Line segment (edge) shape
- `Capsule` - Capsule shape

#### Enums
- `BodyType` - Body type (StaticBody, KinematicBody, DynamicBody)
- `JointType` - Joint type (DistanceJoint, RevoluteJoint, PrismaticJoint, WeldJoint, WheelJoint, MotorJoint)

### World Functions

#### Creation and Simulation
- `CreateWorld(def *WorldDef) WorldId` - Create a physics world
- `DestroyWorld(worldId WorldId)` - Destroy a world
- `(w WorldId) Step(timeStep float32, subStepCount int32)` - Step the simulation
- `(w WorldId) IsValid() bool` - Check if world ID is valid

#### Physics Settings
- `(w WorldId) SetGravity(gravity Vec2)` - Set world gravity
- `(w WorldId) GetGravity() Vec2` - Get world gravity
- `(w WorldId) SetRestitutionThreshold(value float32)` - Set restitution threshold
- `(w WorldId) GetRestitutionThreshold() float32` - Get restitution threshold
- `(w WorldId) SetHitEventThreshold(value float32)` - Set hit event threshold
- `(w WorldId) GetHitEventThreshold() float32` - Get hit event threshold
- `(w WorldId) SetContactTuning(hertz, dampingRatio, pushSpeed float32)` - Set contact tuning
- `(w WorldId) SetMaximumLinearSpeed(maximumLinearSpeed float32)` - Set max linear speed
- `(w WorldId) GetMaximumLinearSpeed() float32` - Get max linear speed

#### Optimization Features
- `(w WorldId) EnableSleeping(flag bool)` - Enable/disable sleeping
- `(w WorldId) IsSleepingEnabled() bool` - Check if sleeping is enabled
- `(w WorldId) EnableContinuous(flag bool)` - Enable/disable continuous collision
- `(w WorldId) IsContinuousEnabled() bool` - Check if continuous collision is enabled
- `(w WorldId) EnableWarmStarting(flag bool)` - Enable/disable warm starting
- `(w WorldId) IsWarmStartingEnabled() bool` - Check if warm starting is enabled
- `(w WorldId) EnableSpeculative(flag bool)` - Enable/disable speculative collision

#### Queries and Information
- `(w WorldId) CastRayClosest(origin, translation Vec2, filter QueryFilter) RayResult` - Cast a ray
- `(w WorldId) GetAwakeBodyCount() int32` - Get number of awake bodies
- `(w WorldId) GetProfile() Profile` - Get timing profile
- `(w WorldId) GetCounters() Counters` - Get simulation counters
- `(w WorldId) DumpMemoryStats()` - Dump memory statistics
- `(w WorldId) RebuildStaticTree()` - Rebuild static collision tree

### Body Functions

#### Creation and Destruction
- `(w WorldId) CreateBody(def *BodyDef) BodyId` - Create a body
- `DestroyBody(bodyId BodyId)` - Destroy a body
- `(b BodyId) IsValid() bool` - Check if body ID is valid

#### Properties
- `(b BodyId) GetType() BodyType` - Get body type
- `(b BodyId) SetType(bodyType BodyType)` - Set body type
- `(b BodyId) GetPosition() Vec2` - Get body position
- `(b BodyId) GetAngle() float32` - Get body angle
- `(b BodyId) GetRotation() Rot` - Get body rotation
- `(b BodyId) GetTransform() Transform` - Get body transform
- `(b BodyId) SetTransform(position Vec2, angle float32)` - Set body transform

#### Velocity
- `(b BodyId) GetLinearVelocity() Vec2` - Get linear velocity
- `(b BodyId) SetLinearVelocity(velocity Vec2)` - Set linear velocity
- `(b BodyId) GetAngularVelocity() float32` - Get angular velocity
- `(b BodyId) SetAngularVelocity(angularVelocity float32)` - Set angular velocity

#### Forces and Impulses
- `(b BodyId) ApplyForce(force, point Vec2, wake bool)` - Apply force at a point
- `(b BodyId) ApplyForceToCenter(force Vec2, wake bool)` - Apply force to center
- `(b BodyId) ApplyTorque(torque float32, wake bool)` - Apply torque
- `(b BodyId) ApplyLinearImpulse(impulse, point Vec2, wake bool)` - Apply linear impulse
- `(b BodyId) ApplyLinearImpulseToCenter(impulse Vec2, wake bool)` - Apply impulse to center
- `(b BodyId) ApplyAngularImpulse(impulse float32, wake bool)` - Apply angular impulse

#### Mass Properties
- `(b BodyId) GetMass() float32` - Get mass
- `(b BodyId) GetRotationalInertia() float32` - Get rotational inertia
- `(b BodyId) GetLocalCenterOfMass() Vec2` - Get local center of mass
- `(b BodyId) GetWorldCenterOfMass() Vec2` - Get world center of mass
- `(b BodyId) SetMassData(massData MassData)` - Set mass data
- `(b BodyId) GetMassData() MassData` - Get mass data
- `(b BodyId) ApplyMassFromShapes()` - Recompute mass from shapes

#### Damping and Gravity
- `(b BodyId) SetLinearDamping(linearDamping float32)` - Set linear damping
- `(b BodyId) GetLinearDamping() float32` - Get linear damping
- `(b BodyId) SetAngularDamping(angularDamping float32)` - Set angular damping
- `(b BodyId) GetAngularDamping() float32` - Get angular damping
- `(b BodyId) SetGravityScale(gravityScale float32)` - Set gravity scale
- `(b BodyId) GetGravityScale() float32` - Get gravity scale

#### Sleep and Activation
- `(b BodyId) IsAwake() bool` - Check if awake
- `(b BodyId) SetAwake(awake bool)` - Set awake status
- `(b BodyId) EnableSleep(enableSleep bool)` - Enable/disable sleep
- `(b BodyId) IsSleepEnabled() bool` - Check if sleep is enabled
- `(b BodyId) SetSleepThreshold(sleepThreshold float32)` - Set sleep threshold
- `(b BodyId) GetSleepThreshold() float32` - Get sleep threshold

#### State Control
- `(b BodyId) IsEnabled() bool` - Check if enabled
- `(b BodyId) Disable()` - Disable body
- `(b BodyId) Enable()` - Enable body
- `(b BodyId) SetBullet(flag bool)` - Set bullet flag (for CCD)
- `(b BodyId) IsBullet() bool` - Check bullet flag

#### Motion Constraints
- `(b BodyId) SetMotionLocks(locks MotionLocks)` - Set motion locks
- `(b BodyId) GetMotionLocks() MotionLocks` - Get motion locks

#### Coordinate Transformations
- `(b BodyId) GetLocalPoint(worldPoint Vec2) Vec2` - World to local point
- `(b BodyId) GetWorldPoint(localPoint Vec2) Vec2` - Local to world point
- `(b BodyId) GetLocalVector(worldVector Vec2) Vec2` - World to local vector
- `(b BodyId) GetWorldVector(localVector Vec2) Vec2` - Local to world vector

#### Queries
- `(b BodyId) GetContactCapacity() int32` - Get contact capacity
- `(b BodyId) GetJointCount() int32` - Get joint count
- `(b BodyId) GetShapeCount() int32` - Get shape count

### Shape Functions

#### Creation
- `MakeBox(hx, hy float32) Polygon` - Create a box polygon
- `MakeCircle(center Vec2, radius float32) Circle` - Create a circle
- `MakePolygon(vertices []Vec2, radius float32) Polygon` - Create polygon from vertices
- `MakeOffsetBox(hx, hy float32, center Vec2, angle float32) Polygon` - Create offset box
- `MakeSquare(h float32) Polygon` - Create a square
- `(b BodyId) CreatePolygonShape(def *ShapeDef, polygon *Polygon) ShapeId` - Attach polygon
- `(b BodyId) CreateCircleShape(def *ShapeDef, circle *Circle) ShapeId` - Attach circle
- `(b BodyId) CreateSegmentShape(def *ShapeDef, segment *Segment) ShapeId` - Attach segment
- `(b BodyId) CreateCapsuleShape(def *ShapeDef, capsule *Capsule) ShapeId` - Attach capsule
- `DestroyShape(shapeId ShapeId)` - Destroy a shape

#### Properties
- `(s ShapeId) IsValid() bool` - Check if shape ID is valid
- `(s ShapeId) GetBody() BodyId` - Get owning body
- `(s ShapeId) IsSensor() bool` - Check if sensor

#### Material Properties
- `(s ShapeId) SetDensity(density float32, updateBodyMass bool)` - Set density
- `(s ShapeId) GetDensity() float32` - Get density
- `(s ShapeId) SetFriction(friction float32)` - Set friction
- `(s ShapeId) GetFriction() float32` - Get friction
- `(s ShapeId) SetRestitution(restitution float32)` - Set restitution
- `(s ShapeId) GetRestitution() float32` - Get restitution

#### Collision Filtering
- `(s ShapeId) SetFilter(filter Filter)` - Set collision filter
- `(s ShapeId) GetFilter() Filter` - Get collision filter

#### Event Flags
- `(s ShapeId) EnableSensorEvents(flag bool)` - Enable/disable sensor events
- `(s ShapeId) AreSensorEventsEnabled() bool` - Check sensor events
- `(s ShapeId) EnableContactEvents(flag bool)` - Enable/disable contact events
- `(s ShapeId) AreContactEventsEnabled() bool` - Check contact events
- `(s ShapeId) EnablePreSolveEvents(flag bool)` - Enable/disable pre-solve events
- `(s ShapeId) ArePreSolveEventsEnabled() bool` - Check pre-solve events
- `(s ShapeId) EnableHitEvents(flag bool)` - Enable/disable hit events
- `(s ShapeId) AreHitEventsEnabled() bool` - Check hit events

#### Queries
- `(s ShapeId) GetAABB() AABB` - Get axis-aligned bounding box
- `(s ShapeId) GetClosestPoint(target Vec2) Vec2` - Get closest point
- `(s ShapeId) TestPoint(point Vec2) bool` - Test if point is inside
- `(s ShapeId) GetContactCapacity() int32` - Get contact capacity

### Joint Functions

#### Common Joint Operations
- `DestroyJoint(jointId JointId)` - Destroy a joint
- `(j JointId) IsValid() bool` - Check if joint ID is valid
- `(j JointId) GetType() JointType` - Get joint type
- `(j JointId) GetBodyA() BodyId` - Get body A
- `(j JointId) GetBodyB() BodyId` - Get body B
- `(j JointId) SetCollideConnected(shouldCollide bool)` - Set collision between connected bodies
- `(j JointId) GetCollideConnected() bool` - Get collision setting
- `(j JointId) WakeBodies()` - Wake connected bodies

#### Distance Joint
- `(j JointId) SetLength(length float32)` - Set length
- `(j JointId) GetLength() float32` - Get length
- `(j JointId) EnableSpring(enableSpring bool)` - Enable/disable spring
- `(j JointId) IsSpringEnabled() bool` - Check spring
- `(j JointId) SetSpringHertz(hertz float32)` - Set spring frequency
- `(j JointId) GetSpringHertz() float32` - Get spring frequency
- `(j JointId) SetSpringDampingRatio(dampingRatio float32)` - Set damping
- `(j JointId) GetSpringDampingRatio() float32` - Get damping
- `(j JointId) EnableLimit(enableLimit bool)` - Enable/disable limit
- `(j JointId) IsLimitEnabled() bool` - Check limit
- `(j JointId) SetLengthRange(minLength, maxLength float32)` - Set length limits
- `(j JointId) GetCurrentLength() float32` - Get current length
- `(j JointId) EnableMotor(enableMotor bool)` - Enable/disable motor
- `(j JointId) SetMotorSpeed(motorSpeed float32)` - Set motor speed
- `(j JointId) GetMotorForce() float32` - Get motor force

#### Revolute Joint (Hinge)
- `(j JointId) EnableRevoluteLimit(enableLimit bool)` - Enable/disable limit
- `(j JointId) GetRevoluteAngle() float32` - Get current angle
- `(j JointId) SetRevoluteLimits(lower, upper float32)` - Set angle limits
- `(j JointId) EnableRevoluteMotor(enableMotor bool)` - Enable/disable motor
- `(j JointId) SetRevoluteMotorSpeed(motorSpeed float32)` - Set motor speed
- `(j JointId) GetRevoluteMotorTorque() float32` - Get motor torque

#### Prismatic Joint (Slider)
- `(j JointId) EnablePrismaticLimit(enableLimit bool)` - Enable/disable limit
- `(j JointId) GetPrismaticTranslation() float32` - Get current translation
- `(j JointId) SetPrismaticLimits(lower, upper float32)` - Set translation limits
- `(j JointId) EnablePrismaticMotor(enableMotor bool)` - Enable/disable motor
- `(j JointId) SetPrismaticMotorSpeed(motorSpeed float32)` - Set motor speed
- `(j JointId) GetPrismaticMotorForce() float32` - Get motor force

#### Weld Joint
- `(j JointId) SetWeldLinearHertz(hertz float32)` - Set linear frequency
- `(j JointId) SetWeldAngularHertz(hertz float32)` - Set angular frequency
- `(j JointId) SetWeldLinearDampingRatio(dampingRatio float32)` - Set linear damping
- `(j JointId) SetWeldAngularDampingRatio(dampingRatio float32)` - Set angular damping

#### Wheel Joint
- `(j JointId) EnableWheelSpring(enableSpring bool)` - Enable/disable spring
- `(j JointId) SetWheelSpringHertz(hertz float32)` - Set spring frequency
- `(j JointId) EnableWheelLimit(enableLimit bool)` - Enable/disable limit
- `(j JointId) SetWheelLimits(lower, upper float32)` - Set limits
- `(j JointId) EnableWheelMotor(enableMotor bool)` - Enable/disable motor
- `(j JointId) SetWheelMotorSpeed(motorSpeed float32)` - Set motor speed

#### Motor Joint
- `(j JointId) SetMotorJointLinearVelocity(linearVelocity Vec2)` - Set linear velocity
- `(j JointId) SetMotorJointAngularVelocity(angularVelocity float32)` - Set angular velocity
- `(j JointId) SetMotorJointLinearHertz(hertz float32)` - Set linear frequency
- `(j JointId) SetMotorJointAngularHertz(hertz float32)` - Set angular frequency

### Math and Utility Functions

#### Vector Operations
- `Vec2Add(a, b Vec2) Vec2` - Add vectors
- `Vec2Sub(a, b Vec2) Vec2` - Subtract vectors
- `Vec2Length(v Vec2) float32` - Get vector length
- `Vec2Distance(a, b Vec2) float32` - Distance between points
- `Vec2Normalize(v Vec2) Vec2` - Normalize vector
- `Vec2Dot(a, b Vec2) float32` - Dot product
- `Vec2Cross(a, b Vec2) float32` - Cross product (scalar)

#### Rotation and Transform
- `MakeRot(angle float32) Rot` - Create rotation from angle
- `RotGetAngle(rot Rot) float32` - Get angle from rotation
- `TransformPoint(transform Transform, point Vec2) Vec2` - Transform a point

#### Contacts
- `(c ContactId) IsValid() bool` - Check if contact ID is valid

## License

This Go binding is provided under the MIT license. Box2D itself is also MIT licensed.

## Credits

- Box2D physics engine by Erin Catto: https://box2d.org/
- Go bindings by the Advik-B/box2d-go contributors
