# Installation Test

This document describes how to verify that the library can be imported and used without any manual build steps.

## Test Steps

1. Create a new Go module in a fresh directory:
```bash
mkdir test-box2d && cd test-box2d
go mod init test-box2d
```

2. Create a simple test program:
```go
// main.go
package main

import (
    "fmt"
    box2d "github.com/Advik-B/box2d-go"
)

func main() {
    worldDef := box2d.DefaultWorldDef()
    worldDef.Gravity = box2d.Vec2{X: 0.0, Y: -10.0}
    worldId := box2d.CreateWorld(&worldDef)
    defer box2d.DestroyWorld(worldId)
    
    fmt.Println("Box2D initialized successfully!")
}
```

3. Add the dependency and run:
```bash
go get github.com/Advik-B/box2d-go
go run main.go
```

Expected output: `Box2D initialized successfully!`

## What Happens

When you run `go get` or `go build`, the Go toolchain:
1. Downloads the box2d-go module
2. CGO automatically compiles all the vendored Box2D C source files
3. Links them with your Go code
4. Creates the final binary

No manual steps required!
