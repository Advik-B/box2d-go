# Merge Conflict Resolution Guide

## Conflicts when merging main into copilot/transform-to-go-library

When you merge the `main` branch into this branch, you will encounter the following conflicts:

### Conflicts:
1. **docs/images/mover.png** - deleted in this branch, modified in main (ImgBot optimization)
2. **docs/images/samples.png** - deleted in this branch, modified in main (ImgBot optimization)

### Resolution:
These files were deleted in commit `8575631` when we removed the Box2D source directories (since Box2D is now a git submodule). The main branch has an ImgBot commit that optimized these images.

**Recommended resolution:** Remove these files (keep the deletion from this branch)

```bash
git rm docs/images/mover.png docs/images/samples.png
```

This keeps our architecture where Box2D source (including docs/) is in the submodule, not duplicated in this repository.

### After resolving:
```bash
git commit
```

The merge will combine this branch's Go bindings with the main branch's image optimizations (which are no longer relevant since we're using a submodule).
