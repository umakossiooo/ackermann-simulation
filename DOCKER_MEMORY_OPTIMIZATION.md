# Docker Memory/Disk Optimization

## Problem

Packages were being reinstalled on every build, potentially duplicating and wasting disk space/memory in the container.

## Solution: Layer Caching + BuildKit Cache Mounts

### How Docker Layer Caching Works

Docker uses **layer caching** - each instruction creates a layer. If a layer's inputs haven't changed, Docker **reuses the cached layer** instead of rebuilding it. This means:

- ✅ **No duplication**: Cached layers are shared, not duplicated
- ✅ **Faster builds**: Reuses existing layers
- ✅ **Less disk usage**: Same layer = same disk space (shared)

### Current Optimization

1. **requirements.txt**: Packages only reinstall if this file changes
2. **BuildKit cache mount**: Pip cache is preserved between builds
3. **Layer ordering**: Dependencies installed before code (code changes don't trigger package reinstall)

## Understanding Docker Layers

```
Layer 1: Base ROS image
Layer 2: System packages (apt-get)
Layer 3: Python packages (pip install) ← CACHED if requirements.txt unchanged
Layer 4: Code clone/build ← Rebuilds when code changes
```

**Key Point**: If Layer 3 is cached, it's **NOT duplicated** - Docker reuses the exact same layer.

## Memory/Disk Usage

### Without Cache (Bad)
```
Build 1: Install packages → 2GB
Build 2: Install packages → 2GB (duplicate!)
Total: 4GB ❌
```

### With Cache (Good)
```
Build 1: Install packages → 2GB
Build 2: Use cached layer → 0GB (reused!)
Total: 2GB ✅
```

## Verifying Cache is Working

### Check Build Output
```bash
docker compose build
```

Look for:
```
Step 4/7 : RUN pip3 install ...
 ---> Using cache  # ✅ Cache used, no duplication!
```

If you see `---> Running in ...` instead, the cache was invalidated.

### Check Image Layers
```bash
docker history alitekes1/ackermann_sim:latest
```

You'll see which layers are cached (same size = reused).

## Ensuring Cache Works

### 1. Don't Use --no-cache
```bash
# ❌ BAD - Forces rebuild, wastes space
docker compose build --no-cache

# ✅ GOOD - Uses cache
docker compose build
```

### 2. Keep requirements.txt Stable
- Only change `requirements.txt` when you need new packages
- Code changes won't trigger package reinstall

### 3. Use BuildKit (Automatic)
BuildKit is enabled by default in newer Docker versions. It provides:
- Better caching
- Cache mounts (pip cache preserved)
- Parallel builds

## BuildKit Cache Mount

The Dockerfile now uses:
```dockerfile
RUN --mount=type=cache,target=/root/.cache/pip \
    pip3 install ...
```

This:
- ✅ Preserves pip's download cache between builds
- ✅ Faster installs (doesn't re-download if already cached)
- ✅ Less network usage
- ✅ Cache stored outside container (doesn't bloat image)

## Disk Space Management

### Check Image Size
```bash
docker images alitekes1/ackermann_sim
```

### Clean Up Unused Images
```bash
# Remove unused images (keeps cache)
docker image prune

# Remove all unused data (including cache)
docker system prune -a
```

### Check Layer Sizes
```bash
docker history --human alitekes1/ackermann_sim:latest
```

## Best Practices

1. **Always use cache** (don't use `--no-cache` unless necessary)
2. **Keep requirements.txt stable** (only change when needed)
3. **Use BuildKit** (enabled by default)
4. **Monitor cache hits** (check build output)
5. **Clean up periodically** (remove unused images)

## Troubleshooting

### Packages Still Reinstalling?

**Check:**
1. Is `requirements.txt` being modified?
2. Are you using `--no-cache`?
3. Is Docker BuildKit enabled? (`DOCKER_BUILDKIT=1`)

**Solution:**
```bash
# Ensure BuildKit is enabled
export DOCKER_BUILDKIT=1
docker compose build
```

### Cache Not Working?

**Check build context:**
```bash
# See what's being sent to Docker
docker build --progress=plain .
```

**Solution:**
- Ensure `.dockerignore` is working
- Check if `requirements.txt` is in build context
- Verify Docker BuildKit is enabled

## Summary

✅ **No duplication**: Docker layers are shared, not duplicated
✅ **Cache preserved**: Packages only reinstall if `requirements.txt` changes
✅ **Memory efficient**: Cached layers use same disk space (shared)
✅ **BuildKit cache**: Pip cache preserved between builds

**Result**: Packages install once, then reuse cached layer on subsequent builds. No memory/disk waste!

