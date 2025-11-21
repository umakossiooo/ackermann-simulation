# Docker Build Optimization Guide

## Problem

Python packages (gymnasium, stable-baselines3, torch, etc.) were being reinstalled on every Docker build, even when they hadn't changed. This was slow and inefficient.

## Solution

The Dockerfile has been optimized to use Docker layer caching more effectively:

### Before (Inefficient)
```dockerfile
# Packages installed every time, even if code didn't change
RUN pip3 install --no-cache-dir --break-system-packages \
    gymnasium>=0.29.0 \
    stable-baselines3>=2.0.0 \
    torch>=2.0.0 \
    numpy>=1.24.0 \
    shapely>=2.0.0

# Code copied/cloned
RUN git clone ...
```

### After (Optimized)
```dockerfile
# Copy requirements.txt first (small file, changes rarely)
COPY requirements.txt /tmp/requirements.txt

# Install packages (this layer is cached unless requirements.txt changes)
RUN pip3 install --no-cache-dir --break-system-packages \
    -r /tmp/requirements.txt && \
    rm /tmp/requirements.txt

# Code copied/cloned (separate layer)
RUN git clone ...
```

## How It Works

1. **Layer Caching**: Docker caches each layer. If a layer's inputs haven't changed, Docker reuses the cached layer.

2. **requirements.txt**: By copying `requirements.txt` first and installing from it, the pip install layer only rebuilds when dependencies change.

3. **Separate Layers**: Code changes don't trigger package reinstallation.

## Benefits

- **Faster Builds**: Packages only reinstall when `requirements.txt` changes
- **Better Caching**: Docker can reuse the package installation layer
- **Easier Updates**: Update `requirements.txt` to change dependencies

## Usage

### Normal Build (uses cache)
```bash
docker compose build
# If requirements.txt hasn't changed, packages won't reinstall
```

### Force Rebuild (ignores cache)
```bash
docker compose build --no-cache
# Rebuilds everything from scratch
```

### Rebuild Only Packages Layer
```bash
# Modify requirements.txt, then:
docker compose build
# Only the package installation layer rebuilds
```

## Adding New Dependencies

1. Edit `requirements.txt`:
```txt
gymnasium>=0.29.0
stable-baselines3>=2.0.0
torch>=2.0.0
numpy>=1.24.0
shapely>=2.0.0
new-package>=1.0.0  # Add here
```

2. Rebuild:
```bash
docker compose build
# Only package layer rebuilds (faster)
```

## Verification

Check build output:
```
Step 4/7 : RUN pip3 install ...
 ---> Using cache  # <-- This means cache was used!
```

If you see "Using cache", the packages weren't reinstalled.

## Additional Optimization Tips

### 1. Use .dockerignore
Create `.dockerignore` to exclude unnecessary files:
```
.git
*.pyc
__pycache__
*.log
checkpoints/
logs/
```

### 2. Multi-stage Builds (Advanced)
For even better optimization, consider multi-stage builds (not implemented yet).

### 3. BuildKit Cache Mounts (Advanced)
Use BuildKit cache mounts for pip cache:
```dockerfile
RUN --mount=type=cache,target=/root/.cache/pip \
    pip3 install --no-cache-dir --break-system-packages \
    -r /tmp/requirements.txt
```

## Current Status

✅ Dockerfile optimized with requirements.txt
✅ Layer caching enabled
✅ Packages only reinstall when dependencies change
✅ Faster rebuilds for code changes

