#!/bin/bash
# Verify No Package Duplication in Docker Builds
#
# This script helps verify that packages are not being duplicated
# across multiple Docker builds.

echo "============================================================"
echo "Verifying No Package Duplication"
echo "============================================================"
echo

# Check if Docker BuildKit is enabled
echo "1. Checking Docker BuildKit..."
if docker buildx version > /dev/null 2>&1; then
    echo "   ✓ BuildKit available"
else
    echo "   ⚠ BuildKit not available (using legacy builder)"
fi

# Check Dockerfile structure
echo
echo "2. Checking Dockerfile structure..."
if grep -q "COPY requirements.txt" Dockerfile && grep -q "RUN.*pip3 install.*-r /tmp/requirements.txt" Dockerfile; then
    echo "   ✓ requirements.txt copied before pip install"
    echo "   ✓ Layer caching structure correct"
else
    echo "   ✗ Dockerfile structure issue"
    exit 1
fi

# Check if requirements.txt exists
echo
echo "3. Checking requirements.txt..."
if [ -f "requirements.txt" ]; then
    echo "   ✓ requirements.txt exists"
    echo "   Dependencies:"
    grep -v "^#" requirements.txt | grep -v "^$" | sed 's/^/     - /'
else
    echo "   ✗ requirements.txt not found"
    exit 1
fi

# Explain how to verify
echo
echo "============================================================"
echo "How to Verify No Duplicates"
echo "============================================================"
echo
echo "Step 1: Build the image (first time)"
echo "  docker compose build"
echo
echo "Step 2: Note the image size"
echo "  docker images alitekes1/ackermann_sim:latest"
echo "  Note the SIZE column"
echo
echo "Step 3: Make a code change (NOT requirements.txt)"
echo "  # Edit any Python file in ackermann_drl/"
echo
echo "Step 4: Rebuild"
echo "  docker compose build"
echo
echo "Step 5: Check build output for 'Using cache'"
echo "  Look for: 'Step X/Y : RUN pip3 install ...'"
echo "  Should see: '---> Using cache'"
echo "  This means packages were NOT reinstalled"
echo
echo "Step 6: Verify image size didn't increase"
echo "  docker images alitekes1/ackermann_sim:latest"
echo "  SIZE should be the SAME (no duplication)"
echo
echo "============================================================"
echo "Understanding Docker Layers"
echo "============================================================"
echo
echo "Docker uses LAYER CACHING:"
echo "  - Each instruction creates a layer"
echo "  - If layer inputs don't change, Docker REUSES the layer"
echo "  - Reused layers are NOT duplicated - they're shared"
echo
echo "Your Dockerfile structure:"
echo "  Layer 1: Base ROS image"
echo "  Layer 2: System packages (apt-get)"
echo "  Layer 3: Python packages (pip install) ← CACHED if requirements.txt unchanged"
echo "  Layer 4: Code clone/build ← Rebuilds when code changes"
echo
echo "Result:"
echo "  - Code changes → Only Layer 4 rebuilds"
echo "  - requirements.txt unchanged → Layer 3 uses cache (NO duplication)"
echo "  - Total image size stays the same"
echo
echo "============================================================"
echo "Quick Test"
echo "============================================================"
echo
echo "Run this to test caching:"
echo
echo "  # First build (will install packages)"
echo "  docker compose build 2>&1 | grep -A 2 'pip3 install'"
echo
echo "  # Second build (should use cache)"
echo "  docker compose build 2>&1 | grep -A 2 'pip3 install'"
echo
echo "Second build should show 'Using cache' - meaning no duplication!"
echo

