#!/usr/bin/env python3
"""Phase 1 import test - verify ackermann_drl package can be imported."""

import sys
import os
from pathlib import Path

# Add the package directory to Python path
# This allows importing before the package is installed via colcon
package_dir = Path(__file__).parent.parent / 'ackermann_drl'
if package_dir.exists():
    sys.path.insert(0, str(package_dir.parent))


def test_import_ackermann_drl():
    """Test that ackermann_drl package can be imported."""
    try:
        import ackermann_drl
        print("✓ Successfully imported ackermann_drl")
        return True
    except ImportError as e:
        print(f"✗ Failed to import ackermann_drl: {e}")
        return False


def test_import_envs():
    """Test that environment modules can be imported."""
    try:
        from ackermann_drl.envs import AckermannCityEnv
        print("✓ Successfully imported AckermannCityEnv")
        return True
    except ImportError as e:
        print(f"✗ Failed to import AckermannCityEnv: {e}")
        return False


def test_import_utils():
    """Test that utility modules can be imported."""
    try:
        from ackermann_drl.utils import RoadsGeometry, BatteryModel, ResetHelpers
        print("✓ Successfully imported utility modules")
        return True
    except ImportError as e:
        print(f"✗ Failed to import utility modules: {e}")
        return False


def main():
    """Run all import tests."""
    print("=" * 60)
    print("PHASE 1 IMPORT TEST")
    print("=" * 60)
    
    tests = [
        ("Import ackermann_drl", test_import_ackermann_drl),
        ("Import envs", test_import_envs),
        ("Import utils", test_import_utils),
    ]
    
    results = []
    for test_name, test_func in tests:
        print(f"\nTesting: {test_name}")
        result = test_func()
        results.append((test_name, result))
    
    print("\n" + "=" * 60)
    print("TEST RESULTS")
    print("=" * 60)
    
    all_passed = True
    for test_name, result in results:
        status = "PASS" if result else "FAIL"
        print(f"{test_name}: {status}")
        if not result:
            all_passed = False
    
    print("=" * 60)
    if all_passed:
        print("✓ ALL TESTS PASSED")
        return 0
    else:
        print("✗ SOME TESTS FAILED")
        return 1


if __name__ == '__main__':
    sys.exit(main())

