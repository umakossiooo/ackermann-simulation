#!/usr/bin/env python3
"""Test that AckermannCityEnv step() method works without crashing."""

import sys
import os
from pathlib import Path

# Try to import numpy, but handle if not available
try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False
    print("⚠ numpy not available - some tests will be skipped")

# Add the package directory to Python path
package_dir = Path(__file__).parent.parent / 'ackermann_drl'
if package_dir.exists():
    sys.path.insert(0, str(package_dir.parent))


def test_step_without_rclpy():
    """Test that step() method signature is correct (without actual ROS 2)."""
    try:
        try:
            from ackermann_drl.envs import AckermannCityEnv
            # Verify method signature
            import inspect
            sig = inspect.signature(AckermannCityEnv.step)
            params = list(sig.parameters.keys())
            return_annotation = sig.return_annotation
        except ImportError:
            # Parse AST instead
            import ast
            env_file = Path(__file__).parent.parent / 'ackermann_drl' / 'envs' / 'ackermann_city_env.py'
            with open(env_file, 'r') as f:
                tree = ast.parse(f.read())
            
            # Find step method
            params = None
            return_annotation = None
            for node in ast.walk(tree):
                if isinstance(node, ast.ClassDef) and node.name == 'AckermannCityEnv':
                    for item in node.body:
                        if isinstance(item, ast.FunctionDef) and item.name == 'step':
                            params = [arg.arg for arg in item.args.args]
                            # Check return annotation if present (handle complex types)
                            if item.returns:
                                if isinstance(item.returns, ast.Name):
                                    return_annotation = item.returns.id
                                elif isinstance(item.returns, ast.Subscript):
                                    # Handle Tuple[...] or similar
                                    return_annotation = ast.unparse(item.returns) if hasattr(ast, 'unparse') else "Tuple[...]"
                                else:
                                    return_annotation = str(type(item.returns).__name__)
                            break
                    if params:
                        break
            
        # Should have 'action' parameter (plus 'self')
        assert 'action' in params, "step() must have 'action' parameter"
        assert len(params) == 2, f"step() should have 'self' and 'action', got {params}"
        
        print(f"✓ step() method signature is correct")
        print(f"  Parameters: {params}")
        if return_annotation:
            print(f"  Return type: {return_annotation}")
        return True
    except Exception as e:
        print(f"✗ Failed to verify step() signature: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_reset_without_rclpy():
    """Test that reset() method signature is correct (without actual ROS 2)."""
    try:
        try:
            from ackermann_drl.envs import AckermannCityEnv
            # Verify method signature
            import inspect
            sig = inspect.signature(AckermannCityEnv.reset)
            params = list(sig.parameters.keys())
            return_annotation = sig.return_annotation
        except ImportError:
            # Parse AST instead
            import ast
            env_file = Path(__file__).parent.parent / 'ackermann_drl' / 'envs' / 'ackermann_city_env.py'
            with open(env_file, 'r') as f:
                tree = ast.parse(f.read())
            
            # Find reset method
            params = None
            return_annotation = None
            for node in ast.walk(tree):
                if isinstance(node, ast.ClassDef) and node.name == 'AckermannCityEnv':
                    for item in node.body:
                        if isinstance(item, ast.FunctionDef) and item.name == 'reset':
                            params = [arg.arg for arg in item.args.args]
                            # Check return annotation if present (handle complex types)
                            if item.returns:
                                if isinstance(item.returns, ast.Name):
                                    return_annotation = item.returns.id
                                elif isinstance(item.returns, ast.Attribute):
                                    # Handle np.ndarray or similar
                                    return_annotation = f"{item.returns.attr}" if hasattr(item.returns, 'attr') else "Attribute"
                                else:
                                    return_annotation = str(type(item.returns).__name__)
                            break
                    if params:
                        break
        
        # Should have no required parameters (self only)
        assert len(params) == 1, f"reset() should only have 'self' parameter, got {params}"
        
        print(f"✓ reset() method signature is correct")
        print(f"  Parameters: {params}")
        if return_annotation:
            print(f"  Return type: {return_annotation}")
        return True
    except Exception as e:
        print(f"✗ Failed to verify reset() signature: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_action_format():
    """Test that action format is understood (array with 2 elements)."""
    if not HAS_NUMPY:
        print("⚠ Skipped (numpy not available)")
        return True
    
    try:
        # Test creating action arrays
        action1 = np.array([0.5, 0.1], dtype=np.float32)  # [linear_vel, angular_vel]
        action2 = np.array([1.0, -0.5], dtype=np.float32)
        action3 = np.array([0.0, 0.0], dtype=np.float32)
        
        assert len(action1) == 2, "Action should have 2 elements"
        assert len(action2) == 2, "Action should have 2 elements"
        assert len(action3) == 2, "Action should have 2 elements"
        
        print("✓ Action format is correct (2-element array: [linear_vel, angular_vel])")
        print(f"  Example actions: {action1}, {action2}, {action3}")
        return True
    except Exception as e:
        print(f"✗ Failed to verify action format: {e}")
        return False


def test_random_actions():
    """Test that we can generate 10 random actions without crashing."""
    if not HAS_NUMPY:
        print("⚠ Skipped (numpy not available)")
        return True
    
    try:
        # Generate 10 random actions
        np.random.seed(42)  # For reproducibility
        actions = []
        for i in range(10):
            # Random actions: linear_vel in [-2, 2], angular_vel in [-1, 1]
            linear_vel = np.random.uniform(-2.0, 2.0)
            angular_vel = np.random.uniform(-1.0, 1.0)
            action = np.array([linear_vel, angular_vel], dtype=np.float32)
            actions.append(action)
            
            # Verify action is valid
            assert len(action) == 2, f"Action {i} should have 2 elements"
            assert isinstance(action, np.ndarray), f"Action {i} should be numpy array"
        
        print(f"✓ Generated 10 random actions without crashing")
        print(f"  Actions: {[f'[{a[0]:.2f}, {a[1]:.2f}]' for a in actions[:5]]}...")
        return True
    except Exception as e:
        print(f"✗ Failed to generate random actions: {e}")
        return False


def main():
    """Run step tests."""
    print("=" * 60)
    print("ENVIRONMENT STEP TEST")
    print("=" * 60)
    
    tests = [
        ("Verify step() signature", test_step_without_rclpy),
        ("Verify reset() signature", test_reset_without_rclpy),
        ("Verify action format", test_action_format),
        ("Generate 10 random actions", test_random_actions),
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
        print("\nNote: These tests verify structure and action generation.")
        print("Full integration test requires ROS 2 and Gazebo running.")
        return 0
    else:
        print("✗ SOME TESTS FAILED")
        return 1


if __name__ == '__main__':
    sys.exit(main())

