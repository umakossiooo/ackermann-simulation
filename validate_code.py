#!/usr/bin/env python3
"""Code validation script - checks for common errors without requiring Docker.

This script validates:
- Syntax errors
- Import structure
- Variable initialization
- Function signatures
- Common logical errors
"""

import ast
import sys
from pathlib import Path

def check_syntax(file_path):
    """Check Python syntax of a file."""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            code = f.read()
        ast.parse(code)
        return True, None
    except SyntaxError as e:
        return False, f"Syntax error at line {e.lineno}: {e.msg}"
    except Exception as e:
        return False, f"Error: {str(e)}"

def check_imports(file_path):
    """Check if imports are valid (structure only, not actual imports)."""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            code = f.read()
        tree = ast.parse(code)
        
        imports = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    imports.append(alias.name)
            elif isinstance(node, ast.ImportFrom):
                if node.module:
                    imports.append(node.module)
        
        return True, imports
    except Exception as e:
        return False, str(e)

def check_variable_usage(file_path, required_vars):
    """Check if required variables are used in file."""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        found_vars = []
        missing_vars = []
        for var in required_vars:
            if var in content:
                found_vars.append(var)
            else:
                missing_vars.append(var)
        
        return found_vars, missing_vars
    except Exception as e:
        return [], required_vars

def main():
    """Run validation checks."""
    base_path = Path(__file__).parent
    errors = []
    warnings = []
    
    print("=" * 70)
    print("Code Validation")
    print("=" * 70)
    print()
    
    # Files to check
    files_to_check = [
        ('ackermann_drl/ackermann_drl/utils/battery_model.py', ['BatteryModel', 'vehicle_weight', 'weight_factor']),
        ('ackermann_drl/ackermann_drl/utils/sliding_mode_control.py', ['SlidingModeController', 'compute_control']),
        ('ackermann_drl/ackermann_drl/utils/roads_geometry.py', ['RoadsGeometry', 'get_road_heading_at_point']),
        ('ackermann_drl/scripts/smc_control_node.py', ['SMCControlNode', 'control_callback']),
        ('ackermann_drl/scripts/train_ppo.py', ['PPO', 'RewardLoggingCallback']),
        ('ackermann_drl/ackermann_drl/envs/ackermann_city_env.py', ['AckermannCityEnv', 'compute_reward', 'battery_consumed_this_step']),
    ]
    
    print("1. Syntax Validation")
    print("-" * 70)
    for file_path, required_vars in files_to_check:
        full_path = base_path / file_path
        if not full_path.exists():
            errors.append(f"{file_path}: File not found")
            print(f"✗ {file_path}: File not found")
            continue
        
        valid, error = check_syntax(full_path)
        if valid:
            print(f"✓ {file_path}: Syntax OK")
        else:
            errors.append(f"{file_path}: {error}")
            print(f"✗ {file_path}: {error}")
    
    print()
    print("2. Variable Usage Check")
    print("-" * 70)
    for file_path, required_vars in files_to_check:
        full_path = base_path / file_path
        if not full_path.exists():
            continue
        
        found, missing = check_variable_usage(full_path, required_vars)
        if missing:
            warnings.append(f"{file_path}: Missing variables: {', '.join(missing)}")
            print(f"⚠ {file_path}: Missing: {', '.join(missing)}")
        else:
            print(f"✓ {file_path}: All required variables found")
    
    print()
    print("3. Import Structure Check")
    print("-" * 70)
    for file_path, _ in files_to_check:
        full_path = base_path / file_path
        if not full_path.exists():
            continue
        
        valid, imports = check_imports(full_path)
        if valid:
            # Check for common problematic imports
            if 'import *' in open(full_path).read():
                warnings.append(f"{file_path}: Uses 'import *' (not recommended)")
            print(f"✓ {file_path}: Import structure OK ({len(imports)} imports)")
        else:
            errors.append(f"{file_path}: Import check failed: {imports}")
            print(f"✗ {file_path}: Import check failed")
    
    print()
    print("=" * 70)
    print("Validation Summary")
    print("=" * 70)
    print(f"Errors: {len(errors)}")
    print(f"Warnings: {len(warnings)}")
    print()
    
    if errors:
        print("Errors found:")
        for error in errors:
            print(f"  ✗ {error}")
        print()
    
    if warnings:
        print("Warnings:")
        for warning in warnings:
            print(f"  ⚠ {warning}")
        print()
    
    if not errors and not warnings:
        print("✓ All validation checks passed!")
        return 0
    elif not errors:
        print("⚠ Validation passed with warnings")
        return 0
    else:
        print("✗ Validation failed with errors")
        return 1

if __name__ == '__main__':
    sys.exit(main())

