#!/usr/bin/env python3
"""
Test script for CV Model Manager
"""

import sys
sys.path.insert(0, 'ros2_ws/src/cv_pipeline/python')

from cv_model_manager import ModelManager
import numpy as np

def main():
    print("=" * 50)
    print("  CV Model Manager Test")
    print("=" * 50)
    
    # Create manager
    manager = ModelManager(device="cuda")
    
    # List models
    print("\n1. Available Models:")
    models = manager.list_models()
    for model in models:
        print(f"   - {model}")
    
    # Get model info
    print("\n2. Model Information:")
    for model_name in models:
        info = manager.get_model_info(model_name)
        print(f"\n   {model_name}:")
        print(f"      Loaded: {info['loaded']}")
        print(f"      Device: {info['device']}")
        print(f"      Modes: {', '.join(info['supported_modes'])}")
    
    # Test SAM2 if available
    if "sam2" in models:
        print("\n3. Testing SAM2:")
        print("   Loading model...")
        success = manager.load_model("sam2")
        
        if success:
            print("   ✅ Model loaded successfully!")
            
            # Create dummy image
            print("   Creating test image (960x540)...")
            test_image = np.random.randint(0, 255, (540, 960, 3), dtype=np.uint8)
            
            # Test different modes
            modes = [
                {"prompt_type": "point"},
                {"prompt_type": "point", "x": "480", "y": "270"},
                {"prompt_type": "box", "box": "200,150,700,450"},
            ]
            
            for i, params in enumerate(modes, 1):
                print(f"\n   Test {i}: {params}")
                result = manager.process("sam2", test_image, params)
                
                if "error" in result:
                    print(f"      ❌ Error: {result['error']}")
                else:
                    print(f"      ✅ Success!")
                    print(f"         Processing time: {result['processing_time']:.3f}s")
                    print(f"         Num masks: {result['num_masks']}")
                    print(f"         Scores: {[f'{s:.3f}' for s in result['scores']]}")
        else:
            print("   ❌ Failed to load model")
    
    print("\n" + "=" * 50)
    print("  Test Complete!")
    print("=" * 50)

if __name__ == "__main__":
    main()
