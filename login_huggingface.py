#!/usr/bin/env python3
"""
Login to HuggingFace to access SAM3 model
"""

from huggingface_hub import login
import sys

print("=" * 60)
print("  HuggingFace Login for SAM3 Access")
print("=" * 60)
print()
print("You need a HuggingFace access token to download SAM3.")
print()
print("Steps:")
print("1. Go to: https://huggingface.co/settings/tokens")
print("2. Create a new token (or use existing)")
print("3. Make sure you have access to: https://huggingface.co/facebook/sam3")
print("4. Paste your token below")
print()
print("-" * 60)
print()

try:
    # This will prompt for token
    login()
    
    print()
    print("=" * 60)
    print("✅ Successfully logged in to HuggingFace!")
    print("=" * 60)
    print()
    print("You can now use SAM3. Test it with:")
    print("  ./test_cv_pipeline_simple.sh")
    print()
    
except KeyboardInterrupt:
    print("\n\nLogin cancelled.")
    sys.exit(1)
except Exception as e:
    print(f"\n❌ Error during login: {e}")
    print()
    print("Alternative: Set token as environment variable:")
    print("  export HF_TOKEN='your_token_here'")
    sys.exit(1)
