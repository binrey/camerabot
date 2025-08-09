#!/usr/bin/env python3
"""
Test script to verify OpenAI API setup
"""

import os
import openai
import base64
import cv2
import numpy as np

def test_openai_setup():
    """Test if OpenAI API is properly configured"""
    
    # Check if API key is set
    api_key = os.getenv('OPENAI_API_KEY')
    if not api_key:
        print("❌ OPENAI_API_KEY environment variable not set")
        print("Please set it with: export OPENAI_API_KEY='your-api-key-here'")
        return False
    
    print("✅ OPENAI_API_KEY is set")
    
    # Test OpenAI client initialization
    try:
        client = openai.OpenAI()
        print("✅ OpenAI client initialized successfully")
    except Exception as e:
        print(f"❌ Failed to initialize OpenAI client: {e}")
        return False
    
    # Test with a simple text request
    try:
        response = client.chat.completions.create(
            model="gpt-4o",
            messages=[
                {"role": "user", "content": "write completed version of equation in plain text: 111 + 222 = ?"}
            ],
            max_tokens=50
        )
        print("✅ OpenAI API test successful")
        print(f"Response: {response.choices[0].message.content}")
        return True
    except Exception as e:
        print(f"❌ OpenAI API test failed: {e}")
        return False

def test_image_processing():
    """Test image processing capabilities"""
    
    try:
        # Create a test image
        test_image = np.zeros((480, 640, 3), dtype=np.uint8)
        test_image[:] = (255, 0, 0)  # Red image
        
        # Encode to base64
        _, buffer = cv2.imencode('.jpg', test_image)
        image_base64 = base64.b64encode(buffer).decode("utf-8")
        
        print("✅ Image processing test successful")
        print(f"Base64 image length: {len(image_base64)} characters")
        return True
    except Exception as e:
        print(f"❌ Image processing test failed: {e}")
        return False

if __name__ == "__main__":
    print("Testing OpenAI API setup...")
    print("=" * 40)
    
    openai_ok = test_openai_setup()
    image_ok = test_image_processing()
    
    print("=" * 40)
    if openai_ok and image_ok:
        print("✅ All tests passed! You're ready to run the robot.")
    else:
        print("❌ Some tests failed. Please fix the issues above.") 