import asyncio
import base64
import glob
import io
import os

import aiohttp
from PIL import Image


async def optimize_image(image_path, max_size=(448, 448), quality=85):
    """Optimize image by resizing and compressing before processing"""
    try:
        with Image.open(image_path) as img:
            # Convert to RGB if necessary
            if img.mode in ('RGBA', 'LA', 'P'):
                img = img.convert('RGB')
            
            # Resize if image is too large
            if img.size[0] > max_size[0] or img.size[1] > max_size[1]:
                img.thumbnail(max_size, Image.Resampling.LANCZOS)
            
            # Compress and convert to base64
            buffer = io.BytesIO()
            img.save(buffer, format='JPEG', quality=quality, optimize=True)
            buffer.seek(0)
            img_b64 = base64.b64encode(buffer.getvalue()).decode("utf-8")
            return img_b64
    except Exception as e:
        print(f"Error optimizing image {image_path}: {e}")
        return None


async def process_image_async(image_path,
                              max_size=(448, 448),
                              prompt="count pedestians and write just a number in brackets as answer"):
    """Process image asynchronously with optimized settings"""
    # Optimize the image
    print(f"Processing: {os.path.basename(image_path)}")
    img_b64 = await optimize_image(image_path, max_size=max_size)
    
    if img_b64 is None:
        print(f"Skipping {os.path.basename(image_path)} - could not decode")
        return None
    
    payload = {
        "model": "qwen2.5vl",
        "stream": False,
        "messages": [{
            "role": "user",
            "content": prompt,
            "images": [img_b64]
        }]
    }
    
    # Use async HTTP client for better performance
    async with aiohttp.ClientSession() as session:
        print(f"Sending request for {os.path.basename(image_path)}...")
        try:
            async with session.post(
                "http://localhost:11434/api/chat", 
                json=payload,
                timeout=aiohttp.ClientTimeout(total=60)
            ) as response:
                if response.status == 200:
                    data = await response.json()
                    message = data.get("message", {}).get("content", {})
                    if message:
                        return message
                    else:
                        return await response.text()
                else:
                    return f"Error: {response.status} - {await response.text()}"
        except Exception as e:
            print(f"Error processing {os.path.basename(image_path)}: {e}")
            return None


async def process_folder_async(folder_path, max_size, prompt="count pedestians and write just a number in brackets as answer"):
    """Process all images in a folder"""
    # Supported image extensions
    image_extensions = ['*.jpg', '*.jpeg', '*.png', '*.bmp', '*.tiff', '*.webp']
    
    # Get all image files in the folder
    image_files = []
    for ext in image_extensions:
        image_files.extend(glob.glob(os.path.join(folder_path, ext)))
        image_files.extend(glob.glob(os.path.join(folder_path, ext.upper())))
    
    if not image_files:
        print(f"No image files found in {folder_path}")
        return
    
    print(f"Found {len(image_files)} images to process")
    
    # Process images sequentially to avoid overwhelming the API
    results = {}
    for image_path in image_files:
        result = await process_image_async(image_path, max_size, prompt)
        if result is not None:
            results[os.path.basename(image_path)] = result
            print(f"✓ {os.path.basename(image_path)}: {result}")
        else:
            print(f"✗ {os.path.basename(image_path)}: Failed")
    
    return results


def main():
    # Folder path - you can change this
    folder_path = '/Users/andrybin/Pictures/tram'
    
    # Check if folder exists
    if not os.path.exists(folder_path):
        print(f"Error: Folder not found at {folder_path}")
        return
    
    prompt = "if you see a tram, estimate the distance to it in meters. write [distance] or [no tram]"
    max_size = (640, 480)
    print(f"Processing images from: {folder_path}")
    print(f"Prompt: {prompt}")
    print("-" * 50)
    
    # Run async function
    try:
        results = asyncio.run(process_folder_async(folder_path, max_size, prompt))
        if results:
            print("\n" + "=" * 50)
            print("SUMMARY:")
            for filename, result in results.items():
                print(f"{filename}: {result}")
    except Exception as e:
        print(f"Error processing folder: {e}")


if __name__ == "__main__":
    main()
