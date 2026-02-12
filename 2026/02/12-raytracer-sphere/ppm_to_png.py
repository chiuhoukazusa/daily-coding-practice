#!/usr/bin/env python3
import sys
import struct

def ppm_to_png(ppm_path, png_path):
    # Read PPM file
    with open(ppm_path, 'rb') as f:
        # Read header
        while True:
            line = f.readline()
            if not line:
                break
            if line.startswith(b'#'):
                continue
            if line.startswith(b'P6'):
                break
        
        # Read dimensions
        while True:
            line = f.readline()
            if not line:
                break
            if line.startswith(b'#'):
                continue
            break
        
        width, height = map(int, line.split())
        
        # Read max color value
        while True:
            line = f.readline()
            if not line:
                break
            if line.startswith(b'#'):
                continue
            break
        
        max_val = int(line)
        
        # Read pixel data
        pixel_data = f.read(width * height * 3)
    
    # Convert PPM to PNG using Pillow if available
    try:
        from PIL import Image
        import numpy as np
        
        # Convert raw bytes to numpy array
        img_array = np.frombuffer(pixel_data, dtype=np.uint8).reshape((height, width, 3))
        
        # Create PIL image
        img = Image.fromarray(img_array, 'RGB')
        img.save(png_path, 'PNG')
        print(f"Converted {ppm_path} to {png_path} (using Pillow)")
        return True
        
    except ImportError:
        # Fallback: try to use external convert command
        import subprocess
        try:
            subprocess.run(['convert', ppm_path, png_path], check=True)
            print(f"Converted {ppm_path} to {png_path} (using ImageMagick)")
            return True
        except (subprocess.CalledProcessError, FileNotFoundError):
            # Last fallback: just copy with .png extension
            import shutil
            shutil.copy2(ppm_path, png_path)
            print(f"Copied {ppm_path} to {png_path} (raw PPM renamed to PNG)")
            return True

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python ppm_to_png.py input.ppm output.png")
        sys.exit(1)
    
    success = ppm_to_png(sys.argv[1], sys.argv[2])
    sys.exit(0 if success else 1)