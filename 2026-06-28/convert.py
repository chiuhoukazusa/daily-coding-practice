"""Convert PPM to PNG and create a side-by-side comparison."""
from PIL import Image
import numpy as np

def ppm_to_png(inpath, outpath):
    im = Image.open(inpath)
    im.save(outpath)
    return np.array(im)

# Load both
rest = ppm_to_png('skeletal_animation_rest.ppm', 'skeletal_animation_rest.png')
anim = ppm_to_png('skeletal_animation_anim.ppm', 'skeletal_animation_anim.png')

# Create side-by-side comparison
h, w = rest.shape[:2]
# Add a black bar separator
sep = 4
combined = np.zeros((h, w*2 + sep, 3), dtype=np.uint8)
combined[:, :w] = rest
combined[:, w+sep:] = anim
combined[:, w:w+sep] = [40, 40, 40]  # dark separator

im = Image.fromarray(combined)
im.save('skeletal_animation_comparison.png')

# Add labels (simple text at top)
from PIL import ImageDraw, ImageFont
draw = ImageDraw.Draw(im)
try:
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 20)
except:
    font = ImageFont.load_default()
draw.text((10, 10), "REST POSE", fill=(255,255,255), font=font)
draw.text((w + sep + 10, 10), "ANIMATED POSE", fill=(255,255,255), font=font)
im.save('skeletal_animation_comparison.png')

print(f"Rest: {rest.shape}, mean={rest.mean():.1f}")
print(f"Anim: {anim.shape}, mean={anim.mean():.1f}")
print(f"Comparison: {combined.shape}")
print("Done!")
