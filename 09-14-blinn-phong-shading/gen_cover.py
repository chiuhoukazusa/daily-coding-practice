from PIL import Image, ImageDraw, ImageFont

W, H = 1200, 675
img = Image.new('RGB', (W, H), (16, 20, 30))
d = ImageDraw.Draw(img)

try:
    f_title = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf", 58)
    f_sub = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans.ttf", 26)
    f_small = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans.ttf", 20)
except Exception:
    f_title = f_sub = f_small = ImageFont.load_default()

d.text((60, 48), "Blinn-Phong Shading Model", font=f_title, fill=(235, 240, 250))
d.text((60, 128), "ambient + diffuse(Lambert) + specular(half-vector)", font=f_sub, fill=(120, 185, 225))

# 嵌入渲染结果（三球）
try:
    render = Image.open('blinn_phong.ppm').convert('RGB')
    render = render.resize((760, 428))
    img.paste(render, (60, 185))
except Exception as e:
    print("render paste failed:", e)

# 右侧标注 shininess 关系
d.text((870, 200), "shininess", font=f_sub, fill=(200, 210, 220))
d.text((870, 245), "8     -> 宽高光", font=f_small, fill=(220, 150, 150))
d.text((870, 285), "64    -> 中", font=f_small, fill=(150, 220, 150))
d.text((870, 325), "256   -> 锐利", font=f_small, fill=(150, 170, 220))

d.text((60, 635), "half-vector H = normalize(L+V)  ·  specular = Ks·max(N·H,0)^shininess", font=f_small, fill=(150, 170, 190))
d.text((60, 664), "daily-coding-practice · 2026-09-14", font=f_sub, fill=(95, 115, 135))

img.save("cover.png")
print("cover.png written", img.size)
