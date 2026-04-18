import struct, zlib, sys

def write_png_from_ppm(ppm_path, png_path):
    def chunk(name, data):
        c = struct.pack('>I', len(data)) + name + data
        return c + struct.pack('>I', zlib.crc32(c[4:]) & 0xffffffff)

    with open(ppm_path, 'rb') as f:
        f.readline()
        dims = f.readline().split()
        f.readline()
        w, h = int(dims[0]), int(dims[1])
        px = f.read()

    raw = b''.join(b'\x00' + px[y*w*3:(y+1)*w*3] for y in range(h))

    with open(png_path, 'wb') as out:
        out.write(b'\x89PNG\r\n\x1a\n')
        out.write(chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0)))
        out.write(chunk(b'IDAT', zlib.compress(raw, 6)))
        out.write(chunk(b'IEND', b''))

    print(f'PNG written OK: {png_path}')

if __name__ == '__main__':
    write_png_from_ppm(sys.argv[1], sys.argv[2])
