#!/usr/bin/env python3
"""Generate deterministic ribbed steel appearance and seamless PBR textures.

Only visual geometry is authored here. The station's cylinder remains the
sole collider and rigid body, with its original mass and friction material.
Requires only Python's standard library; the reference photo is not embedded.
"""
import argparse
import math
from pathlib import Path
import random
import struct
import zlib

RADIUS = .012
LENGTH = .6
PITCH = .012


def png(path, width, height, channels, pixels):
    def chunk(kind, data):
        return struct.pack('>I', len(data)) + kind + data + struct.pack(
            '>I', zlib.crc32(kind + data) & 0xffffffff)
    rows = b''.join(b'\0' + pixels[y*width*channels:(y+1)*width*channels]
                    for y in range(height))
    path.write_bytes(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack(
        '>IIBBBBB', width, height, 8, 2 if channels == 3 else 0, 0, 0, 0)) +
        chunk(b'IDAT', zlib.compress(rows, 9)) + chunk(b'IEND', b''))


def textures(directory):
    directory.mkdir(parents=True, exist_ok=True)
    rng = random.Random(133)
    grid = [[rng.random() for _ in range(65)] for _ in range(17)]
    # Tile around the circumference. The two cut ends need not match.
    grid[-1] = grid[0][:]
    width, height = 1024, 256
    color, roughness, metallic = bytearray(), bytearray(), bytearray()
    for y in range(height):
        gy = y / height * 16
        iy, fy = int(gy), gy % 1
        fy = fy*fy*(3-2*fy)
        for x in range(width):
            gx = x / (width-1) * 64
            ix, fx = min(int(gx), 63), gx-min(int(gx), 63)
            fx = fx*fx*(3-2*fx)
            n = ((1-fx)*grid[iy][ix] + fx*grid[iy][ix+1])*(1-fy) + (
                (1-fx)*grid[iy+1][ix] + fx*grid[iy+1][ix+1])*fy
            fleck = rng.random()
            rust = max(0., min(1., (n-.58)*5 + (fleck-.5)*.14))
            grain = (fleck-.5)*.085 + .025*math.sin(y*1.7+x*.03)
            # Weathered grey steel, with sparse warm brown oxidation.
            for steel, oxide in zip((.40, .43, .44), (.34, .18, .075)):
                color.append(round(255*max(0., min(1., steel*(1-rust)+oxide*rust+grain))))
            roughness.append(round(255*(.49+.34*rust+(fleck-.5)*.09)))
            metallic.append(round(255*(.78*(1-rust)+.06*rust)))
    png(directory/'rebar_basecolor.png', width, height, 3, color)
    png(directory/'rebar_roughness.png', width, height, 1, roughness)
    png(directory/'rebar_metallic.png', width, height, 1, metallic)


def radius_at(z, theta):
    # Opposing inclined ribs plus two longitudinal ribs. All visual peaks
    # stay within the nominal 24 mm collision cylinder's outer envelope.
    half = int(theta / math.pi)
    local = theta-half*math.pi
    axial = z + (local-math.pi/2)*.005 * (1 if half == 0 else -1)
    phase = (axial+PITCH/2) % PITCH-PITCH/2
    transverse = math.exp(-.5*(phase/.00095)**2)
    longitudinal = math.exp(-.5*(min(local, math.pi-local)/.055)**2)
    return .0112+.0008*max(transverse, longitudinal)


def generate(directory):
    textures(directory/'textures')
    rows, sides = 400, 64
    points, normals, uv, faces, counts = [], [], [], [], []
    for i in range(rows+1):
        z = -LENGTH/2+LENGTH*i/rows
        for j in range(sides+1):
            theta = 2*math.pi*j/sides
            r = radius_at(z, theta % (2*math.pi))
            dz = (radius_at(z+.00001, theta % (2*math.pi))-
                  radius_at(z-.00001, theta % (2*math.pi)))/.00002
            dt = (radius_at(z, (theta+.0001) % (2*math.pi))-
                  radius_at(z, (theta-.0001) % (2*math.pi)))/.0002/r
            c, s = math.cos(theta), math.sin(theta)
            n = (c+dt*s, s-dt*c, -dz)
            magnitude = math.sqrt(sum(v*v for v in n))
            points.append((r*c, r*s, z))
            normals.append(tuple(v/magnitude for v in n))
            uv.append((i/rows, j/sides))
    for i in range(rows):
        for j in range(sides):
            a = i*(sides+1)+j
            faces.extend((a, a+1, a+sides+2, a+sides+1))
            counts.append(4)
    # Separate end vertices provide flat cut faces rather than rounded tips.
    for end in (0, rows):
        base = len(points)
        for j in range(sides):
            source = end*(sides+1)+j
            points.append(points[source])
            normals.append((0, 0, -1 if end == 0 else 1))
            uv.append((.5+.45*math.cos(2*math.pi*j/sides),
                       .5+.45*math.sin(2*math.pi*j/sides)))
        faces.extend(range(base+sides-1, base-1, -1) if end == 0 else range(base, base+sides))
        counts.append(sides)
    def vectors(values, precision):
        return ', '.join('('+', '.join(f'{v:.{precision}f}' for v in value)+')' for value in values)
    lines = ['#usda 1.0', '(defaultPrim = "RebarVisual"\n metersPerUnit = 1\n upAxis = "Z")',
             'def Xform "RebarVisual" {', ' uniform token purpose = "default"',
             ' def Material "Steel" {',
             '  token outputs:surface.connect = </RebarVisual/Steel/Surface.outputs:surface>',
             '  def Shader "Surface" {', '   uniform token info:id = "UsdPreviewSurface"',
             '   color3f inputs:diffuseColor.connect = </RebarVisual/Steel/BaseColor.outputs:rgb>',
             '   float inputs:roughness.connect = </RebarVisual/Steel/Roughness.outputs:r>',
             '   float inputs:metallic.connect = </RebarVisual/Steel/Metallic.outputs:r>',
             '   float inputs:ior = 1.5', '   float inputs:opacity = 1', '   token outputs:surface', '  }',
             '  def Shader "UV" {', '   uniform token info:id = "UsdPrimvarReader_float2"',
             '   token inputs:varname = "st"', '   float2 outputs:result', '  }']
    for shader, filename, colorspace in (
        ('BaseColor', 'basecolor', 'sRGB'), ('Roughness', 'roughness', 'raw'),
        ('Metallic', 'metallic', 'raw')):
        lines.extend([f'  def Shader "{shader}" {{', '   uniform token info:id = "UsdUVTexture"',
                      f'   asset inputs:file = @./textures/rebar_{filename}.png@',
                      f'   token inputs:sourceColorSpace = "{colorspace}"',
                      '   token inputs:wrapS = "clamp"', '   token inputs:wrapT = "repeat"',
                      '   float2 inputs:st.connect = </RebarVisual/Steel/UV.outputs:result>',
                      '   float outputs:r', '   float3 outputs:rgb', '  }'])
    lines.extend([' }',
                  ' def Material "ColliderInvisible" {',
                  '  token outputs:surface.connect = </RebarVisual/ColliderInvisible/Surface.outputs:surface>',
                  '  def Shader "Surface" {',
                  '   uniform token info:id = "UsdPreviewSurface"',
                  '   float inputs:opacity = 0', '   float inputs:opacityThreshold = 0.5',
                  '   token outputs:surface', '  }', ' }', ' def Mesh "Surface" (prepend apiSchemas = ["MaterialBindingAPI"]) {',
                  '  uniform token purpose = "default"',
                  '  uniform token subdivisionScheme = "none"',
                  '  float3[] extent = [(-0.012, -0.012, -0.3), (0.012, 0.012, 0.3)]',
                  '  rel material:binding = </RebarVisual/Steel>',
                  f'  point3f[] points = [{vectors(points, 6)}]',
                  f'  normal3f[] normals = [{vectors(normals, 5)}] (interpolation = "vertex")',
                  f'  texCoord2f[] primvars:st = [{vectors(uv, 5)}] (interpolation = "vertex")',
                  '  int[] faceVertexCounts = ['+', '.join(map(str, counts))+']',
                  '  int[] faceVertexIndices = ['+', '.join(map(str, faces))+']',
                  ' }', '}', ''])
    output = directory/'rebar_visual.usda'
    output.write_text('\n'.join(lines))
    print(f'Generated {output}: {len(points)} vertices; PBR textures 1024 x 256')


def visual_reference(axis, radius=.012, length=.6, indent=12,
                     parent_path="/World/RebarStation/Rebar"):
    pad = ' '*indent
    rotation = '0, 90, 0' if axis == 'X' else '-90, 0, 0'
    return '\n'.join([
        f'{pad}rel material:binding = <{parent_path}/SteelVisual/ColliderInvisible>',
        f'{pad}def Xform "SteelVisual" (prepend references = @./rebar_visual.usda@) {{',
        f'{pad}    uniform token purpose = "default"',
        f'{pad}    float3 xformOp:rotateXYZ = ({rotation})',
        f'{pad}    double3 xformOp:scale = ({radius/RADIUS:g}, {radius/RADIUS:g}, {length/LENGTH:g})',
        f'{pad}    uniform token[] xformOpOrder = ["xformOp:rotateXYZ", "xformOp:scale"]',
        f'{pad}}}',
    ])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path,
                        default=Path(__file__).resolve().parents[1]/'urdf')
    generate(parser.parse_args().output_dir)
