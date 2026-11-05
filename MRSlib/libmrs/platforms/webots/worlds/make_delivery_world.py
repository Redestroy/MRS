#!/usr/bin/env python3
"""Writes the Webots delivery world (spec 18 §9) from the files of `mrs_generate delivery`.

Usage: make_delivery_world.py <manifest.csv> <sources.csv> <zones.csv> <timeline.mrsl> <out.wbt>
                              [--robots N] [--pad X,Y] [--low R:FRACTION ...]

Every package of the manifest becomes a box in its source's pile (bottom first, as in the
manifest), with its tag in the Solid's description ("tag <id> <dest x y z> <area> <mass>") and a
passive loop Connector on top. Each Mavic gets the sling in its body slot (a line on a ball joint,
a load cell on a sliding spring, and the hook Connector), supervisor TRUE for the tag reader, and
mrs_uav with --zones and --pad. A Charger stands at the pad. The mission header is written next to
the world as <out>_mission.mrs.
"""
import argparse
import csv
import os

SLING_LENGTH = 2.0  # m, body to hook, as in mavic_sling_webots.mrsd / the generator's sling_length
PACKAGE = (0.3, 0.3, 0.25)  # m
LOOP = 0.10  # m above the package's top
COM_X = -0.075  # m: the Mavic's centre of mass (body and propellers, Mavic2Pro.proto); the line hangs below it
GEO = (56.9496, 24.1052, 10)


def points(path):
    out = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            v = [float(x) for x in line.split(',')]
            out.append((v[0], v[1], v[2] if len(v) > 2 else 0.0))
    return out


def sling():
    half = SLING_LENGTH / 2
    return f'''    BallJoint {{
      jointParameters BallJointParameters {{
        anchor {COM_X} 0 -0.06
        dampingConstant 0.2
      }}
      endPoint Solid {{
        translation {COM_X} 0 {-0.06 - half:.3f}
        name "line"
        children [
          Shape {{
            appearance PBRAppearance {{
              baseColor 0.1 0.1 0.1
              roughness 1
              metalness 0
            }}
            geometry Cylinder {{
              height {SLING_LENGTH - 0.1:.3f}
              radius 0.004
            }}
          }}
          SliderJoint {{
            jointParameters JointParameters {{
              axis 0 0 1
              springConstant 150
              dampingConstant 4
              minStop -0.2
              maxStop 0.05
            }}
            device [
              PositionSensor {{
                name "load cell"
              }}
            ]
            endPoint Solid {{
              translation 0 0 {-half:.3f}
              name "hook body"
              children [
                Shape {{
                  appearance PBRAppearance {{
                    baseColor 0.8 0.6 0.1
                    metalness 0.5
                  }}
                  geometry Sphere {{
                    radius 0.03
                  }}
                }}
                Connector {{
                  rotation 0 1 0 1.5708
                  name "hook"
                  model "sling"
                  type "active"
                  autoLock FALSE
                  distanceTolerance 0.35
                  axisTolerance 1.0
                  rotationTolerance 0
                  numberOfRotations 0
                  snap FALSE
                }}
              ]
              physics Physics {{
                density -1
                mass 0.1
              }}
            }}
          }}
        ]
        physics Physics {{
          density -1
          mass 0.02
        }}
      }}
    }}'''


def mavic(rid, home, args, energy):
    arg_lines = '\n'.join(f'    "{a}"' for a in [str(rid), 'rta', 'mavic_sling_webots.mrsd', 'mavic_sling_webots.mrsp', 'uav_behaviours.mrsb'] + args)
    return f'''Mavic2Pro {{
  translation {home[0]} {home[1]} 0.1
  name "r{rid}"
  controller "mrs_uav"
  controllerArgs [
{arg_lines}
  ]
  supervisor TRUE
  battery [
    {energy:.0f}, 180000, 3000
  ]
  bodySlot [
    Emitter {{
      name "emitter"
      channel 1
    }}
    Receiver {{
      name "receiver"
      channel 1
    }}
{sling()}
  ]
}}
'''


def package(row, base, k):
    w, d, h = PACKAGE
    z = base[2] + h / 2 + k * h
    tag = f"tag {row['package']} {row['dest_x']} {row['dest_y']} {row['dest_z']} {row['area']} {row['mass']}"
    return f'''DEF PKG_{row['package']} Solid {{
  translation {base[0]} {base[1]} {z:.3f}
  name "package {row['package']}"
  description "{tag}"
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.72 0.53 0.32
        roughness 1
        metalness 0
      }}
      geometry DEF BOX Box {{
        size {w} {d} {h}
      }}
    }}
    Connector {{
      translation 0 0 {h / 2 + LOOP:.3f}
      rotation 0 1 0 -1.5708
      name "loop"
      model "sling"
      type "passive"
      distanceTolerance 0.35
      axisTolerance 1.0
      rotationTolerance 0
      numberOfRotations 0
      snap FALSE
    }}
  ]
  boundingObject USE BOX
  physics Physics {{
    density -1
    mass {row['mass']}
  }}
}}
'''


def marker(x, y, colour, size, name):
    return f'''Solid {{
  translation {x} {y} 0.005
  name "{name}"
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor {colour}
        roughness 1
        metalness 0
      }}
      geometry Cylinder {{
        height 0.01
        radius {size}
      }}
    }}
  ]
}}
'''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('manifest')
    ap.add_argument('sources')
    ap.add_argument('zones')
    ap.add_argument('timeline')
    ap.add_argument('out')
    ap.add_argument('--robots', type=int, default=3)
    ap.add_argument('--pad', default='10,-8')
    ap.add_argument('--low', action='append', default=[], help='R:FRACTION, a robot that starts with that much battery')
    a = ap.parse_args()

    sources = points(a.sources)
    with open(a.manifest) as f:
        manifest = list(csv.DictReader(f))
    with open(a.zones) as f:
        zones = list(csv.DictReader(f))
    pad = tuple(float(v) for v in a.pad.split(','))
    low = {int(k): float(v) for k, v in (x.split(':') for x in a.low)}
    homes = [(4.0 * (k - (a.robots - 1) / 2), -8.0, 0.0) for k in range(a.robots)]
    base = os.path.splitext(a.out)[0]
    out_dir = os.path.dirname(os.path.abspath(a.out))
    rel = lambda p: '../../worlds/' + os.path.relpath(os.path.abspath(p), out_dir).replace(os.sep, '/')

    mission = base + '_mission.mrs'
    with open(mission, 'w') as f:
        robots = ' '.join(f'{k + 1} {h[0]:g} {h[1]:g} 0' for k, h in enumerate(homes))
        f.write('# Mission header for %s (spec 06 §6), written by make_delivery_world.py: fence ±100 m,\n'
                '# 60 m high, layers from 20 m every 5 m, claim grace 120 s, homes on y = -8.\n@: MRS 0.1/\n\n' % os.path.basename(a.out))
        f.write(f'H: H_M m1 {GEO[0]} {GEO[1]} {GEO[2]} -100 100 -100 100 60 20 5 120 {a.robots} {robots}/\n')

    parts = ['''#VRML_SIM R2025a utf8
# The package delivery world (spec 18 §9), written by make_delivery_world.py. Mavic 2 Pro with a
# sling deliver the packages of the piles to the destinations on their tags; a charging pad
# stands beside the homes. See README.md.

EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackground.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/floors/protos/Floor.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/robots/dji/mavic/protos/Mavic2Pro.proto"

WorldInfo {
  info [
    "MRS package delivery: Mavics with slings, piles, destinations, a charging pad"
  ]
  title "mavic_delivery"
  basicTimeStep 8
  defaultDamping Damping {
    linear 0.5
    angular 0.5
  }
  gpsCoordinateSystem "WGS84"
  gpsReference %g %g %g
}
Viewpoint {
  orientation -0.3 0.3 0.9 1.7
  position 5 -55 40
  follow "r1"
}
TexturedBackground {
}
TexturedBackgroundLight {
}
Floor {
  size 220 220
  tileSize 4 4
}
''' % GEO]
    for z in zones:
        zid = int(z['id'])
        colour = '0.2 0.4 0.9' if zid < 100 else '0.9 0.3 0.2'
        parts.append(marker(z['x'], z['y'], colour, float(z['radius']), f'zone {zid}'))
    parts.append(marker(pad[0], pad[1], '0.1 0.8 0.2', 0.8, 'pad'))
    parts.append(f'''Charger {{
  translation {pad[0]} {pad[1]} 0.02
  name "charger"
  battery [
    1e9, 1e9, 3000
  ]
  radius 1.2
}}
''')
    stack = {}
    for row in manifest:
        s = int(row['source']) - 1
        k = stack.get(s, 0)
        stack[s] = k + 1
        parts.append(package(row, sources[s], k))
    pad_zone = f'101,{pad[0]:g},{pad[1]:g},{pad[0] + 6:g},{pad[1]:g}'
    for k, h in enumerate(homes):
        rid = k + 1
        args = ['--zones', rel(a.zones), '--pad', pad_zone]
        parts.append(mavic(rid, h, args, 180000 * low.get(rid, 1.0)))
    parts.append(f'''Robot {{
  translation 0 -14 0.5
  name "issuer"
  controller "mrs_issuer"
  controllerArgs [
    "{rel(mission)}"
    "{rel(a.timeline)}"
    "{rel(base + '_results.csv')}"
  ]
  supervisor TRUE
  children [
    Emitter {{
      name "emitter"
      channel 1
    }}
    Receiver {{
      name "receiver"
      channel 1
    }}
  ]
}}
''')
    with open(a.out, 'w') as f:
        f.write('\n'.join(parts))


if __name__ == '__main__':
    main()
