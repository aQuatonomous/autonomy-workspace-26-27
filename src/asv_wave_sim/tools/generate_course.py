#!/usr/bin/env python3
"""Generate a RoboBoat 2026 Gazebo world and its answer key.

Reads course/roboboat2026.yaml and writes:
  <out>.sdf               the world (venue, water, course elements, ourboat at the start)
  <out>.answer_key.json   ground truth for the sim judge (positions in ENU and lat/lon,
                          gates, indicator colours, correct docking slip, boundary)

  --seed 0  the layout exactly as drawn in the handbook diagrams (default)
  --seed N  randomized within the rulebook limits listed under `randomize:` in the YAML

Examples:
  python3 tools/generate_course.py                       # worlds/roboboat2026_course.sdf
  python3 tools/generate_course.py --seed 7 --out /tmp/rb7/course
"""

import argparse
import json
import math
import os
import random
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_ROOT = os.path.normpath(os.path.join(HERE, '..'))
DEFAULT_CONFIG = os.path.join(SIM_ROOT, 'course', 'roboboat2026.yaml')
DEFAULT_OUT = os.path.join(SIM_ROOT, 'gz-waves-models', 'worlds', 'roboboat2026_course')

# ArduPilot's flat-earth conversion (Location::offset, LATLON_TO_M), so lat/lon in the answer key
# match what SITL reports through MAVROS for the same Gazebo position.
M_PER_DEG = 111318.84502145034

WAVE_PRESETS = {
    # name: (wind_speed m/s, steepness)
    'flat': (0.1, 0.0),
    'calm_lake': (1.0, 0.1),      # regatta lake behind a wave attenuator
    'breezy_lake': (3.0, 0.2),
    'legacy': (0.5, 0.2),         # values from the original aquatonomous_world
}

FOOTPRINT_RADIUS = {
    'sur_mark': 0.2286,
    'a0': 0.1016,
    'a2': 0.184,
    'indicator_buoy': 0.305,
    'boundary_marker': 0.15,
}

KIND_ALIASES = {'polyform_a0': 'a0', 'polyform_a2': 'a2'}

VESSEL_OUTLINE = [(-0.5334, -0.2832), (0.5334, -0.2832), (1.0334, 0.0), (0.5334, 0.2832), (-0.5334, 0.2832)]


def model_uri(kind, color=None, number=None):
    c = (color or '').capitalize()
    if kind == 'sur_mark':
        return f'model://{c}SurMark'
    if kind == 'a0':
        return f'model://{c}Ropehold'
    if kind == 'a2':
        return f'model://PolyformA2{c}'
    if kind == 'indicator_buoy':
        return f'model://IndicatorBuoy{c}'
    if kind == 'vessel':
        return f'model://StationaryVessel{c}'
    if kind == 'dock_indicator':
        return f'model://DockIndicator{c}'
    if kind == 'banner':
        return f'model://NumberBanner{number}'
    raise ValueError(kind)


class Frame:
    """Course frame (handbook drawing, u right / v up) -> Gazebo ENU -> WGS84.

    The Gazebo world origin sits at the course centre so the wave tiles cover the course;
    lat0/lon0 is the WGS84 position of that origin (the SITL home).
    """

    def __init__(self, venue, centre_uv):
        datum_lat = float(venue['datum']['lat'])
        datum_lon = float(venue['datum']['lon'])
        self.alt0 = float(venue['datum'].get('alt', 0.0))
        self.beta = math.radians(float(venue['course_up_bearing_deg']))
        self.ox, self.oy = self._rotate(*centre_uv)
        self.lat0 = datum_lat + self.oy / M_PER_DEG
        self.lon0 = datum_lon + self.ox / (M_PER_DEG * math.cos(math.radians(datum_lat)))

    def _rotate(self, u, v):
        cb, sb = math.cos(self.beta), math.sin(self.beta)
        return u * cb + v * sb, -u * sb + v * cb

    def enu(self, u, v):
        x, y = self._rotate(u, v)
        return x - self.ox, y - self.oy

    def course(self, x, y):
        x, y = x + self.ox, y + self.oy
        cb, sb = math.cos(self.beta), math.sin(self.beta)
        return x * cb - y * sb, x * sb + y * cb

    def yaw(self, course_yaw_deg):
        y = math.radians(course_yaw_deg) - self.beta
        return math.atan2(math.sin(y), math.cos(y))

    def latlon(self, x, y):
        lat = self.lat0 + y / M_PER_DEG
        lon = self.lon0 + x / (M_PER_DEG * math.cos(math.radians(self.lat0)))
        return lat, lon


def vec(yaw_deg):
    r = math.radians(yaw_deg)
    return math.cos(r), math.sin(r)


class Course:
    def __init__(self, cfg, seed):
        self.cfg = cfg
        self.seed = seed
        self.rng = random.Random(seed)
        self.random = seed != 0
        self.rand_cfg = cfg.get('randomize', {}) if self.random else {}
        b = cfg['boundary']
        self.frame = Frame(cfg['venue'], ((b['u_min'] + b['u_max']) / 2, (b['v_min'] + b['v_max']) / 2))
        self.elements = []
        self.gates = []
        self.models = []      # extra inline SDF models (docks, boundary markers)
        self.tasks = {}

    # -- helpers ---------------------------------------------------------
    def pick(self, key, default, lo_hi):
        if self.rand_cfg.get(key):
            return self.rng.uniform(*lo_hi)
        return default

    def add(self, task, kind, color, u, v, yaw=0.0, role=None, z=0.0, number=None, extra=None):
        kind = KIND_ALIASES.get(kind, kind)
        x, y = self.frame.enu(u, v)
        lat, lon = self.frame.latlon(x, y)
        eid = len(self.elements) + 1
        name = f't{task}_{role or kind}_{eid}'
        e = {
            'id': eid, 'name': name, 'task': task, 'kind': kind, 'color': color, 'role': role,
            'x': round(x, 4), 'y': round(y, 4), 'z': round(z, 4), 'yaw': round(self.frame.yaw(yaw), 5),
            'lat': round(lat, 8), 'lon': round(lon, 8), 'uri': model_uri(kind, color, number),
        }
        if kind in FOOTPRINT_RADIUS:
            e['radius'] = FOOTPRINT_RADIUS[kind]
        if kind == 'vessel':
            e['footprint'] = self.polygon(VESSEL_OUTLINE, x, y, e['yaw'])
        if number is not None:
            e['number'] = number
        if extra:
            e.update(extra)
        self.elements.append(e)
        return e

    @staticmethod
    def polygon(local, x, y, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        return [[round(x + px * c - py * s, 4), round(y + px * s + py * c, 4)] for px, py in local]

    def add_gate(self, task, label, model, cu, cv, yaw, width):
        fx, fy = vec(yaw)
        lx, ly = -fy, fx
        red = self.add(task, model, 'red', cu + lx * width / 2, cv + ly * width / 2, role=f'{label}_red')
        green = self.add(task, model, 'green', cu - lx * width / 2, cv - ly * width / 2, role=f'{label}_green')
        x, y = self.frame.enu(cu, cv)
        gate = {
            'task': task, 'label': label, 'red_id': red['id'], 'green_id': green['id'],
            'center': [round(x, 4), round(y, 4)], 'heading': round(self.frame.yaw(yaw), 5),
            'width': round(width, 4),
        }
        self.gates.append(gate)
        return gate, red, green

    def separated(self, u, v, others, min_sep):
        return all(math.hypot(u - a, v - b) >= min_sep for a, b in others)

    # -- tasks -------------------------------------------------------------
    def task1(self):
        t = self.cfg['tasks']['task1']
        w = self.pick('gate_width', t['gate_width'], t['gate_width_range'])
        spacing = self.pick('task1_gate_spacing', t['gate_spacing'], t['gate_spacing_range'])
        fx, fy = vec(t['yaw'])
        g1u, g1v = t['gate1']['u'], t['gate1']['v']
        g1, _, _ = self.add_gate(1, 'gate1', t['gate_model'], g1u, g1v, t['yaw'], w)
        g2, _, _ = self.add_gate(1, 'gate2', t['gate_model'], g1u + fx * spacing, g1v + fy * spacing, t['yaw'], w)
        self.tasks['1'] = {'name': 'Evacuation Route & Return', 'gates': [g1['label'], g2['label']],
                           'gate_width': round(w, 3), 'gate_spacing': round(spacing, 3)}

    def task2(self):
        t = self.cfg['tasks']['task2']
        ou, ov, s = t['origin']['u'], t['origin']['v'], t['spacing_scale']
        labels, gate_geo = [], []
        for i, g in enumerate(t['gates'], start=1):
            w = self.pick('gate_width', g['width'], t['gate_width_range'])
            cu, cv = ou + s * g['u'], ov + s * g['v']
            gate, red, green = self.add_gate(2, f'channel{i}', t['gate_model'], cu, cv, g['yaw'], w)
            labels.append(gate['label'])
            fx, fy = vec(g['yaw'])
            lx, ly = -fy, fx
            gate_geo.append(((cu + lx * w / 2, cv + ly * w / 2), (cu - lx * w / 2, cv - ly * w / 2), (fx, fy)))
        for o in t['channel_obstacles']:
            (ru, rv), (gu, gv), (fx, fy) = gate_geo[o['gate'] - 1]
            pu = ru + o['t'] * (gu - ru) + fx * o['forward']
            pv = rv + o['t'] * (gv - rv) + fy * o['forward']
            self.add(2, 'a0', 'black', pu, pv, role='channel_obstacle')

        df = t['debris_field']
        jitter = float(self.rand_cfg.get('debris_jitter', 0.0))
        min_sep = float(self.cfg.get('randomize', {}).get('min_separation', 1.0))
        placed = [(e_u, e_v) for (e_u, e_v) in self._course_positions(task=2)]

        def place(u, v):
            for _ in range(100):
                du = self.rng.uniform(-jitter, jitter) if jitter else 0.0
                dv = self.rng.uniform(-jitter, jitter) if jitter else 0.0
                pu, pv = ou + s * u + du, ov + s * v + dv
                if not jitter or self.separated(pu, pv, placed, min_sep):
                    placed.append((pu, pv))
                    return pu, pv
            placed.append((ou + s * u, ov + s * v))
            return ou + s * u, ov + s * v

        debris_ids = []
        for b in df['black_buoys']:
            debris_ids.append(self.add(2, 'a0', 'black', *place(b['u'], b['v']), role='debris')['id'])
        colors = [i['color'] for i in df['indicators']]
        if self.rand_cfg.get('task2_indicator_colors'):
            if self.rand_cfg.get('task2_require_both_colors') and len(colors) >= 2:
                colors = ['red', 'green'] + [self.rng.choice(['red', 'green']) for _ in colors[2:]]
                self.rng.shuffle(colors)
            else:
                colors = [self.rng.choice(['red', 'green']) for _ in colors]
        indicators = []
        for ind, color in zip(df['indicators'], colors):
            e = self.add(2, 'indicator_buoy', color, *place(ind['u'], ind['v']), role='debris_indicator')
            indicators.append({'id': e['id'], 'color': color})
            debris_ids.append(e['id'])
        for vsl in df.get('vessels', []):
            yaw = vsl['yaw'] + (self.rng.uniform(-30, 30) if jitter else 0.0)
            self.add(2, 'vessel', vsl['color'], *place(vsl['u'], vsl['v']), yaw=yaw, role='debris_vessel')
        self.tasks['2'] = {'name': 'Debris Clearance', 'gates': labels, 'indicators': indicators,
                           'debris_ids': debris_ids}

    def _course_positions(self, task):
        return [self.frame.course(e['x'], e['y']) for e in self.elements if e['task'] == task]

    def task3(self):
        t = self.cfg['tasks']['task3']
        w = self.pick('gate_width', t['gate_width'], t['gate_width_range'])
        dist = self.pick('task3_distance', t['distance'], t['distance_range'])
        cu, cv, yaw = t['gate']['u'], t['gate']['v'], t['gate']['yaw']
        fx, fy = vec(yaw)
        lx, ly = -fy, fx
        gate, _, _ = self.add_gate(3, 'speed_gate', t['gate_model'], cu, cv, yaw, w)

        def at(spec):
            a = dist * spec['along'] + spec.get('along_offset', 0.0)
            return cu + fx * a + lx * spec['left'], cv + fy * a + ly * spec['left']

        yellow = self.add(3, 'a2', 'yellow', *at(t['yellow']), role='yellow_buoy')
        color = t['indicator']['color']
        if self.rand_cfg.get('task3_indicator_color'):
            color = self.rng.choice(['red', 'green'])
        ind = self.add(3, 'indicator_buoy', color, *at(t['indicator']), role='speed_indicator')
        for b in t['black_buoys']:
            self.add(3, 'a2', 'black', *at(b), role='obstacle')
        for vsl in t.get('vessels', []):
            self.add(3, 'vessel', vsl['color'], *at(vsl), yaw=yaw + vsl['yaw'], role='obstacle_vessel')
        self.tasks['3'] = {
            'name': 'Emergency Response Sprint', 'gate': gate['label'], 'yellow_id': yellow['id'],
            'indicator_id': ind['id'], 'indicator_color': color,
            # red = circle from the right (counter-clockwise), green = from the left (clockwise)
            'required_direction': 'ccw' if color == 'red' else 'cw',
            'distance': round(dist, 3), 'gate_width': round(w, 3),
        }

    def task5(self):
        t = self.cfg['tasks']['task5']
        el = self.cfg['elements']
        cube = el['dock_cube']['size']
        c = cube[0]
        free = el['dock_cube']['freeboard']
        ou, ov = t['origin']['u'], t['origin']['v']
        n_rows, n_w, n_tine = t['spine_length_cubes'], t['spine_width_cubes'], t['tine_length_cubes']
        off = t['spine_centre_offset']
        tine_rows = t['tine_rows']
        slip_rows = [(tine_rows[k] + 1, tine_rows[k + 1] - 1) for k in range(len(tine_rows) - 1)]
        numbers = list(t['slip_numbers'])

        example = t.get('example', {})
        layout = {}
        for dock in t['docks']:
            name = dock['name']
            if self.rand_cfg.get('dock_indicators'):
                layout[name] = [{'indicator': self.rng.choice(['red', 'green'])} for _ in slip_rows]
            else:
                layout[name] = [dict(s) for s in example.get(name, [{'indicator': 'green'}] * len(slip_rows))]
        if self.rand_cfg.get('dock_indicators'):
            all_slips = [s for d in layout.values() for s in d]
            if not any(s['indicator'] == 'green' for s in all_slips):
                self.rng.choice(all_slips)['indicator'] = 'green'
            p = float(self.rand_cfg.get('dock_vessel_probability', 0.6))
            for s in all_slips:
                if s['indicator'] == 'red' and self.rng.random() < p:
                    s['vessel'] = self.rng.choice(['yellow', 'black'])

        slips_out = []
        for dock in t['docks']:
            name, side = dock['name'], dock['side']
            nums = list(numbers)
            if self.rand_cfg.get('slip_number_order'):
                self.rng.shuffle(nums)
            spine_u = ou + side * off
            inner_u = spine_u - side * (n_w * c / 2)
            tip_u = inner_u - side * n_tine * c
            cubes = []
            for r in range(n_rows):
                for k in range(n_w):
                    cubes.append((spine_u + (k - (n_w - 1) / 2) * c, ov - (r + 0.5) * c))
            for r in tine_rows:
                for k in range(n_tine):
                    cubes.append((inner_u - side * (k + 0.5) * c, ov - (r + 0.5) * c))
            self.models.append(self.dock_model(f'marina_dock_{name}', cubes, cube, free))
            for k, (r0, r1) in enumerate(slip_rows):
                v_top, v_bot = ov - r0 * c, ov - (r1 + 1) * c
                v_mid = (v_top + v_bot) / 2
                slot = layout[name][k]
                number = nums[k]
                face_yaw = 0.0 if side < 0 else 180.0   # banner faces the slip
                self.add(5, 'banner', None, spine_u, v_mid, yaw=face_yaw + 90.0, z=free, number=number,
                         role=f'banner_{name}{number}')
                ind = self.add(5, 'dock_indicator', slot['indicator'], spine_u - side * c / 2, v_mid, z=free,
                               role=f'dock_indicator_{name}{number}')
                vessel_id = None
                if slot.get('vessel'):
                    bow_yaw = 180.0 if side < 0 else 0.0   # bow toward the spine
                    vu = inner_u - side * (0.15 + 1.0334)
                    vessel_id = self.add(5, 'vessel', slot['vessel'], vu, v_mid, yaw=bow_yaw,
                                         role=f'slip_vessel_{name}{number}')['id']
                corners = [(inner_u, v_top), (tip_u, v_top), (tip_u, v_bot), (inner_u, v_bot)]
                poly = [list(map(lambda a: round(a, 4), self.frame.enu(a, b))) for a, b in corners]
                cx, cy = self.frame.enu((inner_u + tip_u) / 2, v_mid)
                slips_out.append({
                    'dock': name, 'slip': number, 'indicator': slot['indicator'], 'indicator_id': ind['id'],
                    'vessel_id': vessel_id, 'occupied': vessel_id is not None, 'polygon': poly,
                    'center': [round(cx, 4), round(cy, 4)],
                    'entry_heading': round(self.frame.yaw(0.0 if side > 0 else 180.0), 5),
                })
        greens = [s for s in slips_out if s['indicator'] == 'green' and not s['occupied']]
        best = min((s['slip'] for s in greens), default=None)
        self.tasks['5'] = {
            'name': 'Navigate the Marina', 'slips': slips_out,
            'correct': [{'dock': s['dock'], 'slip': s['slip']} for s in greens if s['slip'] == best],
            'green': [{'dock': s['dock'], 'slip': s['slip']} for s in greens],
        }

    def dock_model(self, name, cubes, size, freeboard):
        sx, sy, sz = size
        zc = freeboard - sz / 2
        gap = 0.012
        vis, cols = [], []
        for i, (u, v) in enumerate(cubes):
            x, y = self.frame.enu(u, v)
            p = f'<pose>{x:.4f} {y:.4f} {zc:.4f} 0 0 {self.frame.yaw(0.0):.5f}</pose>'
            vis.append(f'<visual name="cube{i}">{p}<geometry><box><size>{sx - gap:.4f} {sy - gap:.4f} {sz:.4f}'
                       f'</size></box></geometry><material><ambient>0.74 0.66 0.55 1</ambient>'
                       f'<diffuse>0.76 0.68 0.56 1</diffuse><specular>0.1 0.1 0.1 1</specular></material></visual>')
            cols.append(f'<collision name="c{i}">{p}<geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box>'
                        f'</geometry></collision>')
        footprint = [[round(c, 4) for c in self.frame.enu(u, v)] for u, v in cubes]
        self.elements.append({'id': len(self.elements) + 1, 'name': name, 'task': 5, 'kind': 'dock',
                              'cubes': footprint, 'cube_size': sx})
        body = '\n        '.join(vis + cols)
        return f"""
    <model name="{name}">
      <static>true</static>
      <link name="link">
        {body}
      </link>
    </model>"""

    def boundary(self):
        b = self.cfg['boundary']
        corners = [(b['u_min'], b['v_min']), (b['u_max'], b['v_min']), (b['u_max'], b['v_max']), (b['u_min'], b['v_max'])]
        poly = []
        for i, (u, v) in enumerate(corners):
            x, y = self.frame.enu(u, v)
            poly.append([round(x, 4), round(y, 4)])
            self.models.append(f"""
    <model name="boundary_marker_{i}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} 0 0 0 0</pose>
      <link name="link">
        <visual name="post"><pose>0 0 0.6 0 0 0</pose><geometry><cylinder><radius>0.15</radius><length>1.2</length></cylinder></geometry>
          <material><ambient>0.95 0.95 0.95 1</ambient><diffuse>0.95 0.95 0.95 1</diffuse></material></visual>
        <collision name="post"><pose>0 0 0.6 0 0 0</pose><geometry><cylinder><radius>0.15</radius><length>1.2</length></cylinder></geometry></collision>
      </link>
    </model>""")
        self.boundary_polygon = poly

    def build(self):
        self.task1()
        self.task2()
        self.task3()
        self.task5()
        self.boundary()
        s = self.cfg['start']
        x, y = self.frame.enu(s['u'], s['v'])
        yaw = self.frame.yaw(s['yaw'])
        self.start = {'x': round(x, 4), 'y': round(y, 4), 'yaw': round(yaw, 5)}


# ---------------------------------------------------------------------------
# World SDF
# ---------------------------------------------------------------------------

def waves_model(preset):
    wind, steep = WAVE_PRESETS[preset]
    wave = f"""<wave>
            <algorithm>fft</algorithm>
            <tile_size>64</tile_size>
            <cell_count>64</cell_count>
            <wind_speed>{wind}</wind_speed>
            <wind_angle_deg>135</wind_angle_deg>
            <steepness>{steep}</steepness>
          </wave>"""
    return f"""
    <model name="waves">
      <static>true</static>
      <plugin filename="gz-waves1-waves-model-system" name="gz::sim::systems::WavesModel">
        <static>0</static>
        <update_rate>20</update_rate>
        {wave}
      </plugin>
      <link name="base_link">
        <visual name="visual">
          <visibility_flags>2</visibility_flags>
          <geometry><mesh><uri>model://waves/materials/mesh_L256m_N256.dae</uri></mesh></geometry>
          <material>
            <ambient>0.05 0.18 0.22 0.92</ambient>
            <diffuse>0.08 0.25 0.30 0.92</diffuse>
            <specular>0.7 0.7 0.7 0.92</specular>
            <pbr><metal>
              <albedo_map>model://waves/materials/water.jpg</albedo_map>
              <normal_map>model://waves/materials/wave_normals.dds</normal_map>
              <environment_map>model://waves/materials/skybox_lowres.dds</environment_map>
              <roughness>0.3</roughness>
              <metalness>0.1</metalness>
            </metal></pbr>
          </material>
          <plugin filename="gz-waves1-waves-visual-system" name="gz::sim::systems::WavesVisual">
            <static>0</static>
            <mesh_deformation_method>DYNAMIC_GEOMETRY</mesh_deformation_method>
            <tiles_x>-2 2</tiles_x>
            <tiles_y>-2 2</tiles_y>
            {wave}
            <shader language="metal">
              <vertex>model://waves/materials/waves_vs.metal</vertex>
              <fragment>model://waves/materials/waves_fs.metal</fragment>
            </shader>
            <param><shader>vertex</shader><name>world_matrix</name></param>
            <param><shader>vertex</shader><name>worldviewproj_matrix</name></param>
            <param><shader>vertex</shader><name>camera_position</name></param>
            <param><shader>vertex</shader><name>rescale</name><value>0.5</value><type>float</type></param>
            <param><shader>vertex</shader><name>bumpScale</name><value>128 128</value><type>float_array</type></param>
            <param><shader>vertex</shader><name>bumpSpeed</name><value>0.01 0.01</value><type>float_array</type></param>
            <param><shader>vertex</shader><name>t</name><value>TIME</value></param>
            <param><shader>fragment</shader><name>deepColor</name><value>0.0 0.05 0.2 1.0</value><type>float_array</type></param>
            <param><shader>fragment</shader><name>shallowColor</name><value>0.0 0.1 0.3 1.0</value><type>float_array</type></param>
            <param><shader>fragment</shader><name>fresnelPower</name><value>5.0</value><type>float</type></param>
            <param><shader>fragment</shader><name>hdrMultiplier</name><value>0.4</value><type>float</type></param>
            <param><shader>fragment</shader><name>bumpMap</name><value>model://waves/materials/wave_normals.dds</value><type>texture</type><arg>0</arg></param>
            <param><shader>fragment</shader><name>cubeMap</name><value>model://waves/materials/skybox_lowres.dds</value><type>texture_cube</type><arg>1</arg></param>
          </plugin>
        </visual>
      </link>
    </model>"""


def shoreline(center_x, center_y, rng):
    """Low-poly shoreline of the regatta lake: grass banks, tree lines and the finish tower."""
    parts = []
    lake_half_width = 95.0
    for side, name in ((1, 'north'), (-1, 'south')):
        y = center_y + side * lake_half_width
        parts.append(f'<visual name="{name}_bank"><pose>{center_x:.1f} {y + side * 15:.1f} 0.6 0 0 0</pose>'
                     f'<geometry><box><size>1600 30 1.2</size></box></geometry><material>'
                     f'<ambient>0.30 0.45 0.20 1</ambient><diffuse>0.33 0.50 0.22 1</diffuse></material></visual>')
        x = center_x - 300.0
        i = 0
        while x < center_x + 300.0:
            h = rng.uniform(6.0, 14.0)
            w = rng.uniform(8.0, 20.0)
            parts.append(f'<visual name="{name}_trees{i}"><pose>{x:.1f} {y + side * rng.uniform(18, 28):.1f} '
                         f'{h / 2 + 1.2:.1f} 0 0 0</pose><geometry><box><size>{w:.1f} 8 {h:.1f}</size></box>'
                         f'</geometry><material><ambient>0.10 0.25 0.10 1</ambient><diffuse>0.12 0.30 0.12 1'
                         f'</diffuse></material></visual>')
            x += w + rng.uniform(2.0, 12.0)
            i += 1
    # Finish tower and boathouse on the south bank (Nathan Benderson Park landmarks), stylized.
    parts.append(f'<visual name="finish_tower"><pose>{center_x + 20:.1f} {center_y - lake_half_width - 6:.1f} 9 0 0 0'
                 f'</pose><geometry><box><size>10 10 18</size></box></geometry><material><ambient>0.85 0.82 0.75 1'
                 f'</ambient><diffuse>0.88 0.85 0.78 1</diffuse></material></visual>')
    parts.append(f'<visual name="boathouse"><pose>{center_x - 40:.1f} {center_y - lake_half_width - 10:.1f} 4 0 0 0'
                 f'</pose><geometry><box><size>40 14 8</size></box></geometry><material><ambient>0.75 0.70 0.62 1'
                 f'</ambient><diffuse>0.78 0.72 0.64 1</diffuse></material></visual>')
    body = '\n        '.join(parts)
    return f"""
    <model name="shoreline">
      <static>true</static>
      <link name="link">
        {body}
      </link>
    </model>"""


def sun_direction(azimuth_deg, elevation_deg):
    az, el = math.radians(azimuth_deg), math.radians(elevation_deg)
    to_sun = (math.sin(az) * math.cos(el), math.cos(az) * math.cos(el), math.sin(el))
    return tuple(-c for c in to_sun)


def world_sdf(course, args):
    cfg = course.cfg
    fr = course.frame
    s = course.start
    dx, dy, dz = sun_direction(args.sun_azimuth, args.sun_elevation)
    includes = []
    for e in course.elements:
        if 'uri' not in e:
            continue
        includes.append(f"""    <include>
      <name>{e['name']}</name>
      <pose>{e['x']:.4f} {e['y']:.4f} {e['z']:.4f} 0 0 {e['yaw']:.5f}</pose>
      <uri>{e['uri']}</uri>
    </include>""")
    xs = [p[0] for p in course.boundary_polygon]
    ys = [p[1] for p in course.boundary_polygon]
    cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
    shore = shoreline(cx, cy, random.Random(1234)) if not args.no_shoreline else ''
    boat = '' if args.no_boat else f"""
    <include>
      <name>ourboat</name>
      <pose>{s['x']:.4f} {s['y']:.4f} 0 0 0 {s['yaw']:.5f}</pose>
      <uri>model://ourboat</uri>
    </include>"""
    return f"""<?xml version="1.0" ?>
<!-- Generated by src/asv_wave_sim/tools/generate_course.py (seed {course.seed}). Do not edit by hand. -->
<sdf version="1.9">
  <world name="aquatonomous_course">
    <physics name="2ms" type="ode">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"></plugin>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"></plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"></plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"></plugin>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"></plugin>

    <!-- {cfg['venue']['name']} -->
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{fr.lat0}</latitude_deg>
      <longitude_deg>{fr.lon0}</longitude_deg>
      <elevation>{fr.alt0}</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>

    <scene>
      <ambient>0.55 0.55 0.58 1</ambient>
      <background>0.62 0.76 0.92 1</background>
      <sky><clouds><speed>2</speed><humidity>0.35</humidity></clouds></sky>
      <shadows>{'true' if args.shadows else 'false'}</shadows>
      <grid>false</grid>
    </scene>

    <!-- Florida sun: azimuth {args.sun_azimuth} deg, elevation {args.sun_elevation} deg -->
    <light type="directional" name="sun">
      <cast_shadows>{'true' if args.shadows else 'false'}</cast_shadows>
      <pose>0 0 50 0 0 0</pose>
      <diffuse>1.0 0.97 0.90 1</diffuse>
      <specular>0.6 0.6 0.6 1</specular>
      <intensity>1.2</intensity>
      <direction>{dx:.4f} {dy:.4f} {dz:.4f}</direction>
    </light>
{waves_model(args.waves)}
{shore}
{''.join(course.models)}
{chr(10).join(includes)}
{boat}
  </world>
</sdf>
"""


def answer_key(course, args, world_path):
    fr = course.frame
    s = course.start
    lat, lon = fr.latlon(s['x'], s['y'])
    heading = (90.0 - math.degrees(s['yaw'])) % 360.0
    return {
        'generator': 'src/asv_wave_sim/tools/generate_course.py',
        'seed': course.seed,
        'world': os.path.basename(world_path),
        'world_name': 'aquatonomous_course',
        'venue': course.cfg['venue']['name'],
        'datum': {'lat': fr.lat0, 'lon': fr.lon0, 'alt': fr.alt0, 'm_per_deg': M_PER_DEG},
        # SITL home must be the Gazebo world origin: ArduPilot adds the JSON position (relative to the
        # world origin) to its home location.
        'sitl_home': f'{fr.lat0:.8f},{fr.lon0:.8f},{fr.alt0:.1f},{heading:.0f}',
        'start': {**s, 'lat': round(lat, 8), 'lon': round(lon, 8), 'compass_heading_deg': round(heading, 1)},
        'boundary': course.boundary_polygon,
        'waves': args.waves,
        'water_density': course.cfg['venue'].get('water_density', 1000.0),
        'gates': course.gates,
        'tasks': course.tasks,
        'elements': course.elements,
    }


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--config', default=DEFAULT_CONFIG)
    ap.add_argument('--seed', type=int, default=0, help='0 = layout as drawn; N > 0 = randomized')
    ap.add_argument('--out', default=DEFAULT_OUT, help='output path without extension')
    ap.add_argument('--waves', default=None, choices=sorted(WAVE_PRESETS), help='wave preset (default from YAML)')
    ap.add_argument('--sun-azimuth', type=float, default=120.0, help='compass degrees')
    ap.add_argument('--sun-elevation', type=float, default=50.0)
    ap.add_argument('--shadows', action='store_true', help='enable sun shadows (costs GPU time)')
    ap.add_argument('--no-shoreline', action='store_true')
    ap.add_argument('--no-boat', action='store_true', help='omit ourboat (course preview)')
    args = ap.parse_args(argv)

    with open(args.config) as f:
        cfg = yaml.safe_load(f)
    if args.waves is None:
        args.waves = cfg['venue'].get('waves_preset', 'calm_lake')

    course = Course(cfg, args.seed)
    course.build()

    out_dir = os.path.dirname(os.path.abspath(args.out))
    os.makedirs(out_dir, exist_ok=True)
    world_path = args.out + '.sdf'
    key_path = args.out + '.answer_key.json'
    with open(world_path, 'w', newline='\n') as f:
        f.write(world_sdf(course, args))
    with open(key_path, 'w', newline='\n') as f:
        json.dump(answer_key(course, args, world_path), f, indent=2)
        f.write('\n')

    t2 = course.tasks['2']['indicators']
    t3 = course.tasks['3']
    t5 = course.tasks['5']
    print(f'world:      {world_path}')
    print(f'answer key: {key_path}')
    print(f'seed {args.seed}: task2 indicators {[i["color"] for i in t2]}, task3 {t3["indicator_color"]} '
          f'({t3["required_direction"]}), dock target {t5["correct"]}')
    print(f'SITL home:  {answer_key(course, args, world_path)["sitl_home"]}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
