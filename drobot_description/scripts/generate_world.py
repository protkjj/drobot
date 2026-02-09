#!/usr/bin/env python3
"""
Jinja2 기반 월드 생성기 for Drobot

YAML config + SDF 템플릿 → world.sdf

Usage:
    python3 generate_world.py config.yaml
    python3 generate_world.py config.yaml -o my_world.sdf
    python3 generate_world.py -i                # Interactive mode
"""
import argparse
import math
import os
import random
import sys
from typing import Any, Dict, List, Optional, Tuple

import yaml
from jinja2 import Environment, FileSystemLoader


# ==================== 색상 프리셋 ====================

COLORS = {
    'red':       {'r': 0.8, 'g': 0.2, 'b': 0.2},
    'green':     {'r': 0.2, 'g': 0.6, 'b': 0.2},
    'blue':      {'r': 0.2, 'g': 0.2, 'b': 0.8},
    'yellow':    {'r': 0.8, 'g': 0.8, 'b': 0.2},
    'orange':    {'r': 0.8, 'g': 0.4, 'b': 0.1},
    'brown':     {'r': 0.4, 'g': 0.25, 'b': 0.1},
    'white':     {'r': 0.9, 'g': 0.9, 'b': 0.9},
    'gray':      {'r': 0.5, 'g': 0.5, 'b': 0.5},
    'dark_gray': {'r': 0.3, 'g': 0.3, 'b': 0.3},
    'asphalt':   {'r': 0.2, 'g': 0.2, 'b': 0.2},
}


class WorldGenerator:
    """YAML config + Jinja2 템플릿 기반 월드 생성기"""

    def __init__(self, yaml_path: str):
        with open(yaml_path, 'r') as f:
            self.config = yaml.safe_load(f)['world']

        self.world_name = self.config.get('name', 'generated_world')

        # 스폰 설정
        spawn = self.config.get('spawn', {})
        spawn_pos = spawn.get('pos', {'x': 0, 'y': 0})
        self.spawn_x = spawn_pos['x']
        self.spawn_y = spawn_pos['y']
        self.spawn_safe_radius = spawn.get('safe_radius', 1.5)

        # 충돌 방지용 점유 위치
        self._occupied: List[Tuple[float, float, float]] = []
        self._model_count = 0

        # Jinja2 환경 설정
        self._base_dir = os.path.dirname(os.path.abspath(__file__))
        models_dir = os.path.join(self._base_dir, '..', 'models')
        self.env = Environment(
            loader=FileSystemLoader(models_dir),
            keep_trailing_newline=True,
        )

    # ==================== 유틸리티 ====================

    def _resolve_color(self, color) -> Dict[str, float]:
        """문자열("red") 또는 dict({r,g,b})를 RGB dict로 변환"""
        if isinstance(color, str):
            return COLORS.get(color, COLORS['gray'])
        if isinstance(color, dict):
            return color
        return COLORS['gray']

    def _next_name(self, prefix: str) -> str:
        self._model_count += 1
        return f'{prefix}_{self._model_count}'

    def _is_position_safe(self, x: float, y: float, radius: float) -> bool:
        dist = math.sqrt((x - self.spawn_x)**2 + (y - self.spawn_y)**2)
        if dist < self.spawn_safe_radius + radius:
            return False
        for ox, oy, orad in self._occupied:
            if math.sqrt((x - ox)**2 + (y - oy)**2) < orad + radius + 1.0:
                return False
        return True

    def _find_safe_position(
        self, radius: float, half_w: float, half_h: float, margin: float = 0.5
    ) -> Optional[Tuple[float, float]]:
        hw = half_w - margin - radius
        hh = half_h - margin - radius
        for _ in range(100):
            x = random.uniform(-hw, hw)
            y = random.uniform(-hh, hh)
            if self._is_position_safe(x, y, radius):
                return (x, y)
        return None

    def _render(self, template_path: str, **kwargs) -> str:
        return self.env.get_template(template_path).render(**kwargs)

    # ==================== 벽 생성 ====================

    def _generate_walls(self) -> str:
        walls_cfg = self.config.get('walls')
        if not walls_cfg or not walls_cfg.get('enabled', True):
            return ''

        w = walls_cfg['size']['w']
        h = walls_cfg['size']['h']
        height = walls_cfg.get('height', 1.0)
        thickness = walls_cfg.get('thickness', 0.1)
        color = self._resolve_color(walls_cfg.get('color', 'gray'))
        doors = walls_cfg.get('doors', [])

        half_w = w / 2
        half_h = h / 2
        segments = []

        # 각 방향별 벽 생성
        directions = {
            'north': {'full': (0, half_h, height/2, w, thickness, height),
                      'left': (-w/4 - 0.25, half_h, height/2, (w-1)/2, thickness, height),
                      'right': (w/4 + 0.25, half_h, height/2, (w-1)/2, thickness, height)},
            'south': {'full': (0, -half_h, height/2, w, thickness, height),
                      'left': (-w/4 - 0.25, -half_h, height/2, (w-1)/2, thickness, height),
                      'right': (w/4 + 0.25, -half_h, height/2, (w-1)/2, thickness, height)},
            'east':  {'full': (half_w, 0, height/2, thickness, h, height),
                      'left': (half_w, h/4 + 0.25, height/2, thickness, (h-1)/2, height),
                      'right': (half_w, -h/4 - 0.25, height/2, thickness, (h-1)/2, height)},
            'west':  {'full': (-half_w, 0, height/2, thickness, h, height),
                      'left': (-half_w, h/4 + 0.25, height/2, thickness, (h-1)/2, height),
                      'right': (-half_w, -h/4 - 0.25, height/2, thickness, (h-1)/2, height)},
        }

        for direction, data in directions.items():
            if direction in doors:
                for side in ('left', 'right'):
                    x, y, z, sx, sy, sz = data[side]
                    segments.append(self._render('obstacles/wall.sdf.j2',
                        name=f'wall_{direction}_{side}',
                        pose={'x': x, 'y': y, 'z': z},
                        size={'x': sx, 'y': sy, 'z': sz},
                        color=color))
            else:
                x, y, z, sx, sy, sz = data['full']
                segments.append(self._render('obstacles/wall.sdf.j2',
                    name=f'wall_{direction}',
                    pose={'x': x, 'y': y, 'z': z},
                    size={'x': sx, 'y': sy, 'z': sz},
                    color=color))

        return '\n'.join(segments)

    # ==================== 정적 장애물 ====================

    def _generate_static_obstacles(self) -> str:
        obstacles = self.config.get('obstacles', {}).get('static', [])
        parts = []

        for obs in obstacles:
            model = obs['model']
            color = self._resolve_color(obs.get('color', 'gray'))
            name = obs.get('name') or self._next_name(model)

            if model == 'box':
                size = obs['size']
                z = size['z'] / 2
                parts.append(self._render('obstacles/box.sdf.j2',
                    name=name,
                    pose={'x': obs['pos']['x'], 'y': obs['pos']['y'], 'z': z},
                    size=size,
                    color=color,
                    static=True))
                self._occupied.append((obs['pos']['x'], obs['pos']['y'],
                                       max(size['x'], size['y']) / 2))

            elif model == 'cylinder':
                z = obs['height'] / 2
                parts.append(self._render('obstacles/cylinder.sdf.j2',
                    name=name,
                    pose={'x': obs['pos']['x'], 'y': obs['pos']['y'], 'z': z},
                    radius=obs['radius'],
                    height=obs['height'],
                    color=color,
                    static=True))
                self._occupied.append((obs['pos']['x'], obs['pos']['y'], obs['radius']))

        return '\n'.join(parts)

    # ==================== 이동 장애물 ====================

    def _generate_moving_obstacles(self) -> str:
        obstacles = self.config.get('obstacles', {}).get('moving', [])
        parts = []

        for obs in obstacles:
            model = obs['model']
            color = self._resolve_color(obs.get('color', 'yellow'))
            name = obs.get('name') or self._next_name(f'moving_{model}')

            if model == 'box':
                size = obs['size']
                z = size['z'] / 2
                parts.append(self._render('obstacles/box.sdf.j2',
                    name=name,
                    pose={'x': obs['pos']['x'], 'y': obs['pos']['y'], 'z': z},
                    size=size,
                    color=color,
                    static=False))

            elif model == 'cylinder':
                z = obs['height'] / 2
                parts.append(self._render('obstacles/cylinder.sdf.j2',
                    name=name,
                    pose={'x': obs['pos']['x'], 'y': obs['pos']['y'], 'z': z},
                    radius=obs['radius'],
                    height=obs['height'],
                    color=color,
                    static=False))

        return '\n'.join(parts)

    # ==================== 랜덤 장애물 ====================

    def _generate_random_obstacles(self) -> str:
        rand_cfg = self.config.get('obstacles', {}).get('random')
        if not rand_cfg:
            return ''

        count = rand_cfg.get('count', 5)
        models = rand_cfg.get('models', ['box', 'cylinder'])
        size_min, size_max = rand_cfg.get('size_range', [0.3, 0.8])
        seed = rand_cfg.get('seed')

        if seed is not None:
            random.seed(seed)

        # 벽 크기로 배치 범위 결정
        walls_cfg = self.config.get('walls', {})
        if walls_cfg.get('enabled', False):
            half_w = walls_cfg['size']['w'] / 2
            half_h = walls_cfg['size']['h'] / 2
        else:
            half_w = half_h = 10.0

        color_names = ['red', 'blue', 'green', 'orange', 'yellow']
        parts = []
        placed = 0

        for _ in range(count * 3):
            if placed >= count:
                break

            size = random.uniform(size_min, size_max)
            pos = self._find_safe_position(size / 2, half_w, half_h)
            if pos is None:
                continue

            x, y = pos
            color = self._resolve_color(random.choice(color_names))
            model = random.choice(models)

            if model == 'box':
                w = random.uniform(size_min, size_max)
                d = random.uniform(size_min, size_max)
                h = random.uniform(0.4, 0.8)
                parts.append(self._render('obstacles/box.sdf.j2',
                    name=self._next_name('random_box'),
                    pose={'x': x, 'y': y, 'z': h / 2},
                    size={'x': w, 'y': d, 'z': h},
                    color=color,
                    static=True))
                self._occupied.append((x, y, max(w, d) / 2))
            else:
                r = random.uniform(size_min / 2, size_max / 2)
                h = random.uniform(0.4, 0.8)
                parts.append(self._render('obstacles/cylinder.sdf.j2',
                    name=self._next_name('random_cylinder'),
                    pose={'x': x, 'y': y, 'z': h / 2},
                    radius=r,
                    height=h,
                    color=color,
                    static=True))
                self._occupied.append((x, y, r))

            placed += 1

        print(f'[WorldGenerator] Random obstacles: {placed}/{count} placed')
        return '\n'.join(parts)

    # ==================== 도로 ====================

    def _generate_roads(self) -> str:
        roads = self.config.get('roads', [])
        if not roads:
            return ''

        asphalt = COLORS['asphalt']
        road_yellow = {'r': 0.9, 'g': 0.8, 'b': 0.1}
        road_white = {'r': 0.95, 'g': 0.95, 'b': 0.95}

        parts = []

        for i, road_cfg in enumerate(roads):
            x1, y1 = road_cfg['start']['x'], road_cfg['start']['y']
            x2, y2 = road_cfg['end']['x'], road_cfg['end']['y']
            width = road_cfg.get('width', 4.0)
            lanes = road_cfg.get('lanes', 2)

            length = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            angle = math.atan2(y2 - y1, x2 - x1)
            cx = (x1 + x2) / 2
            cy = (y1 + y2) / 2

            road_name = f'road_{i + 1}'

            # 도로 바닥
            parts.append(self._render('environment/road.sdf.j2',
                name=road_name,
                pose={'x': cx, 'y': cy, 'yaw': angle},
                length=length,
                width=width,
                color=asphalt))

            # 중앙선 (점선)
            if lanes >= 2:
                parts.extend(self._make_lane_markings(
                    cx, cy, length, angle, offset=0,
                    color=road_yellow, dashed=True,
                    name_prefix=f'{road_name}_center'))

            # 양쪽 가장자리 (실선)
            for side, off in [('left', width/2 - 0.1), ('right', -(width/2 - 0.1))]:
                parts.extend(self._make_lane_markings(
                    cx, cy, length, angle, offset=off,
                    color=road_white, dashed=False,
                    name_prefix=f'{road_name}_edge_{side}'))

        return '\n'.join(parts)

    def _make_lane_markings(
        self, cx, cy, length, angle, offset, color, dashed, name_prefix
    ) -> List[str]:
        perp = angle + math.pi / 2
        mx = cx + offset * math.cos(perp)
        my = cy + offset * math.sin(perp)

        parts = []

        if dashed:
            dash_len = 1.0
            gap_len = 2.0
            num = int(length / (dash_len + gap_len))
            for j in range(num):
                dist = (j - num / 2) * (dash_len + gap_len) + dash_len / 2
                dx = mx + dist * math.cos(angle)
                dy = my + dist * math.sin(angle)
                parts.append(self._render('environment/lane_marking.sdf.j2',
                    name=f'{name_prefix}_{j}',
                    pose={'x': dx, 'y': dy, 'yaw': angle},
                    length=dash_len,
                    color=color))
        else:
            parts.append(self._render('environment/lane_marking.sdf.j2',
                name=name_prefix,
                pose={'x': mx, 'y': my, 'yaw': angle},
                length=length,
                color=color))

        return parts

    # ==================== 생성 & 저장 ====================

    def generate(self) -> str:
        ground_color = self._resolve_color(self.config.get('ground_color', 'gray'))

        model_parts = []
        model_parts.append(self._generate_walls())
        model_parts.append(self._generate_static_obstacles())
        model_parts.append(self._generate_moving_obstacles())
        model_parts.append(self._generate_random_obstacles())
        model_parts.append(self._generate_roads())

        models_str = '\n'.join(p for p in model_parts if p)

        return self._render('base/world.sdf.j2',
            world_name=self.world_name,
            ground_color=ground_color,
            models=models_str)

    def save(self, output_name: str, output_dir: Optional[str] = None) -> str:
        if output_dir is None:
            output_dir = os.path.join(self._base_dir, '..', 'worlds', 'generated')

        os.makedirs(output_dir, exist_ok=True)

        if not output_name.endswith('.sdf'):
            output_name += '.sdf'

        filepath = os.path.join(output_dir, output_name)
        with open(filepath, 'w') as f:
            f.write(self.generate())

        print(f'[WorldGenerator] Saved: {filepath}')
        return filepath


# ==================== Interactive UI ====================

class InteractiveBuilder:
    """단계별 프롬프트로 YAML config를 생성하는 대화형 UI"""

    COLOR_NAMES = list(COLORS.keys())

    def __init__(self):
        self._base_dir = os.path.dirname(os.path.abspath(__file__))
        self.config = {}

    # ---------- 입력 헬퍼 ----------

    def _input(self, prompt: str, default: str = '') -> str:
        suffix = f' [{default}]' if default else ''
        val = input(f'  {prompt}{suffix}: ').strip()
        return val if val else default

    def _input_float(self, prompt: str, default: float) -> float:
        val = self._input(prompt, str(default))
        try:
            return float(val)
        except ValueError:
            print(f'    -> 숫자가 아닙니다. 기본값 {default} 사용')
            return default

    def _input_int(self, prompt: str, default: int) -> int:
        val = self._input(prompt, str(default))
        try:
            return int(val)
        except ValueError:
            print(f'    -> 숫자가 아닙니다. 기본값 {default} 사용')
            return default

    def _input_yn(self, prompt: str, default: bool = True) -> bool:
        d = 'Y/n' if default else 'y/N'
        val = input(f'  {prompt} ({d}): ').strip().lower()
        if not val:
            return default
        return val in ('y', 'yes')

    def _input_choice(self, prompt: str, choices: List[str], default: str = '') -> str:
        choices_str = ', '.join(choices)
        val = self._input(f'{prompt} ({choices_str})', default)
        if val not in choices:
            print(f'    -> 잘못된 선택. 기본값 {default} 사용')
            return default
        return val

    def _header(self, title: str):
        print(f'\n{"=" * 50}')
        print(f'  {title}')
        print(f'{"=" * 50}')

    def _step(self, num: int, title: str):
        print(f'\n--- Step {num}: {title} ---')

    # ---------- 각 단계 ----------

    def step_world_name(self):
        self._header('Drobot World Generator - Interactive Mode')
        self._step(1, '월드 이름')
        name = self._input('월드 이름', 'my_world')
        ground = self._input_choice('바닥 색상', self.COLOR_NAMES, 'gray')
        self.config = {
            'world': {
                'name': name,
                'ground_color': ground,
            }
        }

    def step_walls(self):
        self._step(2, '벽 설정')
        if not self._input_yn('벽을 추가할까요?', True):
            return

        w = self._input_float('가로 크기 (m)', 10.0)
        h = self._input_float('세로 크기 (m)', 10.0)
        height = self._input_float('벽 높이 (m)', 1.0)
        color = self._input_choice('벽 색상', self.COLOR_NAMES, 'gray')

        doors = []
        if self._input_yn('문을 추가할까요?', False):
            for d in ['north', 'south', 'east', 'west']:
                if self._input_yn(f'  {d} 문?', False):
                    doors.append(d)

        self.config['world']['walls'] = {
            'enabled': True,
            'size': {'w': w, 'h': h},
            'height': height,
            'thickness': 0.1,
            'color': color,
            'doors': doors,
        }

    def step_spawn(self):
        self._step(3, '스폰 위치')
        x = self._input_float('스폰 X', 0.0)
        y = self._input_float('스폰 Y', 0.0)
        radius = self._input_float('안전 반경 (m)', 1.5)
        self.config['world']['spawn'] = {
            'pos': {'x': x, 'y': y},
            'safe_radius': radius,
        }

    def step_obstacles(self):
        self._step(4, '장애물 설정')
        obstacles = {}

        # 정적 장애물
        static_list = []
        if self._input_yn('정적 장애물을 수동 배치할까요?', False):
            while True:
                model = self._input_choice('모델', ['box', 'cylinder'], 'box')
                x = self._input_float('X 위치', 0.0)
                y = self._input_float('Y 위치', 0.0)

                obs = {'model': model, 'pos': {'x': x, 'y': y}}

                if model == 'box':
                    sx = self._input_float('가로 크기', 0.5)
                    sy = self._input_float('세로 크기', 0.5)
                    sz = self._input_float('높이', 0.8)
                    obs['size'] = {'x': sx, 'y': sy, 'z': sz}
                else:
                    r = self._input_float('반지름', 0.3)
                    h = self._input_float('높이', 0.6)
                    obs['radius'] = r
                    obs['height'] = h

                color = self._input_choice('색상', self.COLOR_NAMES, 'red')
                obs['color'] = color
                static_list.append(obs)
                print(f'    -> {model} 추가됨 ({x}, {y})')

                if not self._input_yn('장애물 더 추가?', False):
                    break

        if static_list:
            obstacles['static'] = static_list

        # 이동 장애물
        moving_list = []
        if self._input_yn('이동 장애물을 추가할까요?', False):
            while True:
                model = self._input_choice('모델', ['box', 'cylinder'], 'box')
                x = self._input_float('초기 X 위치', 0.0)
                y = self._input_float('초기 Y 위치', 0.0)

                obs = {'model': model, 'pos': {'x': x, 'y': y}}

                if model == 'box':
                    sx = self._input_float('가로 크기', 0.4)
                    sy = self._input_float('세로 크기', 0.4)
                    sz = self._input_float('높이', 0.6)
                    obs['size'] = {'x': sx, 'y': sy, 'z': sz}
                else:
                    r = self._input_float('반지름', 0.3)
                    h = self._input_float('높이', 0.6)
                    obs['radius'] = r
                    obs['height'] = h

                color = self._input_choice('색상', self.COLOR_NAMES, 'yellow')
                obs['color'] = color
                moving_list.append(obs)
                print(f'    -> 이동 {model} 추가됨 ({x}, {y})')

                if not self._input_yn('이동 장애물 더 추가?', False):
                    break

        if moving_list:
            obstacles['moving'] = moving_list

        # 랜덤 장애물
        if self._input_yn('랜덤 장애물을 자동 배치할까요?', True):
            count = self._input_int('개수', 5)
            models_str = self._input('모델 종류 (box,cylinder)', 'box,cylinder')
            models = [m.strip() for m in models_str.split(',')]
            s_min = self._input_float('최소 크기', 0.3)
            s_max = self._input_float('최대 크기', 0.8)
            seed_str = self._input('랜덤 시드 (빈칸=랜덤)', '')
            seed = int(seed_str) if seed_str else None

            rand_cfg = {
                'count': count,
                'models': models,
                'size_range': [s_min, s_max],
            }
            if seed is not None:
                rand_cfg['seed'] = seed
            obstacles['random'] = rand_cfg

        if obstacles:
            self.config['world']['obstacles'] = obstacles

    def step_roads(self):
        self._step(5, '도로 설정')
        if not self._input_yn('도로를 추가할까요?', False):
            return

        roads = []
        while True:
            sx = self._input_float('시작 X', -10.0)
            sy = self._input_float('시작 Y', 0.0)
            ex = self._input_float('끝 X', 10.0)
            ey = self._input_float('끝 Y', 0.0)
            width = self._input_float('도로 너비 (m)', 4.0)
            lanes = self._input_int('차선 수', 2)

            roads.append({
                'start': {'x': sx, 'y': sy},
                'end': {'x': ex, 'y': ey},
                'width': width,
                'lanes': lanes,
            })
            print(f'    -> 도로 추가됨 ({sx},{sy}) -> ({ex},{ey})')

            if not self._input_yn('도로 더 추가?', False):
                break

        self.config['world']['roads'] = roads

    def step_save(self):
        self._step(6, '저장')
        name = self.config['world']['name']
        save_yaml = self._input_yn('YAML config도 저장할까요?', True)
        output_name = self._input('SDF 파일 이름', name)

        configs_dir = os.path.join(self._base_dir, '..', 'worlds', 'configs')
        generated_dir = os.path.join(self._base_dir, '..', 'worlds', 'generated')

        # YAML 저장
        if save_yaml:
            os.makedirs(configs_dir, exist_ok=True)
            yaml_path = os.path.join(configs_dir, f'{name}.yaml')
            with open(yaml_path, 'w') as f:
                yaml.dump(self.config, f, default_flow_style=False, allow_unicode=True)
            print(f'  [Saved] YAML: {yaml_path}')
        else:
            # 임시 YAML 생성
            import tempfile
            tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
            yaml.dump(self.config, tmp, default_flow_style=False, allow_unicode=True)
            tmp.close()
            yaml_path = tmp.name

        # SDF 생성
        gen = WorldGenerator(yaml_path)
        sdf_path = gen.save(output_name, generated_dir)
        print(f'  [Saved] SDF:  {sdf_path}')

        # 임시 파일 정리
        if not save_yaml:
            os.unlink(yaml_path)

        print(f'\n  Done! 월드 "{name}" 생성 완료')

    def run(self):
        try:
            self.step_world_name()
            self.step_walls()
            self.step_spawn()
            self.step_obstacles()
            self.step_roads()
            self.step_save()
        except KeyboardInterrupt:
            print('\n\n  취소됨.')
            sys.exit(0)


# ==================== CLI ====================

def main():
    parser = argparse.ArgumentParser(
        description='Generate Gazebo world from YAML config + Jinja2 templates')
    parser.add_argument('config', nargs='?', default=None,
                        help='YAML config file path')
    parser.add_argument('-i', '--interactive', action='store_true',
                        help='Interactive mode (단계별 프롬프트)')
    parser.add_argument('-o', '--output', default=None,
                        help='Output filename (default: <world_name>.sdf)')
    parser.add_argument('-d', '--output-dir', default=None,
                        help='Output directory (default: worlds/generated/)')
    args = parser.parse_args()

    # Interactive mode
    if args.interactive or args.config is None:
        builder = InteractiveBuilder()
        builder.run()
        return

    # File mode
    if not os.path.exists(args.config):
        print(f'[Error] Config file not found: {args.config}')
        sys.exit(1)

    gen = WorldGenerator(args.config)
    output_name = args.output or gen.world_name
    gen.save(output_name, args.output_dir)


if __name__ == '__main__':
    main()
