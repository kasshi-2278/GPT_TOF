"""Render the saved three-lap speed/distance validation trace."""
import csv
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter, FuncAnimation
from matplotlib.patches import Polygon, Circle

from config import VehicleConfig
from geometry import local_to_world, direction, rectangle
from world import World

ROOT = Path(__file__).resolve().parents[2]
TRACE = ROOT / 'optimization/three_laps/logs/pace_distance_fast_short.csv'
RESULTS = ROOT / 'optimization/speed_distance/validation.json'
OUTPUT = ROOT / 'videos/speed_distance_score.mp4'
FPS = 12
SIM_SECONDS_PER_FRAME = 1.0
SENSORS = ('Fr', 'FrLh', 'RrLh', 'FrRh', 'RrRh')
COLORS = ('#52d6ff', '#ffbc42', '#ff5f77', '#b88cff', '#76e3a2')


def load_trace():
    with TRACE.open(encoding='utf-8-sig', newline='') as stream:
        rows = list(csv.DictReader(stream))
    for row in rows:
        for key in ('time_s', 'x_cm', 'y_cm', 'heading_deg', 'speed_cm_s', 'steer_deg', 'Handle'):
            row[key] = float(row[key])
        for name in SENSORS:
            row[name] = float(row[name])
    return rows


def main():
    rows = load_trace()
    result = next(r for r in json.loads(RESULTS.read_text(encoding='utf-8'))
                  if r['name'] == 'pace_distance_fast_short')
    world = World.load()
    cfg = VehicleConfig()
    specs = cfg.sensor_specs()
    extent = world.extents

    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10,
                         'figure.facecolor': '#0c1422', 'axes.facecolor': '#101c2c',
                         'text.color': '#e8f0fa', 'axes.labelcolor': '#b5c7db',
                         'xtick.color': '#8da2ba', 'ytick.color': '#8da2ba'})
    fig = plt.figure(figsize=(16, 9), dpi=90, constrained_layout=True)
    grid = fig.add_gridspec(1, 2, width_ratios=(1.65, 1))
    ax = fig.add_subplot(grid[0, 0])
    side = fig.add_subplot(grid[0, 1])

    ax.set_title('COURSE MAP  |  ultrasonic echoes and chosen direction', loc='left', pad=12,
                 fontsize=15, fontweight='bold')
    ax.set_xlim(extent[0], extent[2]); ax.set_ylim(extent[1], extent[3]); ax.set_aspect('equal')
    ax.grid(True, color='#26384d', linewidth=.45)
    ax.set_xlabel('X (cm)'); ax.set_ylabel('Y (cm)')
    for wall in world.data.get('walls', []):
        if wall.get('solid', True):
            a, b = wall['a'], wall['b']
            ax.plot([a[0], b[0]], [a[1], b[1]], color='#65788e',
                    linewidth=max(1.0, wall['thickness_cm'] * .45), solid_capstyle='round', zorder=1)
    for post in world.data.get('posts', []):
        if post.get('solid', True):
            ax.add_patch(Circle(post['center'], post['radius_cm'], color='#65788e', zorder=1))

    side.set_xlim(0, 1); side.set_ylim(0, 1); side.axis('off')
    side.text(.02, .975, 'SPEED + DISTANCE SCORE', fontsize=16, fontweight='bold', va='top')
    side.text(.02, .925, 'Fast and short candidate • 3-lap replay', color='#8fa8c4', fontsize=10, va='top')

    # Sensor bars and live steering target.
    side.text(.02, .855, 'HC-SR04 readings (cm)', fontsize=12, fontweight='bold', va='top')
    bar_y = [.80, .745, .69, .635, .58]
    for name, color, y in zip(SENSORS, COLORS, bar_y):
        side.text(.03, y, name, color=color, fontsize=10, va='center')
        side.add_patch(plt.Rectangle((.24, y-.012), .48, .024, color='#26384d', transform=side.transAxes))
    steer_text = side.text(.03, .505, '', fontsize=10, color='#dbe7f5', va='center')

    side.text(.03, .445, 'RUN PROGRESS', fontsize=11, fontweight='bold', color='#8fa8c4', va='top')
    time_text = side.text(.03, .405, '', fontsize=13, va='top')
    distance_text = side.text(.03, .365, '', fontsize=13, va='top')
    lap_text = side.text(.03, .325, '', fontsize=13, va='top')

    side.plot([.03, .97], [.275, .275], color='#33465d', linewidth=1)
    side.text(.03, .25, 'VALIDATION COMPARISON', fontsize=11, fontweight='bold',
              color='#8fa8c4', va='top')
    side.text(.03, .205, 'Run                  time       distance    pace + path',
              fontsize=8.5, color='#7f95af', va='top', family='monospace')
    comp = [
        ('Baseline', '401.321s', '10290.3cm', '219.648'),
        ('Fast + short', '395.417s', '10255.3cm', '229.057'),
        ('Shortest', '400.638s', '10246.8cm', '224.680'),
    ]
    for i, (name, t, d, points) in enumerate(comp):
        y = .168 - i * .035
        color = '#76e3a2' if name == 'Fast + short' else '#c9d6e5'
        side.text(.03, y, f'{name:<17}{t:>9}  {d:>10}   {points:>7}',
                  fontsize=8.5, family='monospace', color=color, va='top')
    side.text(.03, .045, 'Bonuses require a clean 3-lap finish.', fontsize=9,
              color='#8fa8c4', va='bottom')

    trail, = ax.plot([], [], color='#41e0b4', linewidth=2.0, alpha=.88, zorder=3)
    body = Polygon([[0, 0]]*4, closed=True, facecolor='#f4f7fb', edgecolor='#07111d',
                   linewidth=1.2, zorder=5)
    ax.add_patch(body)
    sensor_lines = [ax.plot([], [], color=color, linewidth=1.2, alpha=.8, zorder=4)[0]
                    for color in COLORS]
    chosen_line, = ax.plot([], [], color='#ffffff', linewidth=2.0, alpha=.95,
                           linestyle='--', zorder=4)
    point, = ax.plot([], [], marker='o', color='#fff', markersize=3, zorder=6)
    value_texts = [side.text(.75, y, '', fontsize=9, color=color, va='center', ha='right')
                   for y, color in zip(bar_y, COLORS)]

    frame_indices = [i for i, row in enumerate(rows)
                     if row['time_s'] >= rows[0]['time_s'] and
                     (i == 0 or int(row['time_s'] / SIM_SECONDS_PER_FRAME) >
                      int(rows[i-1]['time_s'] / SIM_SECONDS_PER_FRAME))]
    if frame_indices[-1] != len(rows)-1:
        frame_indices.append(len(rows)-1)
    cumulative = [0.0]
    for a, b in zip(rows, rows[1:]):
        cumulative.append(cumulative[-1] + math.hypot(b['x_cm']-a['x_cm'], b['y_cm']-a['y_cm']))

    def draw(frame):
        i = frame_indices[frame]
        row = rows[i]
        x, y, heading = row['x_cm'], row['y_cm'], row['heading_deg']
        trail.set_data([r['x_cm'] for r in rows[:i+1]], [r['y_cm'] for r in rows[:i+1]])
        body.set_xy(rectangle(x, y, heading, cfg.length_cm, cfg.width_cm))
        point.set_data([x], [y])
        for name, (forward, left, angle), line, distance in zip(
                SENSORS, specs.values(), sensor_lines, (row[n] for n in SENSORS)):
            origin = local_to_world(x, y, heading, forward, left)
            dx, dy = direction(heading + angle)
            line.set_data([origin[0], origin[0] + distance*dx],
                          [origin[1], origin[1] + distance*dy])
        desired_heading = heading - row['Handle'] * cfg.max_steer_deg / 100
        dx, dy = direction(desired_heading)
        chosen_line.set_data([x, x + 50*dx], [y, y + 50*dy])
        for name, distance, color, value_text, bar_y0 in zip(SENSORS,
                (row[n] for n in SENSORS), COLORS, value_texts, bar_y):
            value_text.set_text(f'{distance:5.1f}')
            # Width is updated in axes coordinates; clamp at the sensor range.
            patches = [p for p in side.patches if p.get_gid() == name]
            if patches:
                patches[0].set_width(.48 * min(1, distance / cfg.sensor_range_cm))
        steer_text.set_text(f"Steering target: {'LEFT' if row['Handle'] > 2 else 'RIGHT' if row['Handle'] < -2 else 'STRAIGHT'}  ({row['Handle']:+.1f}%)")
        time_text.set_text(f"Elapsed: {row['time_s']:6.1f} s / {result['total_time_s']:.1f} s")
        distance_text.set_text(f"Distance: {cumulative[i]:7.0f} / {result['travel_distance_cm']:.0f} cm")
        lap = min(3, int(float(row.get('lap_count') or 0)) + 1)
        lap_text.set_text(f'Lap: {lap} / 3     Speed: {row["speed_cm_s"]:.1f} cm/s')
        return [trail, body, point, *sensor_lines, chosen_line, steer_text,
                time_text, distance_text, lap_text, *value_texts]

    # Create colored sensor bars after layout is initialized.
    for name, color, y in zip(SENSORS, COLORS, bar_y):
        rect = plt.Rectangle((.24, y-.012), .48, .024, color=color, alpha=.75,
                             transform=side.transAxes, zorder=2)
        rect.set_gid(name)
        side.add_patch(rect)
    writer = FFMpegWriter(fps=FPS, codec='libx264', bitrate=3500,
                          extra_args=['-pix_fmt', 'yuv420p', '-movflags', '+faststart'])
    animation = FuncAnimation(fig, draw, frames=len(frame_indices), interval=1000/FPS,
                              blit=False, repeat=False)
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    animation.save(OUTPUT, writer=writer, dpi=90)
    plt.close(fig)
    print(json.dumps({'video': str(OUTPUT.relative_to(ROOT)), 'frames': len(frame_indices),
                      'duration_s': len(frame_indices)/FPS, 'source': TRACE.name,
                      'sha256_result': result['sha256']}, ensure_ascii=False))


if __name__ == '__main__':
    main()
