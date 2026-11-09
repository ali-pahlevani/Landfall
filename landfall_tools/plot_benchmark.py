# Copyright 2026 Ali Pahlevani
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Figures of the kidnap benchmark (docs/benchmark.md) from kidnap_runner's CSV files.

    python3 -I landfall_tools/plot_benchmark.py --output-dir docs/images RESULTS.csv...

Writes each figure twice, for a light and a dark page (GitHub shows the reader's):
  benchmark_outcomes[-dark].png  what became of the placements, per scenario;
  benchmark_time[-dark].png      the share of placements localized within t, per method, in the
                                 condition given by --time-condition (default: nominal).
The numbers behind them are summarize_benchmark's tables.
"""

import argparse
import csv
import os

from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
from matplotlib.patches import Rectangle

# Chart surfaces, inks and the first three categorical slots, per theme (the validated
# reference palette); the outcome colors are status colors, the same in both themes.
THEMES = {
    'light': {
        'surface': '#fcfcfb', 'ink': '#0b0b0b', 'ink2': '#52514e', 'muted': '#898781',
        'grid': '#e1e0d9', 'axis': '#c3c2b7',
        'series': ['#2a78d6', '#eb6834', '#1baf7a'],
    },
    'dark': {
        'surface': '#1a1a19', 'ink': '#f0efec', 'ink2': '#c3c2b7', 'muted': '#898781',
        'grid': '#2c2c2a', 'axis': '#383835',
        'series': ['#3987e5', '#d95926', '#199e70'],
    },
}
# Outcome, legend name, fill, ink of a label inside the fill.
OUTCOMES = [
    ('success', 'Success', '#0ca30c', '#ffffff'),
    ('inaccurate', 'Inaccurate', '#fab219', '#0b0b0b'),
    ('miss', 'Miss (no pose)', '#898781', '#ffffff'),
    ('wrong', 'Wrong pose', '#d03b3b', '#ffffff'),
]


def read_rows(paths):
    """Group the placements by scenario, in order of appearance; leave out invalid ones."""
    groups = {}
    for path in paths:
        with open(path, newline='') as f:
            for row in csv.DictReader(f):
                if row['outcome'] == 'invalid':
                    continue
                method = row['method']
                name = row['label'] if method in ('landfall', 'amcl') else (
                    f"{row['label']}, kidnap watchdog")
                groups.setdefault((name, method), []).append(row)
    return groups


def new_figure(width, height, theme):
    """Create a figure on the theme's surface, drawn without pyplot's global state."""
    fig = Figure(figsize=(width, height), dpi=160)
    FigureCanvasAgg(fig)
    fig.patch.set_facecolor(theme['surface'])
    return fig, fig.add_subplot()


def style_axes(ax, theme):
    """Make the axes recessive: no top and right spines, hairline grid, secondary ink."""
    ax.set_facecolor(theme['surface'])
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(theme['axis'])
    ax.tick_params(colors=theme['ink2'], labelsize=9)
    ax.grid(color=theme['grid'], linewidth=0.8)
    ax.set_axisbelow(True)


def plot_outcomes(groups, theme, path):
    """Plot one 100 % stacked bar of outcomes per scenario."""
    names = list(groups)
    fig, ax = new_figure(9.0, 0.9 + 0.42 * len(names), theme)
    style_axes(ax, theme)
    ax.grid(axis='y', visible=False)
    for i, key in enumerate(names):
        rows = groups[key]
        n = len(rows)
        y = len(names) - 1 - i
        left = 0.0
        for outcome, _, fill, label_ink in OUTCOMES:
            share = 100.0 * sum(r['outcome'] == outcome for r in rows) / n
            if share <= 0.0:
                continue
            # A 2 px gap of the surface between segments.
            ax.barh(y, share, left=left, height=0.62, color=fill, edgecolor=theme['surface'],
                    linewidth=1.5)
            if share >= 8.0:
                ax.text(left + share / 2, y, f'{share:.0f} %', ha='center', va='center',
                        fontsize=8.5, color=label_ink)
            left += share
        wrong = sum(r['outcome'] == 'wrong' for r in rows)
        ax.text(101.5, y, f'{n} placements' + (f', {wrong} wrong' if wrong else ''),
                va='center', fontsize=8.5, color=theme['ink2'])
    ax.set_yticks(range(len(names)))
    ax.set_yticklabels([name for name, _ in reversed(names)], color=theme['ink'], fontsize=9)
    ax.set_xlim(0, 100)
    ax.set_xticks(range(0, 101, 20))
    ax.set_xticklabels([f'{t} %' for t in range(0, 101, 20)])
    handles = [Rectangle((0, 0), 1, 1, color=fill) for _, _, fill, _ in OUTCOMES]
    ax.legend(handles, [name for _, name, _, _ in OUTCOMES], loc='lower left',
              bbox_to_anchor=(0.0, 1.0), ncol=len(OUTCOMES), frameon=False, fontsize=9,
              labelcolor=theme['ink'], handlelength=1.2, borderaxespad=0.3)
    ax.set_title('What became of the kidnap placements', loc='left', color=theme['ink'],
                 fontsize=11, pad=26)
    fig.tight_layout()
    fig.savefig(path, facecolor=theme['surface'])


def plot_time(groups, theme, path, condition):
    """Plot the share of placements localized within t, per method, in one condition."""
    keys = [k for k in groups
            if k[0].startswith(condition) and k[1] in ('landfall', 'amcl')][:3]
    if not keys:
        return False
    fig, ax = new_figure(9.0, 4.6, theme)
    style_axes(ax, theme)
    t_max = 45.0
    ends = []
    for color, key in zip(theme['series'], keys):
        rows = groups[key]
        n = len(rows)
        times = sorted(float(r['time_to_localize_s']) for r in rows
                       if r['outcome'] == 'success' and r['time_to_localize_s'])
        xs = [0.0] + times + [t_max]
        ys = [0.0] + [100.0 * (i + 1) / n for i in range(len(times))]
        ys.append(ys[-1])
        ax.step(xs, ys, where='post', color=color, linewidth=2.0, label=key[0])
        ends.append([ys[-1], f'{key[0]}: {ys[-1]:.0f} %'])
    # Direct labels at the line ends, pushed apart where lines end close together.
    ends.sort()
    for i in range(1, len(ends)):
        ends[i][0] = max(ends[i][0], ends[i - 1][0] + 5.0)
    for y, text in ends:
        ax.text(t_max + 0.6, y, text, va='center', fontsize=8.5, color=theme['ink'])
    ax.set_xlim(0, t_max)
    ax.set_ylim(0, 100)
    ax.set_xlabel('time since the first request (s), turns included', color=theme['ink2'],
                  fontsize=9)
    ax.set_ylabel('placements localized', color=theme['ink2'], fontsize=9)
    ax.set_yticks(range(0, 101, 20))
    ax.set_yticklabels([f'{t} %' for t in range(0, 101, 20)])
    if len(keys) > 1:
        # Upper left: the curves rise from the bottom left and end on the right.
        ax.legend(loc='upper left', frameon=False, fontsize=9, labelcolor=theme['ink'])
    ax.set_title(f'How fast the robot got its pose back ({condition})', loc='left',
                 color=theme['ink'], fontsize=11)
    fig.tight_layout()
    fig.subplots_adjust(right=0.72)
    fig.savefig(path, facecolor=theme['surface'])
    return True


def main():
    """Read the result files and write both figures in both themes."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('csv', nargs='+', help="kidnap_runner's result files")
    parser.add_argument('--output-dir', default='.')
    parser.add_argument('--time-condition', default='nominal')
    args = parser.parse_args()
    groups = read_rows(args.csv)
    os.makedirs(args.output_dir, exist_ok=True)
    for name, theme in THEMES.items():
        suffix = '' if name == 'light' else '-dark'
        plot_outcomes(groups, theme,
                      os.path.join(args.output_dir, f'benchmark_outcomes{suffix}.png'))
        plot_time(groups, theme, os.path.join(args.output_dir, f'benchmark_time{suffix}.png'),
                  args.time_condition)
    print(f'wrote the figures to {args.output_dir}')


if __name__ == '__main__':
    main()
