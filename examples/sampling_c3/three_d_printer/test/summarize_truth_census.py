#!/usr/bin/env python3
"""Summarizes sample_cost_repeatability --truth_census_times CSVs.

Each row is one candidate sample at one fixture under one plan variant:  its
cost under each model, and what its plan really did when replayed through the
compliant sim's physics (the T columns).  Per fixture and variant this asks
whether each cost model ranks the candidates the way the truth does:

  rho     Spearman correlation between the model's cost and T's cost.
  regret  how much the model's pick loses against the best candidate, as a
          share of what the best candidate gains over doing nothing:
          (T[pick] - T[best]) / (T[do nothing] - T[best]).  0 is the best pick,
          1 is no better than standing still, above 1 is worse than that.
  ties    share of candidates whose model cost equals the most common value,
          i.e. how much of the field the model cannot tell apart.
  low@3   share of the model's three cheapest candidates that sit low behind
          the object's base (body x < -4 mm, within 12 mm of its centre
          height), the push start that works at the goal-3 step edges.
  bent@10 share of fixtures where the model's pick bends the finger 10 mm or
          more in T, i.e. picks a push that jams.

and, model-free, whether any candidate works (true progress >= --works_mm or
--works_deg toward the goal) and how far the best one gets.  Fixtures are then
pooled by goal (2 toe, 3 mid-slope, 4 seated); goal 3 is split into the stall
fixtures and the ones where a low push worked, by --productive.

Usage:
  python3 summarize_truth_census.py truth_*.csv [--productive 315.4,241.66,...]
"""

import csv
import glob
import sys
from collections import defaultdict

import click
import numpy as np
from scipy.stats import spearmanr

MODELS = ['cost_v0', 'V4_cost', 'V4f_cost', 'V4k_cost', 'V4fk_cost',
          'V5_cost', 'V5c_cost',
          'V5k_cost', 'V5p_cost', 'V4fp_cost', 'V5fp_cost', 'T0_cost']
NAMES = {'cost_v0': "variant's LCS cost", 'V4_cost': 'V4 drake 1ms',
         'V4f_cost': 'V4f drake 4ms', 'V4k_cost': 'V4k +spring finger',
         'V4fk_cost': 'V4fk 4ms +spring', 'V5_cost': 'V5 drake+settle',
         'V5c_cost': 'V5c +clean pose', 'V5k_cost': 'V5k +spring finger',
         'V5p_cost': 'V5p +projected', 'V4fp_cost': 'V4fp 4ms projected',
         'V5fp_cost': 'V5fp 4ms+.25 proj',
         'T0_cost': 'T0 truth, no lag'}
LOW_BODY_X = -0.004
LOW_DZ_MM = 12.0
CONTACT_HULL_MM = 10.0  # EE radius:  a start inside the object is not replayable


def is_low(row):
  return (float(row['body_x']) < LOW_BODY_X and
          abs(float(row['dz_mm'])) < LOW_DZ_MM)


def fixture_stats(rows, model, works_mm, works_deg):
  truth = np.array([float(r['T_cost']) for r in rows])
  cost = np.array([float(r[model]) for r in rows])
  dn = float(rows[0]['T_cost_dn'])
  best = truth.min()
  pick = int(np.argmin(cost))
  gain = dn - best
  rho = spearmanr(cost, truth).correlation if len(set(cost)) > 1 else np.nan
  values, counts = np.unique(np.round(cost, 6), return_counts=True)
  top3 = np.argsort(cost, kind='stable')[:3]
  return dict(
      rho=rho,
      regret=(truth[pick] - best) / gain if gain > 1e-9 else np.nan,
      ties=counts.max() / len(cost),
      low3=np.mean([is_low(rows[i]) for i in top3]),
      pick_prog=float(rows[pick]['T_prog_mm']),
      pick_bend=float(rows[pick]['T_defl_mm']))


@click.command()
@click.argument('paths', nargs=-1, required=True)
@click.option('--productive', default='315.4,228.56,241.66,310.32,70.51,'
              '231.02,144.77,357.18',
              help='Comma-separated goal-3 fixture times where a low push '
              'worked in the log; the rest of goal 3 counts as stall.')
@click.option('--works_mm', default=5.0, show_default=True)
@click.option('--works_deg', default=5.0, show_default=True)
@click.option('--per_fixture', is_flag=True, help='Also print every fixture.')
def main(paths, productive, works_mm, works_deg, per_fixture):
  productive_times = [float(t) for t in productive.split(',') if t]
  files = [f for p in paths for f in sorted(glob.glob(p))]
  fixtures = defaultdict(list)
  for path in files:
    for row in csv.DictReader(open(path)):
      if int(row['idx']) == 0 and float(row['hull_mm']) < CONTACT_HULL_MM:
        continue
      fixtures[(path, float(row['t']), row['variant'])].append(row)

  def group_of(t, goal):
    if goal == 3:
      near = any(abs(t - p) < 0.1 for p in productive_times)
      return 'goal 3 productive' if near else 'goal 3 stall'
    return {2: 'goal 2 toe', 4: 'goal 4'}.get(goal, f'goal {goal}')

  pooled = defaultdict(lambda: defaultdict(list))
  for (path, t, variant), rows in sorted(fixtures.items()):
    if len(rows) < 4:
      continue
    goal = int(rows[0]['goal'])
    group = (group_of(t, goal), variant)
    prog = np.array([float(r['T_prog_mm']) for r in rows])
    prog_deg = np.array([float(r['T_prog_deg']) for r in rows])
    works = (prog >= works_mm) | (prog_deg >= works_deg)
    low = np.array([is_low(r) for r in rows])
    pooled[group]['works'].append(works.any())
    pooled[group]['works_share'].append(works.mean())
    pooled[group]['best_prog'].append(prog.max())
    pooled[group]['low_works'].append(works[low].mean() if low.any() else
                                      np.nan)
    pooled[group]['other_works'].append(works[~low].mean() if (~low).any()
                                        else np.nan)
    pooled[group]['n'].append(len(rows))
    line = (f'{path.split("/")[-1]:14s} t={t:7.2f} {variant:14s} '
            f'n={len(rows):3d} works {works.mean():.2f} best '
            f'{prog.max():+6.1f} mm')
    for model in MODELS:
      if model not in rows[0]:
        continue
      st = fixture_stats(rows, model, works_mm, works_deg)
      for key, value in st.items():
        pooled[group][(model, key)].append(value)
      line += (f' | {model.split("_")[0]} rho {st["rho"]:+.2f} '
               f'reg {st["regret"]:.2f}')
    if per_fixture:
      print(line)

  for (group, variant), stats in sorted(pooled.items()):
    n_fix = len(stats['n'])
    print(f'\n== {group}, plans: {variant} ({n_fix} fixtures, '
          f'{np.median(stats["n"]):.0f} candidates each)')
    print(f'   a candidate works at {np.mean(stats["works"]):.0%} of fixtures;'
          f' share that work p50 {np.median(stats["works_share"]):.2f};'
          f' best true progress p50 {np.median(stats["best_prog"]):+.1f} mm;'
          f' works | low start {np.nanmean(stats["low_works"]):.2f} vs '
          f'other {np.nanmean(stats["other_works"]):.2f}')
    print(f'   {"model":18s} {"rho p50":>8s} {"regret p50":>11s} '
          f'{"regret<=.3":>11s} {"ties":>6s} {"low@3":>6s} '
          f'{"pick prog":>10s} {"bent@10":>8s}')
    for model in MODELS:
      if (model, 'rho') not in stats:
        continue
      rho = np.array(stats[(model, 'rho')], dtype=float)
      reg = np.array(stats[(model, 'regret')], dtype=float)
      print(f'   {NAMES[model]:18s} {np.nanmedian(rho):+8.2f} '
            f'{np.nanmedian(reg):11.2f} {np.nanmean(reg <= 0.3):11.2f} '
            f'{np.mean(stats[(model, "ties")]):6.2f} '
            f'{np.mean(stats[(model, "low3")]):6.2f} '
            f'{np.median(stats[(model, "pick_prog")]):+9.1f}mm '
            f'{np.mean(np.array(stats[(model, "pick_bend")]) >= 10):8.2f}')


if __name__ == '__main__':
  sys.exit(main())
