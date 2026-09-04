"""Plots the output of the three_d_printer jamming sweep.

Reads the file written by
examples/sampling_c3/three_d_printer/test/jamming_sweep -- draws from the demo's
real kRandomOnShell sampling strategy -- and answers two questions about whether
peak predicted EE force is a usable jam detector:

  1. Histograms: is the distribution bimodal (separable) or a smear?
  2. Force vs. C3 cost: does force carry information the cost does not already?

The metrics describe the retimed plan -- the C3 solution slowed to the EE
velocity limits -- which is the trajectory the execution path publishes and the
one a kSimImpedanceRetimedObjectCostOnly c3_cost scores.

Samples whose C3 solve fell back to "hold state, zero inputs" are excluded from
both: their recorded force understates the truth and would read as a safe
low-force sample.  Their count is reported instead.

For the spatial question -- where the hot samples sit relative to the cone, the
ramp, and the keep-out box -- use the Meshcat view (jamming_visualizer.cc)
rather than these plots.

Like its sibling diagnose_repositioning_stall.py, this is deliberately
standalone (plain numpy/matplotlib/click) so it runs under a system python3
with no bazel or pydrake build.

Usage:
    python3 plot_jamming_sweep.py <sweep_dir_or_file> [--plot-dir DIR]
"""

import math
import os.path as op
from typing import Dict, List, Optional, Tuple

import click
import matplotlib
import numpy as np

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402


# One plottable metric: its column name, its units, and its panel title.
Metric = Tuple[str, str, str]

# The force/error metrics worth plotting, in the order they are presented, with
# the units to label them by.  All are measured on the retimed plan.  The
# '_retimed'-suffixed entries are the same quantities under the names sweeps
# written before the unretimed family was dropped used for them; present_metrics
# filters the list against the file, so either layout plots.
METRICS = [
    ('max_u_norm_c3', 'N', 'C3 plan: peak |u|'),
    ('max_u_z_c3', 'N', 'C3 plan: peak |u_z|'),
    ('max_ee_contact_force_c3', 'N', 'C3 plan: peak EE contact force'),
    ('max_u_norm_pd', 'N', 'PD rollout: peak |u|'),
    ('max_u_z_pd', 'N', 'PD rollout: peak |u_z|'),
    ('max_delta_x_ee_norm', 'm', 'PD rollout: peak EE tracking error'),
    # How much of the plan sits pinned against the input bound.  The peaks above
    # saturate at that bound, so these are what keep rising once a sample wants
    # more force than the C3 solve is allowed to plan.
    ('frac_knots_u_xy_at_limit', 'fraction',
     'C3 plan: knots with u_xy at its limit'),
    ('frac_knots_u_z_at_limit', 'fraction',
     'C3 plan: knots with u_z at its limit'),
    # Object-side: whether the push accomplishes anything, which is the half of
    # "jamming" the EE-side columns above cannot see.
    ('object_travel_in_rollout', 'm', 'PD rollout: object travel'),
    ('object_rotation_in_rollout', 'rad', 'PD rollout: object rotation'),
    ('max_object_ground_force_c3', 'N', 'C3 plan: peak object-world force'),
    ('object_ground_force_per_travel', 'N/m',
     'Jam index: object-world force per meter moved'),
    # Ground truth, present only in sweeps run with --label_with_sim.
    ('sim_object_progress', 'm', 'Real sim: object travel beyond standing still'),
    ('sim_object_rotation', 'rad', 'Real sim: object rotation'),
    ('sim_ee_tracking_error', 'm', 'Real sim: peak EE tracking error'),
    ('jammed', '0/1', 'Real sim: labelled jammed'),
    # Legacy names, from sweeps that also carried the unretimed family.
    ('max_u_norm_c3_retimed', 'N', 'C3 plan: peak |u| (legacy name)'),
    ('max_u_z_c3_retimed', 'N', 'C3 plan: peak |u_z| (legacy name)'),
    ('max_ee_contact_force_c3_retimed', 'N',
     'C3 plan: peak EE contact force (legacy name)'),
    ('max_u_norm_pd_retimed', 'N', 'PD rollout: peak |u| (legacy name)'),
    ('max_u_z_pd_retimed', 'N', 'PD rollout: peak |u_z| (legacy name)'),
    ('max_delta_x_ee_norm_retimed', 'm',
     'PD rollout: peak EE tracking error (legacy name)'),
]

SWEEP_FILENAME = 'jamming_sweep_random.txt'


def read_sweep(path: str) -> Dict[str, np.ndarray]:
    """Reads the sweep file into a dict of column name -> values.

    The file carries two comment lines: a "# scene ..." record of the frozen
    object pose, then the column header.  Pick the header by looking for the
    line naming ee_x rather than assuming it comes first.
    """
    names = None
    with open(path) as handle:
        for line in handle:
            if not line.startswith('#'):
                break
            tokens = line.lstrip('#').split()
            if tokens and tokens[0] == 'ee_x':
                names = tokens
                break
    if names is None:
        raise ValueError(f'{path} has no column header comment naming ee_x')

    data = np.loadtxt(path)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] != len(names):
        raise ValueError(
            f'{path}: header names {len(names)} but data has {data.shape[1]} '
            'columns')
    return {name: data[:, i] for i, name in enumerate(names)}


def present_metrics(sweep: Dict[str, np.ndarray]) -> List[Metric]:
    """METRICS restricted to the columns this sweep file actually carries."""
    present = [entry for entry in METRICS if entry[0] in sweep]
    if not present:
        raise ValueError('Sweep file carries none of the expected metric '
                         f'columns; it has {sorted(sweep)}')
    return present


def metric_axes(metrics: List[Metric]) -> Tuple[plt.Figure, np.ndarray]:
    """A subplot grid three wide and as tall as `metrics` needs."""
    columns = min(3, len(metrics))
    rows = math.ceil(len(metrics) / columns)
    figure, axes = plt.subplots(rows, columns,
                               figsize=(6 * columns, 4.5 * rows),
                               squeeze=False)
    # Any cell past the last metric would otherwise render as empty axes.
    for axis in axes.flat[len(metrics):]:
        axis.axis('off')
    return figure, axes


def solved_mask(sweep: Dict[str, np.ndarray]) -> np.ndarray:
    """True for samples that actually measured something.

    Two ways a sample can fail to: the QP fell back to holding state with zero
    inputs, or it converged on a plan that commands nothing.  Both report a low
    force because nothing happened, and both have to stay out of the plots --
    the no-op samples are ~17% of a cone sweep and sit near zero force, so
    leaving them in drags every axis down and squashes the real range.
    """
    mask = np.ones(len(sweep['ee_x']), dtype=bool)
    if 'solve_fell_back' in sweep:
        mask &= sweep['solve_fell_back'] <= 0.5
    else:
        print('  NOTE: this sweep file predates the solve_fell_back column, so '
              'failed solves cannot be excluded.')
    if 'no_op_plan' in sweep:
        mask &= sweep['no_op_plan'] <= 0.5
    else:
        print('  NOTE: this sweep file predates the no_op_plan column, so '
              'samples whose solve produced no plan cannot be excluded.')
    return mask


def plot_histograms(sweep: Dict[str, np.ndarray], metrics: List[Metric],
                    solved: np.ndarray, plot_dir: str) -> None:
    """Per-metric histograms, split by whether the keep-out box contains them."""
    inside = sweep['inside_keep_out'] > 0.5
    figure, axes = metric_axes(metrics)
    for axis, (name, units, title) in zip(axes.flat, metrics):
        values = sweep[name]
        usable = solved & np.isfinite(values)
        if not usable.any():
            axis.set_title(f'{title}\n(not available)')
            axis.axis('off')
            continue
        bins = np.linspace(values[usable].min(), values[usable].max(), 40)
        axis.hist(values[usable & ~inside], bins=bins, alpha=0.65,
                  label='outside keep-out box')
        if (usable & inside).any():
            axis.hist(values[usable & inside], bins=bins, alpha=0.65,
                      label='inside keep-out box')
        axis.set_title(title)
        axis.set_xlabel(f'{name} ({units})')
        axis.set_ylabel('samples')
        axis.legend(fontsize='small')

    num_failed = int((~solved).sum())
    figure.suptitle(
        f'Distribution over {int(solved.sum())} solved draws from the demo\'s '
        f'sampling strategy, keep-out disabled '
        f'({num_failed} failed solves excluded)')
    figure.tight_layout()
    path = op.join(plot_dir, 'jamming_sweep_histograms.png')
    figure.savefig(path, dpi=120)
    plt.close(figure)
    print(f'  wrote {path}')


def plot_force_vs_cost(sweep: Dict[str, np.ndarray], metrics: List[Metric],
                       solved: np.ndarray, plot_dir: str) -> None:
    """Does peak force say anything the C3 cost does not already say?"""
    figure, axes = metric_axes(metrics)
    cost = sweep['c3_cost']
    for axis, (name, units, title) in zip(axes.flat, metrics):
        values = sweep[name]
        usable = solved & np.isfinite(values) & np.isfinite(cost)
        if not usable.any():
            axis.set_title(f'{title}\n(not available)')
            axis.axis('off')
            continue
        axis.scatter(cost[usable], values[usable], s=8, alpha=0.5)
        if usable.sum() > 2 and np.std(cost[usable]) > 0 \
                and np.std(values[usable]) > 0:
            correlation = np.corrcoef(cost[usable], values[usable])[0, 1]
            axis.set_title(f'{title}\nPearson r = {correlation:.2f}')
        else:
            axis.set_title(title)
        axis.set_xlabel('C3 cost')
        axis.set_ylabel(f'{name} ({units})')

    figure.suptitle('Peak predicted EE effort vs. the C3 cost of the same '
                    'sample (solved samples only)')
    figure.tight_layout()
    path = op.join(plot_dir, 'jamming_sweep_force_vs_cost.png')
    figure.savefig(path, dpi=120)
    plt.close(figure)
    print(f'  wrote {path}')


def print_summary(sweep: Dict[str, np.ndarray], metrics: List[Metric],
                  solved: np.ndarray) -> None:
    total = len(sweep['ee_x'])
    print(f'\n{total} samples '
          f'({int(sweep["acceptable"].sum())} acceptable to the sampler, '
          f'{int(sweep["inside_keep_out"].sum())} inside the keep-out box, '
          f'{int((~solved).sum())} failed solves excluded)')
    for name, units, _ in metrics:
        values = sweep[name]
        usable = values[solved & np.isfinite(values)]
        if usable.size == 0:
            print(f'  {name:32s} not available')
            continue
        print(f'  {name:32s} min {usable.min():9.4f}  median '
              f'{np.median(usable):9.4f}  p95 '
              f'{np.percentile(usable, 95):9.4f}  max {usable.max():9.4f} '
              f'{units}')


@click.command()
@click.argument('sweep_path', type=click.Path(exists=True))
@click.option('--plot-dir', default=None, type=click.Path(file_okay=False),
              help='Where to write the .png files (default: alongside the '
                   'sweep file).')
def main(sweep_path: str, plot_dir: Optional[str]) -> None:
    if op.isdir(sweep_path):
        directory, sweep_file = sweep_path, op.join(sweep_path, SWEEP_FILENAME)
    else:
        directory, sweep_file = op.dirname(sweep_path) or '.', sweep_path
    if not op.exists(sweep_file):
        raise SystemExit(f'No sweep file at {sweep_file}')
    plot_dir = plot_dir or directory

    sweep = read_sweep(sweep_file)
    metrics = present_metrics(sweep)
    solved = solved_mask(sweep)
    print_summary(sweep, metrics, solved)
    if not solved.any():
        raise SystemExit('Every solve fell back; nothing to plot.')
    plot_histograms(sweep, metrics, solved, plot_dir)
    plot_force_vs_cost(sweep, metrics, solved, plot_dir)


if __name__ == '__main__':
    main()
