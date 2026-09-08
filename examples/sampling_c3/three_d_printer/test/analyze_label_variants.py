"""Reports how well the fast approximate jam labeller agrees with the ground
truth, and what each of its knobs cost to get there.

Reads a jamming_sweep_random.txt written with --label_with_sim --label_variants,
which carries the ground truth columns, one travel/verdict pair per fast
configuration, and a "# label_cost" comment per configuration.

The four things worth knowing about an approximation, in the order they decide
whether to trust it:

  1. Agreement, split into false jams and missed jams.  They are not
     interchangeable in a controller: a false jam throws away a workable sample,
     a missed jam walks into the wedge.

  2. Whether a disagreement is retunable.  A knob that shifts travel is fixable
     by re-fitting the threshold and reports a much better number under
     "re-fit"; a knob that scrambles the ranking of samples is not fixable at
     any threshold, and shows up as a poor rank correlation.  The gap between
     the "as-set" and "re-fit" columns is the whole story.

  3. Where the disagreements sit.  The population is bimodal -- most samples
     never move at all -- so a variant that only errs in the millimetre band
     around the threshold is behaving very differently from one that errs on
     samples that plainly moved.

  4. What it cost, against the ground truth's own seconds per sample.

Run standalone (no bazel):
    python3 analyze_label_variants.py /path/to/jamming_sweep_random.txt
"""

import sys

import numpy as np

# Below this the ground truth calls a push jammed; the fast labeller thresholds
# raw travel instead, so its own threshold is larger.  Only used to describe the
# ground truth column, never applied to a fast one.
GROUND_TRUTH_THRESHOLD = 1e-3


def read_sweep(path):
    """Returns (columns, data, label_costs) from a sweep file."""
    columns = None
    costs = {}
    slugs = {}
    with open(path) as handle:
        for line in handle:
            if not line.startswith("#"):
                continue
            fields = line.lstrip("#").split()
            if not fields:
                continue
            if fields[0] == "label_cost":
                # "# label_cost <name> seconds_per_sample <t> [config <slug>]"
                costs[fields[1]] = float(fields[3])
                if len(fields) > 5 and fields[4] == "config":
                    slugs[fields[1]] = fields[5]
            elif fields[0] == "label_threads":
                costs["__threads__"] = float(fields[1])
            elif "sim_object_travel" in fields:
                columns = fields
    if columns is None:
        raise SystemExit(
            f"{path} has no ground truth columns; rerun the sweep with "
            "--label_with_sim --label_variants")
    data = np.loadtxt(path)
    return columns, data, costs, slugs


def variant_names(columns):
    """The fast configurations present, in file order."""
    return [c[len("fast_"):-len("_travel")] for c in columns
            if c.startswith("fast_") and c.endswith("_travel")]


def best_threshold(travel, truth):
    """The threshold on `travel` that best reproduces `truth`, and its error
    count.  Scanning the observed values is exact -- the optimum always sits at
    one of them -- and cheap enough at sweep sizes."""
    order = np.argsort(travel)
    # Sweeping the split point down the sorted travels, errors = (jammed above
    # the split) + (not-jammed below it).
    jammed_sorted = truth[order]
    jammed_above = np.cumsum(jammed_sorted[::-1])[::-1]
    jammed_above = np.append(jammed_above, 0)
    not_jammed_below = np.cumsum(~jammed_sorted)
    not_jammed_below = np.append(0, not_jammed_below)
    errors = jammed_above + not_jammed_below
    split = int(np.argmin(errors))
    travels = np.append(travel[order], np.inf)
    return float(travels[split]), int(errors[split])


def spearman(a, b):
    """Rank correlation, without a scipy dependency."""
    def rank(values):
        order = np.argsort(values)
        ranks = np.empty(len(values), float)
        ranks[order] = np.arange(len(values))
        return ranks
    ra, rb = rank(a), rank(b)
    ra -= ra.mean()
    rb -= rb.mean()
    denominator = np.sqrt((ra ** 2).sum() * (rb ** 2).sum())
    return float((ra * rb).sum() / denominator) if denominator else float("nan")


# Travel bands the disagreements are reported over.  The first is the settle
# floor -- samples that never moved -- and the middle ones bracket both
# thresholds, which is where an approximation is expected to be wrong.
BANDS = [(0.0, 1.5e-4), (1.5e-4, 1e-3), (1e-3, 2e-3), (2e-3, 5e-3),
         (5e-3, np.inf)]


def main(path):
    columns, data, costs, slugs = read_sweep(path)
    column = {name: data[:, i] for i, name in enumerate(columns)}

    # The ground truth withholds a verdict on plans that commanded nothing, so
    # those samples cannot score an approximation of it either.
    real = column["no_op_plan"] < 0.5
    truth = column["jammed"][real] > 0.5
    gt_travel = column["sim_object_travel"][real]
    n = int(real.sum())

    print(f"{path}")
    print(f"{n} real plans of {len(data)} samples, "
          f"{truth.sum()} ({truth.mean():.1%}) jammed by the ground truth")
    gt_cost = costs.get("ground_truth")
    threads = costs.get("__threads__", 1.0)
    if gt_cost:
        print(f"ground truth cost {gt_cost * 1e3:.1f} ms/sample, serial")
    print(f"variants timed across {threads:.0f} threads, so \"vs GT\" is a "
          "wall-clock ratio that\nincludes the parallelism; \"vs ref\" is "
          "measured at the same thread count and is\nwhat the knob itself "
          "bought.")
    print()

    print("AGREEMENT  (as-set = the variant's own threshold; re-fit = the best "
          "threshold for it)")
    header = (f"{'variant':>12s} {'ms/samp':>8s} {'vs GT':>7s} {'vs ref':>7s} "
              f"{'as-set':>7s} {'false':>6s} {'missed':>7s} "
              f"{'re-fit':>7s} {'at thr':>8s} {'rank r':>7s}")
    print(header)
    print("-" * len(header))
    reference_cost = costs.get("reference")
    for name in variant_names(columns):
        travel = column[f"fast_{name}_travel"][real]
        verdict = column[f"fast_{name}_jammed"][real] > 0.5
        false_jam = int((verdict & ~truth).sum())
        missed_jam = int((~verdict & truth).sum())
        as_set = (false_jam + missed_jam) / n
        # A variant that stops as soon as the verdict is decided reports
        # travel truncated at its threshold.  Its verdict is still exact -- the
        # early exit cannot change it -- but re-fitting a threshold to a
        # truncated measurement, or ranking samples by it, is meaningless.
        truncated = "_exit" in slugs.get(name, "")
        if truncated:
            refit, at_threshold, rank = "      -", "       -", "      -"
        else:
            threshold, errors = best_threshold(travel, truth)
            refit = f"{errors / n:6.1%}"
            at_threshold = f"{threshold * 1e3:6.2f}mm"
            rank = f"{spearman(travel, gt_travel):6.3f}"
        cost = costs.get(name, float("nan"))
        speedup = gt_cost / cost if gt_cost and cost else float("nan")
        vs_reference = (reference_cost / cost if reference_cost and cost
                        else float("nan"))
        print(f"{name:>12s} {cost * 1e3:8.1f} {speedup:6.0f}x "
              f"{vs_reference:6.1f}x {as_set:6.1%} {false_jam:6d} "
              f"{missed_jam:7d} {refit:>7s} {at_threshold:>8s} {rank:>7s}")

    print()
    print("WHERE THE DISAGREEMENTS SIT  (rows are ground truth travel, "
          "values are the variant's error rate in that band)")
    band_labels = [f"{lo * 1e3:g}-{hi * 1e3:g}mm" if np.isfinite(hi)
                   else f">{lo * 1e3:g}mm" for lo, hi in BANDS]
    print(f"{'variant':>12s} " + " ".join(f"{b:>10s}" for b in band_labels))
    counts = [int(((gt_travel >= lo) & (gt_travel < hi)).sum())
              for lo, hi in BANDS]
    for name in variant_names(columns):
        verdict = column[f"fast_{name}_jammed"][real] > 0.5
        wrong = verdict != truth
        cells = []
        for (lo, hi) in BANDS:
            in_band = (gt_travel >= lo) & (gt_travel < hi)
            cells.append(f"{wrong[in_band].mean():9.1%}" if in_band.any()
                         else "        -")
        print(f"{name:>12s} " + " ".join(f"{c:>10s}" for c in cells))
    print(f"{'(samples)':>12s} " + " ".join(f"{c:10d}" for c in counts))
    print(f"{'':>12s} " + " ".join(f"{'':>10s}" for _ in counts))

    print()
    print("Read the table this way: a variant whose re-fit column is much "
          "better than\nits as-set column only needs its threshold moved.  One "
          "whose rank r is well\nbelow 1.0 has reordered the samples and no "
          "threshold will recover it.\nA \"-\" means the variant exits as soon "
          "as the verdict is decided, so its\ntravel is truncated and cannot "
          "be re-thresholded or ranked -- its verdict\ncolumns are still "
          "exact.")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit(__doc__)
    main(sys.argv[1])
