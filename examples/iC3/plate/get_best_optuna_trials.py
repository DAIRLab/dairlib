import optuna

STORAGE_PATH = "sqlite:///examples/iC3/plate/optuna/optuna_plate_hydroelastic.db"
STUDY_NAME = "optuna_plate_hydroelastic"


TOP_N = 5  # Change this to however many you want

study = optuna.load_study(study_name=STUDY_NAME, storage=STORAGE_PATH)
print(len(study.trials))

# Filter to only completed trials and sort by value (ascending since direction="minimize")
completed_trials = [t for t in study.trials if t.state == optuna.trial.TrialState.COMPLETE]
top_trials = sorted(completed_trials, key=lambda t: t.value)[:TOP_N]

print(f"{'Rank':<6} {'Trial #':<10} {'Value':<20} Parameters")
print("=" * 80)
for rank, trial in enumerate(top_trials, start=1):
    print(f"{rank:<6} {trial.number:<10} {trial.value:<20.6f}")
    for key, value in trial.params.items():
        print(f"       {key}: {value}")
    print("-" * 80)

# python3 examples/iC3/plate/get_best_optuna_trials.py