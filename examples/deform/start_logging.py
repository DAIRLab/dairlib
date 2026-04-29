import subprocess
import os
import os.path as op
import glob
import codecs
from datetime import date
import shutil
import sys
import yaml


# Hardcoded deform_settings path.
DAIR_PATH = op.abspath(op.join(op.dirname(__file__), "../../"))
DEFORM_SETTINGS_PATH = op.join(
    DAIR_PATH,
    "examples",
    "deform",
    "parameters",
    "deform_settings.yaml",
)


def main(log_type, folder_path):
    curr_date = date.today().strftime("%m_%d_%y")
    year = date.today().strftime("%Y")
    logdir = f"{folder_path}/{year}/{curr_date}"

    os.makedirs(logdir, exist_ok=True)
    os.chdir(logdir)

    # Determine log number.
    try:
        directories = glob.glob(op.join(logdir, "*"))
        directory_names = [op.basename(d) for d in directories if op.isdir(d)]
        last_log = max(
            [int(name) for name in directory_names if name.isdigit()]
        )
        log_num = str(last_log + 1).zfill(6)
    except:
        log_num = str(0).zfill(6)

    # Get git diff and commit tag.
    git_diff = subprocess.check_output(["git", "diff"], cwd=DAIR_PATH)
    commit_tag = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=DAIR_PATH
    )
    with open("commit_tag_%s" % log_num, "w") as f:
        f.write(str(commit_tag))
        f.write("\n\ngit diff:\n\n")
        f.write(codecs.getdecoder("unicode_escape")(git_diff)[0])

    if not op.isdir(log_num):
        os.mkdir(log_num)
    os.chdir(log_num)
    logname = f"{log_type}log-{log_num}"

    # Copy parameter files.
    shutil.copy2(DEFORM_SETTINGS_PATH, "deform_settings.yaml")
    with open(DEFORM_SETTINGS_PATH) as f:
        deform_settings = yaml.load(f, Loader=yaml.FullLoader)

    for filename, filepath in deform_settings.items():
        if op.exists(op.join(DAIR_PATH, filepath)):
            shutil.copy2(op.join(DAIR_PATH, filepath), filename)

    # Copy object model files.
    # with open(sim_params_path) as f:
    #     sim_params = yaml.load(f, Loader=yaml.FullLoader)

    # Begin logging.
    subprocess.run(["/opt/lcm/1.4.0/bin/lcm-logger", "-f", logname])


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(f"Usage: python {sys.argv[0]} <log_type> <folder_path>")
        sys.exit(1)
    log_type = sys.argv[1]
    folder_path = sys.argv[2]
    main(log_type, folder_path)
