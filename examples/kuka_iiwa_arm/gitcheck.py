import subprocess
import os

os.system("git fetch")
out = subprocess.getoutput(['git status','-l'])
print(out)

if "untracked files present" in out:
    print ("Untracked Files Present. Please add and commit file changes.")
    exit()
if "Changes to be committed" in out:
    print("Uncommitted changes in local workspace.")
    exit()
if "Changes not staged for commit" in out:
    print("Unstaged file changes in workspace")
    exit()

