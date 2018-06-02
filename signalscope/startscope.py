import subprocess

_procs = []

def startScope():
    global _procs
    proc = subprocess.Popen(['signal-scope'])
    _procs.append(proc)
    return proc