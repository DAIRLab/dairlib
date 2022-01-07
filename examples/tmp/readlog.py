import sys
import lcm

if len(sys.argv) < 2:
    sys.stderr.write("usage: read-log <logfile>\n")
    sys.exit(1)

log = lcm.EventLog(sys.argv[1], "r")

print("starting")
for event in log:
    print(event)