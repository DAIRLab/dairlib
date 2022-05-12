import sys
import subprocess

from lcm import LCM
from bot_procman import sheriff
from bot_procman import sheriff_config
from bot_procman.sheriff_gtk.sheriff_gtk import find_bot_procman_deputy_cmd


class RadioSheriff():
    def __init__(self, lc, cfg_filename, spawn_deputy=False):
        self.lc = lc
        self.spawed_deputy = None
        self.sheriff = sheriff.Sheriff(self.lc)

        # Load config
        self.cfg = sheriff_config.config_from_filename(cfg_filename)
        self.sheriff.load_config(self.cfg, False)

        # Spawn deputy if needed
        if spawn_deputy:
            self._spawn_deputy()

    def run_script(self, script):
        err = self.sheriff.execute_script(script)
        if err:
            print("ERROR:")
            for e in err:
                print(f"{e}")

    def _terminate_spawned_deputy(self):
        if not self.spawed_deputy:
            return
        print("Terminating local deputy..")
        self.spawed_deputy.terminate()
        self.spawed_deputy = None

    def _spawn_deputy(self):
        bot_procman_deputy_cmd = find_bot_procman_deputy_cmd()
        args = [bot_procman_deputy_cmd, "-n", "localhost"]
        self.spawned_deputy = subprocess.Popen(args)


def main():
    lc = LCM()
    radio_sheriff = RadioSheriff(lc, sys.argv[1])


if __name__ == "__main__":
    main()