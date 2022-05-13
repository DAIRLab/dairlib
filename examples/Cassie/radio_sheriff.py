import sys
import subprocess

from lcm import LCM
from bot_procman import sheriff
from bot_procman import sheriff_config
from bot_procman.sheriff_gtk.sheriff_gtk import find_bot_procman_deputy_cmd
from time import sleep
import dairlib


class RadioSheriff(object):
    def __init__(self, lc, cfg_filename, spawn_deputy=True):
        self.lc = lc
        self.spawed_deputy = None
        self.sheriff = sheriff.Sheriff(self.lc)

        # Load config
        self.cfg = sheriff_config.config_from_filename(cfg_filename)
        self.sheriff.load_config(self.cfg, False)

        # Spawn deputy if needed
        if spawn_deputy:
            self._spawn_deputy()

    def run_script(self, script_name):
        script = self.sheriff.get_script(script_name)
        if not script:
            print(f"script \"{script_name}\" does not exist")
            self._terminate_spawned_deputy()
            sys.exit(1)
        err = self.sheriff.execute_script(script)
        if err:
            print("Unable to run script.  Errors were detected:\n\n")
            print ("\n".join(err))
            self._terminate_spawned_deputy()
            sys.exit(1)

    def kill(self):
        self._terminate_spawned_deputy()
        print("Killing sheriff")
        sys.exit(0)

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


class RadioBasedFSM:
    def __init__(self, cfg_filename,  lc=None):
        if not lc:
            lc = LCM()
        self.states = ['init', 'pd', 'setup', 'stand', 'loco', 'loco_log']
        self.lc = lc
        self.sheriff = RadioSheriff(lc, cfg_filename)
        self.current_state = 'init'
        sleep(0.25)
        self.cassie_sub = self.lc.subscribe("CASSIE_HEARTBEAT", self.loop)

    def unsubscribe(self):
        if not self.cassie_sub:
            return
        self.lc.unsubscribe(self.cassie_sub)
        self.cassie_sub = None

    def subscribe(self, channel):
        self.unsubscribe()
        self.cassie_sub = self.lc.subscribe(channel, self.loop)

    def transition(self, sb, sc):
        print(f"sc: {sc}, sb: {sb}")
        next_state = {
            'init': {1: {0: 'pd'}.get(sc, 'init')}.get(sb, 'init'),
            'pd': {1: 'init'}.get(
                sc, {1: 'pd', 0: 'setup'}.get(sb, 'pd')),
            'setup': {1: 'init'}.get(
                sc, {-1: 'stand'}.get(sb, 'setup')),
            'stand': {1: 'init'}.get(
                sc, {0: {0: 'loco_log', -1: 'loco'}.get(sc)}.get(sb, 'stand')),
            'loco': {1: 'init'}.get(sc, 'loco'),
            'loco_log': {1: 'init'}.get(sc, 'loco_log')
        }.get(self.current_state)

        if next_state == self.current_state:
            return
        else:
            self.current_state = next_state
            self.sheriff.run_script(next_state)

    def loop(self, _, cassie_out_msg):
        radio = dairlib.lcmt_cassie_out.decode(cassie_out_msg).pelvis.radio
        sb = int(radio.channel[9])
        sc = int(radio.channel[10])
        self.transition(sb, sc)


def main():
    pmd = sys.argv[1]
    radio_sherrif = RadioBasedFSM(pmd)
    while True:
        sleep(0.5)


if __name__ == "__main__":
    main()