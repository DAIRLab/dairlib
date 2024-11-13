from pydrake.systems.lcm import _Serializer_
from pydrake.systems.analysis import Simulator
from pydrake.common.value import Value


class LcmLogPlayback:

    def __init__(self, log, diagram, channel_to_type_map, channel_to_port_map):
        self._lcm_log = log
        self._diagram = diagram
        self._channel_to_port_map = channel_to_port_map
        self._channel_to_type_map = channel_to_type_map
        self._channels = channel_to_type_map.keys()
        self._context = self._diagram.CreateDefaultContext()
        self._serializers = {}
        self._connected = {
            c: False for c in self._channels
        }
        for channel in self._channels:
            self._serializers[channel] = _Serializer_[channel_to_type_map[channel]]
        self._start_timestamp = 0
        self._sim = Simulator(self._diagram, self._context)
        self._finished = False

    def advance(self, t: float):
        assert self._channels

        event = self._lcm_log.read_next_event()

        while event and (event.timestamp - self._start_timestamp) < t * 1e6:
            event = self._lcm_log.read_next_event()
            if event.channel in self._channels:
                system = self._channel_to_port_map[event.channel].get_system()
                context = system.GetMyMutableContextFromRoot(self._sim.get_mutable_context())
                msg_val = self._serializers[event.channel].CreateDefaultValue()

                print(f'fixing port to {event.channel} message')

                self._serializers[event.channel].Deserialize(event.data, msg_val)
                self._channel_to_port_map[event.channel].FixValue(context, msg_val)
                self._connected[event.channel] = True

        for channel in self._channels:
            if not self._connected[channel]:
                return

        self._sim.AdvanceTo(t)

        if not event:
            self._finished = True

    def finished(self):
        return self._finished

    def reset(self):
        self._lcm_log.seek(0)
        event = self._lcm_log.read_next_event()
        self._start_timestamp = event.timestamp
