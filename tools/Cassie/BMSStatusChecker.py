import BMS
from time import sleep
import os

os.system('mode con: cols=150 lines=20')

BMS = BMS.BMS()
# Try to passively read from BMS
data_in_bytes = BMS.passive_read()
# If passive read fails, try active read
if data_in_bytes is None:
    generation = '***' + '{:^84}'.format('First Generation Battery Pack Connected') + '***\n'
    BMS.write_to_register(BMS.CHANNELS, 0x0fff1f00, 4)
    data_in_bytes = BMS.read_from_register(2, 1)
else:
    generation = '***' + '{:^84}'.format('Second Generation Battery Pack Connected') + '***\n'

data_in_int = BMS.bytearray_to_intarray(data_in_bytes, 2, '>H')
V_int = data_in_int[:12]
I_int = data_in_int[12]
T_int = data_in_int[13:]

V_cell = [(5/65535) * datum for datum in V_int]
I_A = ((5/65535) * I_int - 2.65) / (-14.1/1000)
T_degC = [BMS.volts_to_temp(temp * (5/65535)) for temp in T_int]
V_sum = sum(V_cell)
V_min = min(V_cell)
V_max = max(V_cell)
V_mean = V_sum/len(V_int)
SOC = BMS.soc_estimate(V_min)

variance = 0
for meas in V_cell:
    variance += (meas - V_mean)**2
variance /= len(V_cell)
std = variance**.5

max_diff = V_max - V_min

std_perc = (std/V_mean)*100
maxdiff_perc = (max_diff/V_max)*100

title_array = []
for i in range(len(V_cell)):
    title_array.append(str(i))

print('')

#Display state of charge information
pad = '{:25}'.format('')
border = ['*' for i in range(90)]
pack_v_out = '{:>12}'.format('Pack Voltage') + ':  ' + '{:10.6}'.format(str(V_sum))
soc_out = '{:>19}'.format('State of Charge (%)') + ':  ' + '{:10.6}'.format(str(SOC))

print(pad + ''.join(border) + '\n' +
      pad + generation +
      pad + ''.join(border) + '\n' +
      pad + '***  ' + '{:^37}'.format(pack_v_out) + '  **  ' + '{:^37}'.format(soc_out) + '  ***' + '\n' +
      pad + ''.join(border) + '\n')
######

BMS.display_array('Cell', title_array)
BMS.display_array('Cell Voltages (raw)', V_int)
BMS.display_array('Cell Voltages (V)', V_cell)

print('')

print(' {:>21}'.format('Minimum Voltage')    + ':  ' + '{:10.6}'.format(str(V_min)) +
      ' {:>21}'.format('Maximum Voltage')    + ':  ' + '{:10.6}'.format(str(V_max)) +
      ' {:>21}'.format('Average Voltage')    + ':  ' + '{:10.6}'.format(str(V_mean)) +
      ' {:>21}'.format('Standard Deviation') + ':  ' + '{:10.6}'.format(str(std)) + '\n'
      ' {:>56}'.format('Maximum Imbalance (%)')          + ':  ' + '{:10.6}'.format(str(maxdiff_perc)) +
      ' {:>21}'.format('Imbalance STD (%)') + ':  ' + '{:10.6}'.format(str(std_perc)) + '\n')

title_array = []
for i in range(len(T_int)):
    title_array.append(str(i))

BMS.display_array('Thermistor', title_array)
BMS.display_array('Temperature (raw)', T_int)
BMS.display_array('Temperature (deg. C)', T_degC)

print('')
print('{:>21}'.format('Current (A)') + ':   ' + '{:10.6}'.format(str(I_A)))


BMS.ser.close()

while True:
    sleep(1)
