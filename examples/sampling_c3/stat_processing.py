"""Process statistic numbers."""

import numpy as np

DO_DOUBLE_EXPS = False
DO_TRIPLE_EXPS = True

assert DO_DOUBLE_EXPS != DO_TRIPLE_EXPS

if DO_DOUBLE_EXPS:
  NUMS_BY_EXP_LOOSE = {}
  NUMS_BY_EXP_LOOSE['Lotion & Letter R'] = """[ 55.711029  67.527142  85.357802  50.882889  61.046328  46.271816
  171.485737  17.905184  85.839803 100.567408]"""
  NUMS_BY_EXP_LOOSE['Baby toy & Letter E'] = """[20.952914 56.332871 69.386685 65.08423  48.31183  54.434664 51.641837
  71.85777  41.287914 62.042571]"""
  NUMS_BY_EXP_LOOSE['Letter B & Letter 3'] = """[ 17.98141   58.477661  62.782386  35.559907  46.415925  36.314387
    51.264677  48.18212   64.661562 114.738278]"""
  NUMS_BY_EXP_LOOSE['Chicken Broth & Expo Box'] = """[ 60.348074  63.495311  54.419024  72.459633  64.002464 158.161314
    76.859204  53.767588  80.355094  75.546018]"""
  NUMS_BY_EXP_LOOSE['Chicken Broth & Wood Block'] = """[48.138262 46.601177 44.249876 68.581899 64.679639 49.663    76.122953
  74.146961 41.817874 32.243262]"""
  NUMS_BY_EXP_LOOSE['Clamp & Letter I'] = """[49.59296  61.277866 37.107579 80.737476 28.516114 31.730907 31.556964
  44.825115 29.580757 46.525174]"""
  NUMS_BY_EXP_LOOSE['Book & Letter S'] = """[ 77.021734  88.300519  61.914368  98.959138  61.148613  84.79183
  154.139672  61.087805  65.347396  62.393978]"""
  NUMS_BY_EXP_LOOSE['Tape & Letter A'] = """[ 68.578101  62.955725  74.481509 138.858431  65.390915  58.477877
    67.424702  50.632621  92.245767  57.156118]"""
  NUMS_BY_EXP_LOOSE['Letter T & Letter H'] = """[35.342699 60.961558 53.155231 43.090101 44.581196 39.571603 37.496972
  69.336737 54.089913 34.060539]"""
  NUMS_BY_EXP_LOOSE['Letter G & Xbox'] = """[47.403513 60.136416 89.079204 60.299575 58.575521 62.155996 89.712703
  76.913838 55.543127 45.153685]"""

  NUMS_BY_EXP_TIGHT = {}
  NUMS_BY_EXP_TIGHT['Lotion & Letter R'] = """[ 85.624693   139.606369    88.932532    51.242615    71.720972
    47.4884     204.351574    59.281683    94.254986   127.64283277]"""
  NUMS_BY_EXP_TIGHT['Baby toy & Letter E'] = """[ 73.636422   134.9924      96.055005   118.961781   107.228642
    78.728906    65.197767   100.527002    60.977758   232.74956687]"""
  NUMS_BY_EXP_TIGHT['Letter B & Letter 3'] = """[ 41.969651    81.703706    97.3771     129.525115    60.150301
    59.509617   119.243303    68.600915    75.24231    119.76593032]"""
  NUMS_BY_EXP_TIGHT['Chicken Broth & Expo Box'] = """[100.168452    69.375249    79.35884     80.259635    83.3426197
  180.852226   164.547654    63.776864    95.78773     76.03319514]"""
  NUMS_BY_EXP_TIGHT['Chicken Broth & Wood Block'] = """[ 62.908881    55.441409    94.900758    78.078943    64.885896
    78.905356   101.685341   107.63960462  58.196709    41.68959336]"""
  NUMS_BY_EXP_TIGHT['Clamp & Letter I'] = """[ 64.098396   118.177834    42.451813   109.179393   132.873118
    64.004323    78.534282    74.769536    88.10843     99.70129258]"""
  NUMS_BY_EXP_TIGHT['Book & Letter S'] = """[ 88.395177    98.594609    75.89825     99.430851    67.603486
    93.385553   168.001711    89.817       78.04418     69.64313528]"""
  NUMS_BY_EXP_TIGHT['Tape & Letter A'] = """[124.021383    97.2843      84.675094   231.701359    69.879929
  103.350839   131.881783    97.545405    95.912242   154.46752226]"""
  NUMS_BY_EXP_TIGHT['Letter T & Letter H'] = """[119.51958     70.556571    93.894195   105.411879    60.28203
    79.85925     59.508137    84.951174    78.153462    44.14490969]"""
  NUMS_BY_EXP_TIGHT['Letter G & Xbox'] = """[122.115664   149.739404    96.147885    78.014471    88.948618
    75.968638   119.460989    97.971011   132.035388    83.33452462]"""

  RATES_BY_EXP = {
    'Lotion & Letter R': 14.06,
    'Baby toy & Letter E': 14.34,
    'Letter B & Letter 3': 14.31,
    'Chicken Broth & Expo Box': 13.84,
    'Chicken Broth & Wood Block': 14.12,
    'Clamp & Letter I': 14.14,
    'Book & Letter S': 14.43,
    'Tape & Letter A': 14.36,
    'Letter T & Letter H': 11.76,
    'Letter G & Xbox': 13.93,
  }

elif DO_TRIPLE_EXPS:
  NUMS_BY_EXP_LOOSE = {}
  NUMS_BY_EXP_LOOSE['Letter R & Letter A & Letter S'] = """[ 92.262502 120.19433  112.22866  231.696627 259.066598 154.918667
 170.23282  123.286999 169.734414 154.024857]"""

  NUMS_BY_EXP_TIGHT = {}
  NUMS_BY_EXP_TIGHT['Letter R & Letter A & Letter S'] = """[303.049388   195.831032   138.473562   261.173121   288.102346
 231.045255   183.1144     146.439181   179.194267   168.82529912]"""

  RATES_BY_EXP = {
    'Letter R & Letter A & Letter S': 7.23,
  }


ROW_TEMPLATE = """ & {NAME} & {RATE:.2f} & ${TIGHT_MEAN:.2f} \pm {TIGHT_STD_DEV:.2f}$ & ${TIGHT_MIN:.2f}, {TIGHT_MAX:.2f}$ & ${LOOSE_MEAN:.2f} \pm {LOOSE_STD_DEV:.2f}$ & ${LOOSE_MIN:.2f}, {LOOSE_MAX:.2f}$ \\\\ \cline{{2-7}}"""
# ROW_TEMPLATE = """ & \multirow{{2}}{{*}}{{ {NAME} }} & \multirow{{2}}{{*}}{{ {RATE:.2f} }} & ${TIGHT_MEAN:.2f} \pm {TIGHT_STD_DEV:.2f}$ & ${LOOSE_MEAN:.2f} \pm {LOOSE_STD_DEV:.2f}$ \\\\
#  & & & $[{TIGHT_MIN:.2f}, {TIGHT_MAX:.2f}]$ & $[{LOOSE_MIN:.2f}, {LOOSE_MAX:.2f}]$ \\\\ \cline{{2-5}}"""


def convert_str_to_list(s):
  s = s.strip('[]')
  return [float(x) for x in s.split()]

for key, val in NUMS_BY_EXP_LOOSE.items():
  NUMS_BY_EXP_LOOSE[key] = convert_str_to_list(val)
for key, val in NUMS_BY_EXP_TIGHT.items():
  NUMS_BY_EXP_TIGHT[key] = convert_str_to_list(val)

for key in NUMS_BY_EXP_LOOSE.keys():
  nums_loose = np.array(NUMS_BY_EXP_LOOSE[key])
  mean_loose = np.mean(nums_loose)
  std_dev_loose = np.std(nums_loose)
  min_loose = np.min(nums_loose)
  max_loose = np.max(nums_loose)

  nums_tight = np.array(NUMS_BY_EXP_TIGHT[key])
  mean_tight = np.mean(nums_tight)
  std_dev_tight = np.std(nums_tight)
  min_tight = np.min(nums_tight)
  max_tight = np.max(nums_tight)

  control_rate = RATES_BY_EXP[key]

  row = ROW_TEMPLATE.format(
    NAME=key.replace('&', '\&'),
    RATE=control_rate,
    TIGHT_MEAN=mean_tight,
    TIGHT_STD_DEV=std_dev_tight,
    TIGHT_MIN=min_tight,
    TIGHT_MAX=max_tight,
    LOOSE_MEAN=mean_loose,
    LOOSE_STD_DEV=std_dev_loose,
    LOOSE_MIN=min_loose,
    LOOSE_MAX=max_loose,
  )
  print(row)

breakpoint()
