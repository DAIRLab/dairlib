import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import scienceplots

plt.style.use(['science'])

object_names = ["Letter I", "Letter C", "Letter R", "Letter A",
                "Letter Y", "Letter G", "Letter B", "Letter 3",
                "Expo Box", "Lotion", "Wood Block", "Tape",
                "Letter H", "Letter E", "Letter S", "Eraser", "Milk Bottle",
                "Clamp", "Chicken Broth", "Egg Carton", "Book",
                "Baby Toy", "Gallon Milk", "Push T"]
time_to_goal_dict = {
    "Letter I": np.array(
        [25.757018, 29.114636, 33.174714, 16.486989, 34.65505 , 30.222808,
         25.975983, 18.772502, 40.689416, 15.251655, 57.035517, 38.664147,
          9.095817, 15.094964, 18.485326, 22.977314, 25.267483, 34.67599 ,
         51.805841, 34.900511, 33.862241,  9.841146, 22.420204, 15.067717,
         31.844655, 54.862436, 29.752983, 29.228146, 18.73193 , 29.62443249]),
    "Letter C": np.array(
        [
            12.504849,
            46.050372,
            12.904977,
            18.630899,
            22.98211,
            14.355698,
            40.329019,
            22.90577,
            14.243299,
            11.618166,
            21.328255,
            21.218845,
            17.294778,
            11.126389,
            20.120953,
            34.50109,
            29.847047,
            17.267355,
            19.700622,
            29.118946,
            15.565178,
            42.819162,
            32.362129,
            19.851441,
            22.77829,
            25.058341,
            24.335656,
            17.85226,
            9.528054,
            27.780079,
            12.246774,
            16.505961,
            20.668684,
            18.894981,
            40.389608,
            30.123388,
            23.987545,
            18.854979,
            12.492399,
            17.21133413,
            20.496671,
            26.452592,
            21.054605,
            34.737928,
            23.523438,
            16.736926,
            29.601942,
            22.506794,
            26.480555,
            13.69234136,
            30.031089,
            19.510749,
            28.768424,
            12.385184,
            28.722454,
            13.673332,
            9.755754,
            13.96705,
            24.71405,
            16.062451,
            27.674546,
            17.383323,
            30.452491,
            22.5992,
            26.541464,
            22.858676,
            17.75882487,
        ]
    ),
    "Letter R": np.array(
        [14.024184   , 8.873058   , 5.493102   ,48.401111   ,37.702387   ,40.060021,
         20.650591   ,13.107377   ,26.331809   ,38.157198   ,16.820464   ,15.584641,
         35.701609   ,52.481782   ,60.21698    ,11.797333   ,26.450369   ,21.743971,
         35.781673   ,14.545625   , 6.547173   ,28.837372   ,22.450141   ,31.997431,
         24.280292   ,31.53502    ,20.30459    ,60.873837   ,14.927862   , 6.59190719]
    ),
    "Letter A": np.array(
        [20.294977   , 7.607625   , 3.741967   , 4.222951   ,20.307688   ,15.173095,
         17.104555   ,53.843144   ,26.848264   ,28.671989   ,31.780608   ,24.086568,
         60.20648    ,19.418408   ,22.127094   ,31.343856   ,29.264121   ,18.06447,
         16.511513   ,20.317036   ,23.491719   ,17.881879   ,47.830104   ,24.911092,
         26.960036   ,36.194724   ,24.385021   ,22.725815   ,27.538084   ,14.37379481]
    ),

    "Letter Y": np.array(
        [36.376573   ,35.930429   , 8.181078   ,46.907527   ,39.601244   ,99.391923,
         49.720839   ,20.559913   ,22.914147   ,16.232914   , 8.518005   ,22.938322,
         31.395612   ,26.503416   , 9.841349   ,22.212915   ,59.927323   ,30.306667,
         46.035157   ,63.781047   ,48.839974   ,21.435014   ,46.770518   ,42.080366,
         33.116036   ,13.822863   ,12.307019   ,36.518254   , 8.257253   , 7.10467789]
    ),
    "Letter G": np.array(
        [22.024264   ,25.805735   ,30.721846   ,21.57587    ,36.138794   ,20.450896,
         32.511118   ,40.478234   ,33.620629   ,16.753674   ,18.704603   ,22.245355,
         35.863918   ,25.439203   ,50.030275   , 9.461086   ,29.014494   ,20.382764,
         16.389949   ,22.803587   ,38.704954   ,20.538897   ,29.285261   ,24.023584,
         14.469864   ,21.509493   ,23.415809   ,15.386251   ,21.019631   ,18.11392357]
    ),
    "Letter B": np.array(
        [16.571127   ,19.846528   ,15.244721   ,59.413968   ,12.606703   ,17.728804,
         32.72943    ,18.121906   ,26.023503   ,32.04036    ,23.512223   ,31.989777,
         44.920056   ,21.084012   ,23.084887   ,36.349111   ,10.327452   ,18.992252,
         16.696936   ,30.323386   ,12.318057   ,60.052334   ,42.217643   ,32.628965,
         28.974222   ,60.632703   ,34.108633   ,15.531057   ,28.858036   ,22.70506056]
    ),
    "Letter 3": np.array(
        [33.218894, 28.788944, 14.458497, 29.092279, 28.986172, 20.172026,
         18.945857, 38.486983, 27.632148, 21.801162, 24.192482, 35.176846,
         17.547159, 15.880361, 11.167465, 19.183835, 16.717553, 19.464878,
         16.418714, 41.873251, 20.41299 , 25.868894, 13.001288, 14.764182,
         24.973757, 20.233814, 18.41458 ,  7.843881, 24.385828, 31.85325239]
    ),
    "Expo Box": np.array(
        [29.298551  ,35.243647  ,30.756264  ,33.655734  ,34.141525  ,43.145541,
         16.321228  ,63.752388  ,79.79236   ,16.947389  ,20.008365  ,16.536326,
         14.661509  ,22.950989  ,52.257074  ,13.02851   ,80.846792  ,19.790363,
         13.716303  ,23.15309   ,42.450392  ,15.495681  ,31.108057  ,14.885639,
         34.274944  ,52.949559  ,34.41955   ,23.561368  ,47.280751  ,17.6275555]),
    "Lotion": np.array(
        [35.775851,   10.178375,   46.837635,    8.28206 ,   32.73984 ,   27.183357,
         5.351624 ,  38.84224,    25.252049,   17.490403 ,  15.89661  ,  71.565777 ,
         12.044986,   43.697,      40.199991,   22.676181,   15.467743,    7.538044,
         22.852783,   16.288329,   18.73001957, 38.371047,   18.886973,   13.279757,
         27.967975,   16.336693,   16.280886,   21.587882,   10.857986,   21.145498,
         18.95761695]
    ),
    "Wood Block": np.array(
        [17.86797 , 21.457734, 38.508601, 28.967483, 14.898367, 14.549094,
         23.336327, 34.920809, 34.32953 , 11.332717, 23.175423, 15.405093,
         52.550113, 27.12606 , 30.598832, 29.140784, 27.560934, 23.356698,
         24.881366, 10.328723,  7.205077,  8.997016, 23.159283, 64.235609,
         62.269655,  6.265807, 51.911279,  5.806491,  6.130102, 31.09136609]
    ),
    "Tape": np.array(
        [14.995173, 29.6097  , 25.039956, 29.067428, 21.41887 , 34.07814,
         30.606273, 39.06405 , 35.682958, 32.093123, 32.789687, 20.566454,
         27.553937, 36.964265, 19.114107, 74.304231, 26.637079, 18.799556,
          9.363798, 17.46502 , 39.69727 , 47.444177, 14.283822, 14.351053,
         19.691786, 24.581508, 31.854765, 40.665966, 80.197676, 46.94904988]
    ),
    "Letter H": np.array(
        [14.118313, 31.023754, 69.525158,  9.907442, 22.900235, 20.440799,
         34.87607 , 21.472768,  7.641811, 30.426668, 33.823499, 26.685718,
         23.023243, 22.397439, 45.141548, 47.219983, 21.790823, 84.02599,
         65.826383, 15.657783,  5.407556, 29.196172, 25.132813, 26.239343,
         57.105737, 35.987879, 52.91245 , 61.594713, 66.438638, 18.25350853]
    ),
    "Letter E": np.array(
        [10.400291, 10.046252, 15.90657 , 18.250357, 44.669057, 37.402589,  9.728242,
         18.62118 , 58.903314, 24.57548 , 33.832606, 25.933344, 11.890645, 22.532471,
         26.969423, 22.380592, 51.11146 , 20.141707, 21.253144, 17.742299,  8.459278,
         34.066267,  5.901699, 26.827317, 12.407103, 55.419175, 44.096342, 19.21929,
         26.468317, 28.240589]
    ),
    "Letter S": np.array(
        [32.682416,  4.796416, 20.719149, 31.045053, 34.637608, 21.269572,
         13.209143, 27.31895 , 57.331264, 40.080089, 55.077972, 20.200772,
         34.712317, 33.968299, 14.573462, 30.103996, 41.9602  , 40.601405,
         25.431655, 25.827394, 17.539197, 16.053191, 13.587407, 17.578287,
         16.686286, 13.676513, 18.612617, 40.04186 , 19.426312, 32.75868491]
    ),
    "Eraser": np.array(
        [23.329096,  8.874099, 32.265573, 36.046312, 27.199896, 27.074149,
         39.031376, 50.223185, 16.75967 , 22.083881, 27.101266, 21.308449,
         24.13804 , 24.428173, 24.820887, 30.979684, 28.464412, 39.614854,
         34.255025, 34.43368 , 24.82295 , 17.279821, 18.509234, 22.234077,
         15.907446, 41.131675, 23.099555, 33.294733, 15.965616, 34.55686626]
    ),
    "Milk Bottle": np.array(
        [ 14.122439, 13.368286, 120.201522, 56.498082, 90.242219,
          84.728101, 17.726709,  65.042892, 44.940078, 14.034224,
          22.545015, 20.033257,  34.153602, 16.396332, 94.631445,
          15.473138, 12.006978,  25.69048 ,  7.462492, 15.659396,
          12.129298,  9.412821,  21.335113,  5.008727, 12.251569,
          53.379662, 14.181826,  47.944074, 21.349272, 84.19932286]
    ),
    "Clamp": np.array(
        [ 70.503792    ,38.902839   ,196.545356    , 8.810689    ,16.213832,
          127.322113   , 19.354997  ,  27.001246   ,139.903235   , 20.391133,
          51.930429    ,45.852011   ,  5.994609    ,14.539774    ,43.444161,
          48.479119    ,12.253297   , 21.249094    , 7.818597    ,47.44692,
          110.698045   , 38.403935  ,  30.768069   ,110.428809   ,176.733444,
          17.864722    ,48.442539   , 39.36483     ,22.909648    ,31.183576,
          6.60714592]
    ),
    "Chicken Broth": np.array(
        [ 22.1972  ,    10.792049,    17.099497,    86.042382,    59.4963,
          42.966124,     8.571319,   206.097228,    26.914787,    79.385771,
          44.037135,    18.291615,    10.952244,    24.099008,    41.817421,
          57.365332,    55.156654,    12.918298,    56.017176,    55.984842,
          38.396897,    19.183161,    55.433637,    38.646196,    25.563705,
         106.227873,    35.584317,     4.627436,    31.714367,    37.602653,
          18.43562716]
    ),
    "Egg Carton": np.array(
        [ 22.107486    ,39.795559    ,17.338384    ,52.368163   ,101.527914,
          29.152798    ,56.151401    ,32.686615    ,17.443936   , 16.520428,
          42.32608514  ,30.096894    ,33.332293    ,78.5415     , 13.940133,
          35.881131    ,28.048634    ,86.983681    ,39.351549   , 24.582895,
          55.766171    ,28.401565    ,32.702864    ,24.667264   , 32.816471,
          20.914759    ,39.423047    ,33.136573    ,10.163027   , 26.06903154]
    ),
    "Book": np.array(
        [25.722294, 48.234364, 16.680345, 45.589689, 45.866184, 26.865179,
         26.919368, 24.723296, 58.812154, 31.022318, 31.750112, 35.598583,
         83.715212, 31.613182, 48.19067 , 49.106173, 43.879285, 38.818021,
         69.684299, 19.683752, 24.812913, 10.010766, 22.675498, 45.827814,
         27.678418, 21.124211, 37.989377, 41.297306, 64.304234, 30.59936665]
    ),
    "Baby Toy": np.array(
        [31.865263, 36.661331, 36.838468, 15.716777, 59.13489 , 13.242358,
         50.286192, 33.770748, 25.209621, 56.309446, 69.784358, 12.023506,
         40.522026, 41.384595, 41.111703, 20.86831 , 40.272771, 18.296468,
         31.100087, 66.174749, 46.687202,  9.709766, 27.008293, 67.504434,
         28.040726, 18.050038, 45.954216, 26.412005, 73.331854, 16.90514623]
    ),
    "Gallon Milk": np.array(
        [ 77.997401, 42.101358, 101.203106, 19.939704, 28.439287,
          50.825653, 52.907945,  46.885778, 29.052104, 43.166811,
          59.774371, 22.502454,  34.311626, 39.91496 , 35.32721,
          47.348224, 33.057319,  26.279418,  9.652367, 61.26951,
          14.673575, 45.667149,  46.167881, 72.974275, 46.591,
          33.567412, 15.928328,  19.666058, 33.500877, 20.98864506]
    ),
    "Push T": np.array(
        [32.136767   , 9.185306   ,27.629168   ,10.431795   ,22.331885   ,40.017823,
         32.584353   ,17.249512   ,19.660289   ,17.039562   ,43.194483   ,55.35306,
         16.282805   ,12.179409   ,14.27122    ,12.18904    ,12.898052   ,61.879378,
          8.156285   ,25.274766   ,23.295628   ,11.82061    ,28.822678   ,19.092198,
         57.171497   ,18.564928   ,39.442278   ,37.31236    ,46.463339   ,36.40338742]
    ),
}

time_to_goals = [time_to_goal_dict[obj_name].tolist()[2:] for obj_name in object_names]

# Positions for boxplots (y-axis)
positions_our = np.arange(len(object_names))

fig, (ax, ax2) = plt.subplots(1, 2, sharey=True, figsize=(5, 11), gridspec_kw={'width_ratios': [3, 1]}, )


# Boxplots with horizontal orientation
bp_our = ax.boxplot(
    time_to_goals,
    positions=positions_our,
    widths=0.25,
    patch_artist=True,
    vert=False,
    boxprops=dict(facecolor="none", edgecolor="black"),
    medianprops=dict(color="black"),
    whiskerprops=dict(color="black"),
    capprops=dict(color="black"),
    flierprops=dict(marker="o", color="red", alpha=0.5),
    showfliers=False,
    notch=False,
)
# Scatter points over boxplots
for i, x in enumerate(time_to_goals):
    ax.scatter(x, np.full_like(x, positions_our[i]), color="royalblue", alpha=0.5, s=5)
    ax2.scatter(x, np.full_like(x, positions_our[i]), color="royalblue", alpha=0.5, s=5)

# Y-axis labels (categories)
ax.set_yticks(range(len(object_names)))
ax.set_yticklabels(object_names)

# Log scale for x-axis
# ax.set_xscale("log")

# Labels
# ax.set_xlabel("Time to goal (s)")
# ax.set_ylabel("Objects")

fig.suptitle("Time to Goal for Single Object with C3+")
fig.supxlabel("Time to goal (s)")

# hide the spines between ax and ax2
ax.set_xlim(0, 115)
ax2.set_xlim(122, 200)
ax.spines['right'].set_visible(False)
ax2.spines['left'].set_visible(False)
ax.yaxis.tick_left()
ax.tick_params(labelright=False)
ax2.yaxis.tick_right()

d = .010  # how big to make the diagonal lines in axes coordinates
# arguments to pass plot, just so we don't keep repeating them
d1 = d * 3
kwargs = dict(transform=ax.transAxes, color='k', clip_on=False)
ax.plot((1-d, 1+d), (-d, +d), **kwargs)
ax.plot((1-d, 1+d), (1-d, 1+d), **kwargs)

kwargs.update(transform=ax2.transAxes)  # switch to the bottom axes
ax2.plot((-d1, +d1), (1-d, 1+d), **kwargs)
ax2.plot((-d1, +d1), (-d, +d), **kwargs)

plt.tight_layout()
plt.savefig("c3plus_single_object_performance.png", dpi=600)

# This part is from the original code
time_to_goals = [time_to_goal_dict[obj_name].tolist() for obj_name in object_names]
positions_our = np.arange(len(object_names))

# Create a figure with 2 rows and 1 column, sharing the x-axis.
# The plot is rotated, so we now have a vertical broken axis.
# `ax` is the bottom (main) plot, and `ax2` is the top plot.
fig, (ax2, ax) = plt.subplots(2, 1, sharex=True, figsize=(8, 5), gridspec_kw={'height_ratios': [1, 10]})

# --- Main changes for rotation ---

# 1. Boxplots are now vertical (`vert=True`)
common_boxplot_kwargs = dict(
    widths=0.35,
    patch_artist=True,
    vert=True, # Set to True for vertical orientation
    boxprops=dict(facecolor="none", edgecolor="black"),
    medianprops=dict(color="black"),
    whiskerprops=dict(color="black"),
    capprops=dict(color="black"),
    flierprops=dict(marker="o", color="red", alpha=0.5),
    showfliers=False,
    notch=False,
)
ax.boxplot(time_to_goals, positions=positions_our, **common_boxplot_kwargs)
# ax2.boxplot(time_to_goals, positions=positions_our, **common_boxplot_kwargs)


# 2. Scatter points are plotted with x and y swapped.
# A small amount of jitter is added to the x-position for better visibility.
for i, data in enumerate(time_to_goals):
    jitter = np.random.normal(0, 0.05, size=len(data))
    x_positions = np.full_like(data, positions_our[i]) + jitter
    ax.scatter(x_positions, data, color="darkorange", alpha=0.7, s=5)

    x_positions = np.array(x_positions)
    data = np.array(data)

    # Create mask and filter
    mask = data > 112
    x_filtered = x_positions[mask]
    data_filtered = data[mask]

    # Scatter plot
    ax2.scatter(x_filtered, data_filtered, color="darkorange", alpha=0.7, s=5)

# 3. Set x-axis ticks and labels (previously y-axis)
ax.set_xticks(range(len(object_names)))
ax.set_xticklabels(object_names, rotation=45, ha="right")

# --- Adjusting the broken axis for vertical orientation ---

# 4. Set the y-axis limits to create the break (previously x-axis)
ax.set_ylim(0, 112)
ax2.set_ylim(112, 220)

# 5. Hide the spines between the two subplots
ax.spines['top'].set_visible(False)
ax2.spines['bottom'].set_visible(False)
ax.xaxis.tick_bottom()
ax2.xaxis.tick_top()
ax2.tick_params(labeltop=False)
ax.tick_params(labelbottom=True)
ax2.get_xaxis().set_visible(False)


# 6. Add diagonal lines to indicate the break
d = .015  # size of the diagonal lines
kwargs = dict(transform=ax.transAxes, color='k', clip_on=False)
ax.plot((-d, +d), (1 - d, 1 + d), **kwargs)      # top-left diagonal
ax.plot((1 - d, 1 + d), (1 - d, 1 + d), **kwargs) # top-right diagonal

kwargs.update(transform=ax2.transAxes)  # switch to the top axes
ax2.plot((-d, +d), (-d*10, +d*10), **kwargs)      # bottom-left diagonal
ax2.plot((1 - d, 1 + d), (-d*10, +d*10), **kwargs) # bottom-right diagonal

rect = patches.Rectangle(
    (-0.5, 112),                 # (x, y) bottom-left corner
    len(object_names),     # width covers all x positions
    220 - 112,                   # height = full y-range of ax2
    facecolor="lightgrey",
    alpha=0.3,
    zorder=0
)
ax2.add_patch(rect)

# --- Final plot labeling ---

# 7. Update the main title and axis labels
ax2.set_title("Time to Goal for Single Object with C3+")
fig.supylabel("Time to goal (s)") # Was supxlabel

plt.tight_layout(rect=[0, 0.03, 1, 0.95]) # Adjust layout
plt.savefig("c3plus_single_object_performance_rotated.png", dpi=600)
