import numpy as np
DMAP_TOTAL = np.load('DMAP_TOTAL.npy')
alip = np.load('ALIP.npy')
#footstep = np.load('FOOTSTEP.npy')
vdes = np.load('VDES.npy')
alip1 = np.load('ALIP1.npy')
#footstep1 = np.load('FOOTSTEP1.npy')
vdes1 = np.load('VDES1.npy')
ALIP = np.concatenate((alip, alip1), axis=0)
VDES = np.concatenate((vdes, vdes1), axis=0)
#FOOT = np.concatenate((footstep, footstep1), axis=0)
DMAP = DMAP_TOTAL.reshape((DMAP_TOTAL.shape[0], -1))
del DMAP_TOTAL
empty_list = []
empty_list.extend(DMAP)
del DMAP
print("dmap")
empty_list.extend(ALIP)
empty_list.extend(VDES)

#flattened_data = np.concatenate((DMAP, ALIP, VDES), axis=1)
np.save('observationsNN', np.array(empty_list))
