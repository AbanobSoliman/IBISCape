import numpy as np
import time, sys, os
import distutils
from distutils import util
from distutils import dir_util
    
eventsdir = '/media/abanobsoliman/DATA/Abanob_PhD/Algorithms_Dev/CARLA/IBISCape-master/Calib_Acquisition/ESVI/davisR/events'

def getDvsArrayFiles(dir):
    '''Generates a list of files from the directory'''
    dvsarrays_files = list()
    timestamps = list()
    if os.path.exists(dir):
        for path, names, files in os.walk(dir):
            for f in files:
                if os.path.splitext(f)[1] in ['.npz']:
                    dvsarrays_files.append( os.path.join( path, f ) )
                    timestamps.append(os.path.splitext(f)[0]) 
    #sort by timestamp
    sort_list = sorted(zip(timestamps, dvsarrays_files))
    dvsarrays_files = [file[1] for file in sort_list]
    return dvsarrays_files

event_arrays = getDvsArrayFiles(eventsdir)
for event_array in event_arrays:
    events_data = np.load(event_array)
    print(events_data['t'], events_data['x'], events_data['y'], events_data['p'].astype(int))
