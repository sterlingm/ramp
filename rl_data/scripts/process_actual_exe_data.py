#!/usr/bin/env python
import os
import rosbag
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

## open bag
bag = rosbag.Bag("/home/kai/catkin_ws/src/ramp/rl_data/ramp_test_data/raw/2018-01-09_17:51:37/raw_data/1.bag")
## set name
dir_name = 'coes_1_0.3_0.5_1_1'
fig_name = 'fig_0.png'
file_name = 'actual_exe_time.csv'
## paras.
window_size = 1
start = 70
end = 79



## get dir.
ramp_test_data_dir = os.path.join(os.path.dirname(__file__), '../ramp_test_data/{}/'.format(dir_name))
os.system('mkdir -p ' + ramp_test_data_dir)
file_h = open(ramp_test_data_dir + file_name, "a")

exe_times = np.array([])
cnt = 0
print("Processing data......")
for topic, msg, t in bag.read_messages(topics=["/ramp_collection_exe_time"]):
    cnt += 1
    if cnt - 1 < start:
        continue
    elif cnt - 1 > end:
        break
    
    topic = topic.decode()
    if topic == "/ramp_collection_exe_time":
        ## process execution time
        exe_times = np.append(exe_times, msg.data)
    else:
        ## never enter here
        assert False

    file_h.write('{}, 0.0, 0.0, 0.0, 0.0\n'.format(msg.data))

file_h.write('0.0, {}, {}, {}, {}\n'.format(np.mean(exe_times), np.min(exe_times),
                                        np.max(exe_times), np.std(exe_times)))

print("Processing completed!")

## plot
fig_times = plt.figure(figsize = (15, 10))
times_smoothed = pd.Series(exe_times).rolling(window_size, min_periods = window_size).mean()
plt.subplot(1, 1, 1)
plt.plot(times_smoothed)
plt.xlabel("execution")
plt.ylabel("durations/s (smoothed over window size {})".format(window_size))
plt.savefig(ramp_test_data_dir + fig_name) # must be before plt.show()

## close bag
bag.close()
file_h.close()

plt.show(fig_times)