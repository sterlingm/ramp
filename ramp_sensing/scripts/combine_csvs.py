#!/usr/bin/env python

import rospy
import sys
import os


def getTimeStep(l):
    l_list = list(l)
    i_result = 0

    for i,l in enumerate(l_list):
        if not l.isdigit():
            i_result = i
            break
    return i_result

def main():
    rospy.init_node('combine_csvs', anonymous=False)

    # Get all files in directory
    files = \
    os.listdir('/home/sterlingm/ros_workspace/src/ramp/ramp_sensing/occ_map_data')
    print files

    f_result = \
    open('/home/sterlingm/ros_workspace/src/ramp/ramp_sensing/occ_map_data.csv', 
            'w')

    # Persist the time step value through the files
    timeStep = 0
    num_prev = 0

    # Go through each file
    for i,fname in enumerate(files):
        p = \
        os.path.join('/home/sterlingm/ros_workspace/src/ramp/ramp_sensing/occ_map_data', 
                fname)
        f = open(p, 'r')

        # Get the lines
        lines = f.read().split('\n')
        print('Number of lines in file %s: %i' % (fname, len(lines)))

        print('Before starting new file, timeStep: %d' % timeStep)

        # For each line
        for i_l,l in enumerate(lines):


            # Set the correct time step
            l_list = list(l)
            
            # Get the time step in the file
            i_num = getTimeStep(l)
            n = ''.join(l_list[0:i_num])
            num = int(n)

            # check if time step has changed
            if num != num_prev:
                timeStep = timeStep + 1
            num_prev = num

            if timeStep > 9:
                l_list[0:i_num] = str(timeStep)
            else:
                l_list[0:i_num] = str(timeStep)
                

            # Join the list back into a string
            l = "".join(l_list)
            
            # Write to f_result
            f_result.write(l + os.linesep)

        # Adjust the time step value
        #timeStep = timeStep + len(lines)

        print('After file, timeStep: %d' % timeStep)
        
            
        # Close file
        f.close();

    # Close file
    f_result.close()

    # Open the file again to get the total number of lines
    f_result = \
    open('/home/sterlingm/ros_workspace/src/ramp/ramp_sensing/occ_map_data.csv', 
            'r')
    
    # Get total number of lines
    lines = f_result.read().split('\n')
    print('Number of lines in resulting file: %i' % len(lines))

    # Close file
    f_result.close()

if __name__ == '__main__':
    main()
