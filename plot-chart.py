#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np


def main():

    # Open files
    f_pc_fq = open('pc_freqs.txt')
    f_cc_fq = open('cc_freqs.txt')
    f_sc_fq = open('sc_freqs.txt')




    pc_fq = np.array([])
    for line in f_pc_fq:
        data = line.split()
        pc_fq = np.append(pc_fq,float(data[0]))
    
    cc_fq = np.array([])
    for line in f_cc_fq:
        data = line.split()
        cc_fq = np.append(cc_fq,float(data[0]))

    sc_fq = np.array([])
    for line in f_sc_fq:
        data = line.split()
        sc_fq = np.append(sc_fq,float(data[0]))


    print pc_fq

    # Get accumulated value array
    pc_fq_acc = np.cumsum(pc_fq)
    print 'pc'
    print pc_fq_acc

    # Insert where the first CC happens
    cc_fq = np.insert(cc_fq, 0, 0.5)
    cc_fq_acc = np.cumsum(cc_fq)
    print 'cc'
    print cc_fq_acc
    
    
    v = sc_fq[0]
    l = pc_fq_acc[len(pc_fq_acc)-1]
    sc_fq = np.zeros(l / (1.0/v))
    print sc_fq

    sc_fq.fill(1.0/v)
    print 'sc_fq'
    print l
    print sc_fq

    sc_fq_acc = np.cumsum(sc_fq)
    print 'sc'
    print sc_fq_acc

    pc_y = np.zeros(len(pc_fq_acc))
    pc_y.fill(0.1)
    cc_y = np.zeros(len(cc_fq_acc))
    cc_y.fill(0.05)
    sc_y = np.zeros(len(sc_fq_acc))
    sc_y.fill(0)
    
    s = 40
    
    fig, ax = plt.subplots()

    #ax is the axes instance
    group_labels = ['', 'SC', 'CC', 'PC', '']
    #group_labels = ['SC', 'CC', 'PC']
    ax.set_yticklabels(group_labels)

    # Plot the values
    plt.plot(pc_fq_acc, pc_y, 'k+', markersize=s)
    plt.plot(cc_fq_acc, cc_y, 'k+', markersize=s)
    plt.plot(sc_fq_acc, sc_y, 'k+', markersize=s)
    plt.axis([0, 18.75, -0.05, 0.15])

    plt.show()


    # Close files
    f_pc_fq.close()
    f_cc_fq.close()
    f_sc_fq.close()

    print 'Exiting normally'


if __name__ == '__main__':
    main()
