#!/usr/bin/env python
import utils
import numpy
import matplotlib.pyplot as plt
###YOUR IMPORTS HERE###
from icp_utils import *
import sys
###YOUR IMPORTS HERE###


def main():
    #Import the cloud
    pc_source = utils.load_pc('cloud_icp_source.csv')

    ###YOUR CODE HERE###
    pc_target = utils.load_pc('cloud_icp_target3.csv') # Change this to load in a different target



    utils.view_pc([pc_source, pc_target], None, ['b', 'r'], ['o', '^'])
    plt.axis([-0.15, 0.15, -0.15, 0.15])
    ###YOUR CODE HERE###
    iteration = 100
    epson = .5
    wait_no_change = 10
    
    error = []
    e_best = np.Infinity
    j = 0
    for i in range(iteration):
        Cp, Cq = findclosest(pc_source,pc_target)
        R, t = GetTransform(Cp,Cq,False)
        error.append(SumCorrespondError(R,t,[Cp,Cq]))
        if error[-1] < epson:
            print "Smaller than epson ",error[-1], " < ", epson
            break
        print "Iteration ",i,", error = ",error[-1]
        pc_source = updateP(R,t,pc_source)
        if e_best > error[-1]:
            e_best = error[-1]
            pc_best = pc_source
            j = 0
        else:
            j = j+1
            if j > wait_no_change:
                print "No more better solution, stop iteration."
                break
    utils.view_pc([pc_source, pc_target], None, ['b', 'r'], ['o', '^'])
    plt.gca().set_aspect('equal', adjustable='box')
    plt.legend(["Fitting","Target"])
    # utils.view_pc([pc_best, pc_target], None, ['b', 'r'], ['o', '^'])
    # plt.axis([-0.15, 0.15, -0.15, 0.15])
    
    # duration = 1  # seconds
    # freq = 440  # Hz
    # os.system('play -nq -t alsa synth {} sine {}'.format(duration, freq))
    sys.stdout.write('\a')
    sys.stdout.flush()

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(range(len(error)),error)
    ax.set_xlabel('Iteration')
    ax.set_ylabel('Error')
    ax.legend()
    plt.show()

    raw_input("Press enter to end:")


if __name__ == '__main__':
    main()
