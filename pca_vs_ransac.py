#!/usr/bin/env python
import utils
import numpy
import time
import random
import matplotlib
###YOUR IMPORTS HERE###
from myutils import *
###YOUR IMPORTS HERE###

def add_some_outliers(pc,num_outliers):
    pc = utils.add_outliers_centroid(pc, num_outliers, 0.75, 'uniform')
    random.shuffle(pc)
    return pc

def main():
    #Import the cloud
    pc = utils.load_pc('cloud_pca.csv')

    num_tests = 10
    fig = None

    e_pca = []
    e_ransac = []
    time_pca = []
    time_ransac = []
    n_outlier_pca = []
    n_outlier_ransac = []
    epson = .05

    for i in range(0,num_tests):
        pc = add_some_outliers(pc,10) #adding 10 new outliers for each test
        # fig = utils.view_pc([pc])

        ###YOUR CODE HERE###

        start = time.clock()
        pca_model = pca(pc)
        end   = time.clock()
        time_pca.append(end-start)
        
        start = time.clock()
        ransac_model = ransac(pc,iteration = 400,lower_bound=100,N_to_fit = 3, epson = epson)
        end   = time.clock()
        time_ransac.append(end-start)

        # fig = utils.draw_plane(fig,pca_model[1],pca_model[0],color = (1,0,0,0.3))
        # fig = utils.draw_plane(fig,ransac_model[1],ransac_model[0])
        
        sum_pca = 0
        sum_ransac = 0
        num_pca = 0
        num_ransac = 0
        for pt in pc:
            error = error_plane(pt,pca_model)
            if error < epson:
                sum_pca = sum_pca + error
                num_pca = num_pca + 1
            error = error_plane(pt,ransac_model)
            if error < epson:
                sum_ransac = sum_ransac + error
                num_ransac = num_ransac + 1
        e_pca.append(float(sum_pca))
        e_ransac.append(float(sum_ransac))
        n_outlier_pca.append(len(pc)-num_pca)
        n_outlier_ransac.append(len(pc)-num_ransac)
        # raw_input("Press enter to continue")
    
    inlier  = []
    outlier = []    
    for j in range(len(pc)):
        if error_plane(pc[j],pca_model) <= epson:
            inlier.append(pc[j])
        else:
            outlier.append(pc[j])
    fig1 = utils.view_pc([inlier],color='r')
    fig1 = utils.view_pc([outlier],fig1,color='b')
    fig1 = utils.draw_plane(fig1,pca_model[1],pca_model[0],color=(0,1,0,0.3))
    fig1.suptitle('PCA fitting')

    inlier  = []
    outlier = []    
    for j in range(len(pc)):
        if error_plane(pc[j],ransac_model) <= epson:
            inlier.append(pc[j])
        else:
            outlier.append(pc[j])
    fig2 = utils.view_pc([inlier],color='r')
    fig2 = utils.view_pc([outlier],fig2,color='b')
    fig2 = utils.draw_plane(fig2,ransac_model[1],ransac_model[0],color=(0,1,0,0.3))
    fig2.suptitle('RANSAC fitting')
    
    fig3 = matplotlib.pyplot.figure()
    ax = fig3.add_subplot(111)
    ax.scatter(e_pca,n_outlier_pca,label="PCA")
    ax.scatter(e_ransac,n_outlier_ransac,label="RANSAC")
    ax.legend()
    ax.set_xlabel('Error')
    ax.set_ylabel('Numter of Outliers')
    matplotlib.pyplot.show()

    fig4 = matplotlib.pyplot.figure()
    ax = fig4.add_subplot(111)
    ax.plot(range(num_tests),time_pca,label="PCA")
    ax.set_xlabel('Iteration')
    ax.set_ylabel('Computation time')
    ax.legend()
    matplotlib.pyplot.show()

    fig5 = matplotlib.pyplot.figure()
    ax = fig5.add_subplot(111)
    ax.plot(range(num_tests),time_ransac,label="RANSAC")
    ax.set_xlabel('Iteration')
    ax.set_ylabel('Computation time')
    ax.legend()
    matplotlib.pyplot.show()
    
    choice = raw_input("Save figures?(y/n)\n")
    if choice == "y":
        fig1.savefig("PCAvsRAN_PCA_Fitting.png")
        fig2.savefig("PCAvsRAN_RAN_Fitting.png")
        fig3.savefig("PCAvsRAN_Error_Outlier.png")
        fig4.savefig("PCAvsRAN_PCA_ComputationTime.png")
        fig5.savefig("PCAvsRAN_RAN_ComputationTime.png")

    raw_input("Press enter to end:")


if __name__ == '__main__':
    main()
