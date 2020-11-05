#!/usr/bin/env python
import utils
import numpy
###YOUR IMPORTS HERE###
from ransac_util import *
###YOUR IMPORTS HERE###


def main():
    #Import the cloud
    pc = utils.load_pc('cloud_ransac.csv')


    ###YOUR CODE HERE###
    # Show the input point cloud
    utils.view_pc([pc])

    #Fit a plane to the data using ransac
    iteration = 500
    N = 120
    epson = 0.07

    N_to_fit = 3
    N_sample = len(pc)

    error_best = numpy.Infinity
    model = None

    for i in range(iteration):
        picked=[]
        pts = []
        while len(picked) < N_to_fit:
            n = numpy.random.randint(0,N_sample)
            if n not in picked:
                picked.append(n)
                pts.append(pc[n])
        
        plane_org = pts[0]
        plane_normal = numpy.cross(numpy.reshape(numpy.array(pts[1]-pts[0]),(3,)),numpy.reshape(numpy.array(pts[2]-pts[0]),(3,)))

        for j in range(N_sample):
            if j not in picked:
                if error_plane(pc[j],[plane_org,plane_normal]) <= epson:
                    pts.append(pc[j])
        

        if len(pts) >= N:
            picked = []
            pt = []
            while len(pt) < N_to_fit:
                n = numpy.random.randint(0,len(pts))
                if n not in picked :
                    picked.append(n)
                    pt.append(pts[n])

            # plane_org, plane_normal = plane_model(pts)
            plane_org = pts[0]
            plane_normal = numpy.cross(numpy.reshape(numpy.array(pts[1]-pts[0]),(3,)),numpy.reshape(numpy.array(pts[2]-pts[0]),(3,)))
            plane_normal = numpy.reshape(numpy.matrix(plane_normal),(3,1))

            error_new = 0
            for j in range(len(pts)):
                error_new = error_new + error_plane(pts[j],[plane_org, plane_normal])
            error_new = error_new/len(pts)
            if error_new < error_best:
                error_best = error_new
                model = [plane_org, plane_normal]

    #Show the resulting point cloud
    if model == None:
        print "model not found"
    
    else:
        inlier  = []
        outlier = []
        
        for j in range(N_sample):
            if error_plane(pc[j],model) <= epson:
                inlier.append(pc[j])
            else:
                outlier.append(pc[j])
        fig = utils.view_pc([inlier],color='r')
        fig = utils.view_pc([outlier],fig,color='b')

    #Draw the fitted plane
        fig = utils.draw_plane(fig,model[1],model[0],color=[0,1,0,0.3])
        print model[1]
        print model[0]

    ###YOUR CODE HERE###
    raw_input("Press enter to end:")


if __name__ == '__main__':
    main()
