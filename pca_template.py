#!/usr/bin/env python
import utils
import numpy
###YOUR IMPORTS HERE###

def GetArrayToPC( X ):
    X = numpy.reshape(X,(X.shape[0],X.shape[1],1))
    X = [numpy.matrix(x) for x in X]
    return X

###YOUR IMPORTS HERE###
    

def main():

    #Import the cloud
    pc = utils.load_pc('cloud_pca.csv')


    ###YOUR CODE HERE###
    # Show the input point cloud
    fig = utils.view_pc([pc])

    #Rotate the points to align with the XY plane

    X = numpy.squeeze(numpy.array(pc),axis = 2)
    mu = numpy.mean(X,axis = 0)

    for i in range(X.shape[0]):
        X[i,:] = X[i,:]-mu
    N = X.shape[0]
    Q = numpy.matmul(X.T,X)/(N-1)   
    u,s,vh = numpy.linalg.svd(Q)

    print "----------------"
    print "V.T"
    print vh

    Xnew = numpy.matmul(vh,X.T).T
    pca_pc = GetArrayToPC(Xnew)
    
    #Show the resulting point cloud

    utils.view_pc([pca_pc])

    #Rotate the points to align with the XY plane AND eliminate the noise

    S = numpy.diag(numpy.diag(s))
    
    threshold = 0.01

    vsh = numpy.copy(vh)
    reduce_dim = 0
    for s,i in zip(S,range(len(S))):
        if abs(s) < threshold:
            vsh[i,:] = 0
            reduced_dim = i
    print "----------------"
    print "Vs.T"
    print vsh

    X_reduced = numpy.matmul(vsh,X.T).T
    pca_reduced_pc = GetArrayToPC(X_reduced)    

    # Show the resulting point cloud

    utils.view_pc([pca_reduced_pc])

    # Draw plane to fit the original point cloud

    no = numpy.matrix('0.;0.;1.')
    no = numpy.matmul(vh.T,no)

    no = numpy.matrix(vh[reduced_dim,:])
    no = numpy.reshape(no,(3,1))

    utils.draw_plane(fig,no,mu,color=[0,1,0,0.3])

    ###YOUR CODE HERE###


    raw_input("Press enter to end:")


if __name__ == '__main__':
    main()
