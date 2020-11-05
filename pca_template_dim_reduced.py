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
    utils.view_pc([pc])

    #Rotate the points to align with the XY plane

    X = numpy.squeeze(numpy.array(pc),axis = 2)
    mu = numpy.mean(X,axis = 0)

    for i in range(X.shape[0]):
        X[i,:] = X[i,:]-mu
    N = X.shape[0]
    Q = numpy.matmul(X.T,X)/(N-1)   
    u,sigma,vh = numpy.linalg.svd(Q)

    S = numpy.diag(numpy.diag(sigma))

    threshold = 0.01

    vsh = vh
    for s,i in zip(S,range(len(S))):
        if abs(s) < threshold:
            vsh[i,:] = 0

    print vsh

    Xnew = numpy.matmul(vsh,X.T).T
    pca_pc = GetArrayToPC(Xnew)
    

    utils.view_pc([pca_pc])


    #Show the resulting point cloud


    #Rotate the points to align with the XY plane AND eliminate the noise


    # Show the resulting point cloud

    ###YOUR CODE HERE###


    raw_input("Press enter to end:")


if __name__ == '__main__':
    main()
