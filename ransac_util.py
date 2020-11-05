import numpy as np
import copy

def norm2(a):
    return np.sqrt(np.sum(np.square(a)))

def plane_model(pts):
    n = len(pts)
    X = copy.copy(pts)
    X = np.squeeze(np.array(pts),axis = 2).T 
    mu = np.mean(X,axis=1)

    for i in range(n):
        X[:,i] = X[:,i] - mu
    Q = np.matmul(X,X.T)/(n-1)
    u,s,vh = np.linalg.svd(Q)
    min = np.Infinity
    for i in range(vh.shape[0]):
        if np.abs(s[i]) < min:
            min = np.abs(s[i])
            min_component = vh[i,:]
    

    return np.matrix(mu).T,np.matrix(min_component).T

# calculate error from plane model to pt
# pt: point
# model: [point, normal vector]
def error_plane(pt,model):
    v = pt-model[0]
    e = np.dot(v.T,model[1])/norm2(model[1])
    return abs(e)