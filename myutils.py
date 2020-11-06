import numpy as np
import copy

def norm2(a):
    return np.sqrt(np.sum(np.square(a)))

def error_plane(pt,model):
    v = pt-model[0]
    e = np.dot(v.T,model[1])/norm2(model[1])
    return abs(e)

def pca(pts):
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

def plane_fitting(pts):
    pt = []
    picked = []
    while len(pt) < 3:
        n = np.random.randint(0,len(pts))
        if n not in picked :
            picked.append(n)
            pt.append(pts[n])
    pt_central = pt[0]
    plane_normal = np.cross(np.reshape(np.array(pts[1]-pts[0]),(3,)),np.reshape(np.array(pts[2]-pts[0]),(3,)))
    plane_normal = np.reshape(np.matrix(plane_normal),(3,1))

    return pt_central, plane_normal

def ransac(pts,iteration=200,lower_bound=100,N_to_fit = 3,epson=0.07):
    N_sample = len(pts)
    picked = set()
    error_best = np.Infinity
    model = None    
    for i in range(iteration):
        picked.clear()
        pt_pick = []
        while len(picked) < N_to_fit:
            n = np.random.randint(0,N_sample)
            if n not in picked:
                picked.add(n)
                pt_pick.append(pts[n])
        
        plane_org, plane_normal = plane_fitting(pt_pick)

        for j in range(N_sample):
            if j not in picked:
                if error_plane(pts[j],[plane_org,plane_normal]) <= epson:
                    picked.add(j)
                    pt_pick.append(pts[j])
        if len(picked) >= lower_bound:
            plane_org, plane_normal = pca(pt_pick)
            error_new = 0
            for j in range(len(pt_pick)):
                error_new = error_new + error_plane(pt_pick[j],[plane_org, plane_normal])
            error_new = error_new/len(pt_pick)
            if error_new < error_best:
                error_best = error_new
                model = [plane_org, plane_normal]
    return model
