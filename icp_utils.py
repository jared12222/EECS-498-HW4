import numpy as np

def euclidean(a,b):
    return np.sqrt(np.sum(np.square(a-b)))

def findclosest(P,Q):
    P_pair = []
    Q_pair = []
    for p in P:
        d_min = np.Infinity
        for q in Q:
            d = euclidean(p,q)
            if d < d_min :
                d_min = d
                q_min = q
        P_pair.append(p)
        Q_pair.append(q_min)
    return P_pair,Q_pair

def mean(pts):
    sum = np.matrix("0;0;0")
    for pt in pts:
        sum = sum + pt
    return sum/len(pts)

def weight_mean(Cp,Cq):
    w = []
    dist = []
    sum = 0
    for p,q in zip(Cp,Cq):
        dist.append(euclidean(p,q))
        sum = sum + dist[-1]
    avg = sum/len(Cp)

    for d in dist:
        w.append(abs(d-avg))
    w = np.array(w)/np.max(w)
    
    return np.sum(Cp,axis = 0)/np.sum(w), np.sum(Cq,axis=0)/np.sum(w), w

def GetTransform(Cp,Cq, weighted = True):
    
    if weighted:
        p_mean, q_mean , w = weight_mean(Cp,Cq)
    else:
        p_mean = mean(Cp)
        q_mean = mean(Cq)


    X = np.matrix(np.reshape(np.array(Cp).T,(3,len(Cp))))
    Y = np.matrix(np.reshape(np.array(Cq).T,(3,len(Cq))))
    X = X-p_mean
    Y = Y-q_mean
    if weighted:
        S = np.matmul(np.matmul(X,np.diag(w)),Y.T)
    else:
        S = np.matmul(X,Y.T)
    u,s,vT = np.linalg.svd(S)
    m = np.diag([1,1,np.linalg.det(np.matmul(vT.T,u.T))])
    R = np.matmul(np.matmul(vT.T,m),u.T)
    t = q_mean - np.matmul(R,p_mean)

    return R,t

def SumCorrespondError(R,t,C):
    sum = 0
    for i in range(len(C[0])):
        sum = sum + euclidean(R*C[0][i]+t,C[1][i])
    return sum

def updateP(R,t,P):
    p_new = []
    for p in P:
        p_new.append(R*p+t)
    return p_new