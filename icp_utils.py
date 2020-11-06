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
    return np.mean(pts,axis=0)

def GetTransform(Cp,Cq):
    
    
    p_mean = mean(Cp)
    q_mean = mean(Cq)

    X = np.matrix(np.reshape(np.array(Cp).T,(3,len(Cp))))
    Y = np.matrix(np.reshape(np.array(Cq).T,(3,len(Cq))))
    # print X[:,0]
    X = X-p_mean
    Y = Y-q_mean
    # print X[:,0]
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