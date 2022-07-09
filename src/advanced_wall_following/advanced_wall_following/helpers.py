import random
import numpy as np
from numpy.linalg import norm

def ransac(points,maxIter,t,d):
    i = 0
    bestLine = None
    bestInliers = list()
    bestOutliers = list()
    random.seed()
    while i < maxIter:
        i = i+1;
        idx1 = random.randrange(len(points))
        idx2 = random.randrange(len(points))
        while idx2 == idx1:
            idx2 = random.randrange(len(points));
        
        #model
        B = points[idx1]
        C = points[idx2]
        inliers = list()
        outliers = list()
        for j  in range(0, len(points)):
            if j == idx1 or j == idx2:
                continue
            A = points[j]
            dist = point2lineDist(A,B,C)
            # print(dist)
            if abs(dist) <= t:
                inliers.append(A)
            else:
                outliers.append(A)
        if len(inliers) >= d:
            np.polyfit(inliers[0], inliers[1], 1)
            bestInliers = bestInliers
            bestOutliers = bestOutliers
            break
    
    return bestLine, bestInliers, bestOutliers

def point2lineDist(A,B,C):
    #A is the point to "project", B and C are the points that define the
    return norm(np.cross(C-B, B-A))/norm(C-B)

