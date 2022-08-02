import numpy as np
import cv2
import gtsam
from gtsam.symbol_shorthand import B, V, X, L


class EssentialMatrixEstimator():
    def __init__(self):
        self.estimation_mode=1
        self.inlier_threshold=0.07
    def create_row(self,point2,point1):
        u1=point1[0]
        v1=point1[1]
        u2=point2[0]
        v2=point2[1]
        line=np.array(\
        [u1*u2, u1*v2, u1, v1*u2, v1*v2, v1, u2, v2, 1]\
        ).reshape(1,9)
        return line
    def fit(self,data):
        rows=[]
        for i in range(len(data[0])):
            rows.append(self.create_row(data[0][i,:],data[1][i,:]))
        A=np.vstack(rows)
        u,s,v=np.linalg.svd(A)
        self.E=v.T[:,8].reshape(3,3).T
        u,s,v=np.linalg.svd(self.E)
        s[2]=0
        self.E=np.matmul(u * s, v)
        return self.E
    def get_performace(self,data):
        error=[]
        for i in range(len(data[0])):
            P1=np.hstack([data[0][i,:] ,1])
            P2=np.hstack([data[1][i,:] ,1])
            P1=P1.reshape(3,1)
            P2=P2.reshape(3,1)
            error.append(np.dot(np.dot(P2.T,self.E),P1))
        error=np.array(error).reshape(-1)
        cost=np.sum(error**2)
        return 1.0/cost
    def get_score(self,data,E):
        error=[]
        inliers=0
        for i in range(len(data[0])):
            P1=np.hstack([data[0][i,:] ,1])
            P2=np.hstack([data[1][i,:] ,1])
            P1=P1.reshape(3,1)
            P2=P2.reshape(3,1)
            dis=np.dot(np.dot(P2.T,E),P1)
            if np.abs(dis)<self.inlier_threshold:
                inliers=inliers+1
        return inliers
    def get_mask(self,data,E):
        mask=np.zeros((data[0].shape[0],1))
        for i in range(len(data[0])):
            P1=np.hstack([data[0][i,:] ,1])
            P2=np.hstack([data[1][i,:] ,1])
            P1=P1.reshape(3,1)
            P2=P2.reshape(3,1)
            dis=np.dot(np.dot(P2.T,E),P1)
            if np.abs(dis)<self.inlier_threshold:
                mask[i]=1
        return mask
    def fit_ransac(self,data, max_iters=100, samples_to_fit=8, inlierThreshold=None):    
        best_model = None
        max_number_of_inliers = 0
        if inlierThreshold is not None:
            self.inlier_threshold=inlierThreshold

        num_samples = data[0].shape[0]
        progress=[]
        for i in range(max_iters):
            samples = np.random.choice(num_samples, size=samples_to_fit, replace=False)
            model_params = self.fit([data[0][samples,:],data[1][samples,:]])
            number_of_inliers = self.get_score(data,model_params) #How many inliers for
                                                                  #the given model?
            if number_of_inliers > max_number_of_inliers:
                best_model = model_params
                max_number_of_inliers = number_of_inliers
                progress.append(number_of_inliers)
        return progress,best_model


class StereoGeometricalCalibrator():
    def __init__(self, cam_data_manager):
        self.cam_dm = cam_data_manager
        self.extrinsics = {}
        self.ess=EssentialMatrixEstimator()
        
    def getSkewMat(self, vec):
        return np.array([0, -vec[2], vec[1], vec[2], 0, -vec[0], -vec[1], vec[0], 0]).reshape(3,3)
    
    def getEssentialError(self, points1, points2, E):
        error=[]
        for i in range(points1.shape[0]):
            P1=np.hstack([points1[i,:] , 1.0])
            P2=np.hstack([points2[i,:] , 1.0])
            P1=P1.reshape(3,1)
            P2=P2.reshape(3,1)
            error.append(np.dot(np.dot(P2.T,E),P1))
        error=np.array(error).reshape(-1)
        return error
    
    def decomposeEandVerify(self, camiModel, camjModel, E, pi , pj ):
        '''
        Decompose the E matrix based on a modified verison of the rutine explained in: 
        https://stackoverflow.com/questions/22807039/decomposition-of-essential-matrix-validation-of-the-four-possible-solutions-for
        '''
        
        u,s,vt=np.linalg.svd(E)
        if np.linalg.det(u) < 0:
            u = -u
            
        if np.linalg.det(vt) < 0:
            vt = -vt
            
        W=np.array([0,-1,0,1,0,0,0,0,1],dtype='float32').reshape(3,3)
        R = u @ W @ vt
        t = u[...,-1].reshape(3,1)
        #Triangulate a sample point
        P1=np.dot(camiModel.P[0:3,0:3],np.hstack([np.eye(3),np.zeros((3,1))]))
        P2=np.dot(camjModel.P[0:3,0:3],np.hstack([R,t]))
        X1=cv2.triangulatePoints(P1,P2,pi[1000].reshape(-1,1,2),pj[1000].reshape(-1,1,2))
        X1=X1/X1[-1]
        X2=np.hstack([R,t])@X1
        #Are both at the back of the camera?
        if X1[2] * X2[2] < 0:
            print('the product turned out negative')
            R = u @ W.T @ vt
            #Triangulate with the fixed R
            P1=np.dot(camiModel.P[0:3,0:3],np.hstack([np.eye(3),np.zeros((3,1))]))
            P2=np.dot(camjModel.P[0:3,0:3],np.hstack([R,t]))
            X1=cv2.triangulatePoints(P1,P2,pi[1000].reshape(-1,1,2),pj[1000].reshape(-1,1,2))
            X1=X1/X1[-1]
        #is at the back again?   
        if X1[2] < 0:
            t = -t
            #Update the P1 and P2
            P1=np.dot(camiModel.P[0:3,0:3],np.hstack([np.eye(3),np.zeros((3,1))]))
            P2=np.dot(camjModel.P[0:3,0:3],np.hstack([R,t]))
        #Triangulate all the points
        points=cv2.triangulatePoints(P1,P2,pi.reshape(-1,1,2),pj.reshape(-1,1,2))
        points_normalized=np.vstack([points[:,i]/points[-1,i] for i in range(len(points.T))])
        return camiModel.P[0:3,0:3], camjModel.P[0:3,0:3], P1, P2, R, t, points_normalized

        
    def getExtrinsics(self, cam_i, cam_j):
        #Do not recalculate the extrinsics multiple times. 
        # try:
        #     return self.extrinsics[f'{cam_i},{cam_j}'] + (p1_pix, p2_pix)
        # except:
        p1, p2 = self.cam_dm.getCovisibleMarkerPairs(cam_i, cam_j, undist= True ,normalized=True)
        p1_pix, p2_pix = self.cam_dm.getCovisibleMarkerPairs(cam_i, cam_j, undist=True, normalized=False)
        data=[p2.copy(),p1.copy()]
        E = self.ess.fit(data)
        error = self.getEssentialError(p1, p2, E)
        K1, K2, P1, P2, R, t, points_normalized = \
                                                self.decomposeEandVerify(self.cam_dm.camModels[cam_i],
                                                                         self.cam_dm.camModels[cam_j],\
                                                                         E, 
                                                                         p1_pix, 
                                                                         p2_pix)
        self.extrinsics[f'{cam_i},{cam_j}'] = (K1, K2, E, P1, P2, R, t, points_normalized, error)
        return self.extrinsics[f'{cam_i},{cam_j}'] + (p1_pix, p2_pix)


class StereoBundleAdjustment():
    def __init__(self, R_init, t_init, Ki, Kj):
        self.Ki = gtsam.Cal3_S2(Ki[0,0], Ki[1,1], 0, Ki[0,-1], Ki[1,-1])
        self.Kj = gtsam.Cal3_S2(Kj[0,0], Kj[1,1], 0, Kj[0,-1], Kj[1,-1])
        self.camiNoise = gtsam.noiseModel.Isotropic.Sigmas( np.array([0.1, 0.1]) )
        self.camjNoise = gtsam.noiseModel.Isotropic.Sigmas( np.array([0.01, 0.01]) )
        self.priorPoseNoise =  gtsam.noiseModel.Isotropic.Sigma(6,1e-7)
        self.relativePoseNoise = gtsam.noiseModel.Isotropic.Sigma(6,0.001)
        self.R_init = R_init
        self.t_init = t_init
        
    def addUnaryPosePrior(self, graph, T, noise, key):
        graph.push_back(gtsam.PriorFactorPose3(key, T, noise))         

    def addProjectionFactor(self, graph, K, z, noise, camera_key, landmark_key):
        self.F = gtsam.GenericProjectionFactorCal3_S2( z.reshape(2,1),noise, camera_key, landmark_key, K)
        graph.push_back(self.F)
        
    def addRelativeExtrinsics(self, graph, cam_i, cam_j, R, t, noise):
        Pose = gtsam.Pose3(gtsam.Rot3(R), t)
        graph.push_back(gtsam.BetweenFactorPose3(cam_i, cam_j, Pose, noise))
    
    def run(self, pi, pj, init_poses):
            graph = gtsam.NonlinearFactorGraph()
            initial = gtsam.Values()
            num_of_landmarks = pi.shape[0]
            # In GTSAM, the pose assosiated with each camera is the transformation of the camera with respect the the world
            # However, the R and T that we extract using the geometrica method represent the transformation of the world (reference camera)
            # with respect to the camera (second camera). That is why in what follows, we feed the inverse of this transformation to the GTSAM
            self.addUnaryPosePrior(graph, gtsam.Pose3(), self.priorPoseNoise, X(0))
#             self.addUnaryPosePrior(graph, gtsam.Pose3(gtsam.Rot3(self.R_init), self.t_init).inverse(), self.priorPoseNoise, X(1))

            initial.insert(X(0), gtsam.Pose3())
            initial.insert(X(1), gtsam.Pose3(gtsam.Rot3(self.R_init), self.t_init).inverse())
            
            #Add the transformation of the X(1) with respect to X(0) as a constraint to the graph
            self.addRelativeExtrinsics(graph, X(1), X(0), self.R_init, self.t_init, self.relativePoseNoise)

            for n in range(num_of_landmarks):
                self.addProjectionFactor(graph, self.Ki, pi[n,:], self.camiNoise, X(0), L(n))
                self.addProjectionFactor(graph, self.Kj, pj[n,:], self.camjNoise, X(1), L(n))
                initial.insert(L(n), gtsam.Point3(x=init_poses[n,0], y=init_poses[n,1], z=init_poses[n,2]))
            print(self.F.error(initial))

#             params = gtsam.LevenbergMarquardtParams()
            params = gtsam.GaussNewtonParams()
#             optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial, params)
            optimizer = gtsam.GaussNewtonOptimizer(graph, initial, params)
            result = optimizer.optimize()
            self.marginals = gtsam.Marginals(graph, result)

            i = 0
            self.L = []
            #Extract the new extrinsics and construct the new projection matrices
            self.Ri = result.atPose3(X(0)).inverse().matrix()[0:3,0:3].reshape(3,3)
            self.ti = result.atPose3(X(0)).inverse().matrix()[0:3,-1].reshape(3,1) 
            self.Pi = self.Ki.K()@np.hstack([self.Ri, self.ti])
            self.Xi = result.atPose3(X(0)).matrix()
            self.Xi_cov = self.marginals.marginalCovariance(X(0))

            self.Rj = result.atPose3(X(1)).inverse().matrix()[0:3,0:3].reshape(3,3)
            self.tj = result.atPose3(X(1)).inverse().matrix()[0:3,-1].reshape(3,1) 
            self.Pj = self.Kj.K()@np.hstack([self.Rj, self.tj])
            self.Xj = result.atPose3(X(1)).matrix()
            self.Xj_cov = self.marginals.marginalCovariance(X(1))
            
            print('Extrinsics Before:')
            print(gtsam.Pose3(gtsam.Rot3(self.R_init.T), -self.R_init.T@self.t_init).matrix())
            print('Extrinsics After:')
            print(gtsam.Pose3(gtsam.Rot3(self.Rj), self.tj).matrix())
            #Extract the resutls (landmarks and poses)
            while result.exists(L(i)):
                self.L.append(result.atPoint3(L(i)))
                i += 1
            self.L = np.stack(self.L)
            
            self.P1=np.dot(self.Ki.K(),np.hstack([np.eye(3),np.zeros((3,1))]))
            self.P2=np.dot(self.Kj.K(),np.hstack([self.Rj,self.tj]))
