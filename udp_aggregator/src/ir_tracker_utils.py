import cv2
import numpy as np
from pyclustering.cluster.bsas import bsas
import yaml
from tqdm import tqdm

class markerExteractor(object):
    '''
    This is a class that extracts the pixel coordinate of the IR markers in the image. The hparams is
    a dataclass that holds the parameters for the blub detector and the thresholding fucntion.
    '''
    def __init__(self,hparams):
        self.hparams=hparams
        self.blubParams = cv2.SimpleBlobDetector_Params()
        self.blubParams.minThreshold = self.hparams.minThreshold
        self.blubParams.maxThreshold = self.hparams.maxThreshold
        self.blubParams.filterByArea = self.hparams.filterByArea
        self.blubParams.minArea = self.hparams.minArea
        self.blubParams.filterByCircularity = self.hparams.filterByCircularity
        self.blubParams.minCircularity = self.hparams.minCircularity
        self.blubParams.filterByConvexity = self.hparams.filterByConvexity
        self.blubParams.minConvexity = self.hparams.minConvexity
        self.blubParams.filterByInertia = self.hparams.filterByInertia
        self.blubParams.minInertiaRatio = self.hparams.minInertiaRatio
        self.blubParams.blobColor = self.hparams.blobColor
        ver = (cv2.__version__).split('.')
        if int(ver[0]) < 3 :
            self.blubDetector = cv2.SimpleBlobDetector(self.blubParams)
        else :
            self.blubDetector = cv2.SimpleBlobDetector_create(self.blubParams)
    def detect(self,frame):
        self.cms=[]
        self.image_ROIs=[]
        self.keypoints=[]
        img_gray=cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret,img_thresh = cv2.threshold(img_gray,100,255,cv2.THRESH_TOZERO)
        #Find the clusters
        self.nonzro_samples = cv2.findNonZero(img_thresh)
        if self.nonzro_samples is None:
            return None
        else:
            self.nonzro_samples=self.nonzro_samples.reshape(-1, 2).astype('float32')
        bsas_instance = bsas(self.nonzro_samples, self.hparams.max_clusters, self.hparams.threshold)
        bsas_instance.process()
        clusters = bsas_instance.get_clusters()
        #Calculate the center of the clusters and the Regions of Interests
        self.ROIs=np.zeros((len(clusters),4))
        for i,cluster in enumerate(clusters):
            current_batch=self.nonzro_samples[cluster]
            self.cms.append(np.sum(current_batch,axis=0)/current_batch.shape[0])
            row_max=np.max(current_batch[:,1],axis=0)+6
            row_min=np.min(current_batch[:,1],axis=0)-6
            col_max=np.max(current_batch[:,0],axis=0)+6
            col_min=np.min(current_batch[:,0],axis=0)-6
            self.ROIs[i,:]=[row_min,row_max,col_min,col_max]
        for roi in self.ROIs.astype('int32'):
            self.image_ROIs.append(img_thresh.copy()[roi[0]:roi[1],roi[2]:roi[3]])
        #Return The Results
        marker_points=[]
        for i,roi in enumerate(self.image_ROIs):
            keys_in_roi=self.blubDetector.detect(roi)
            for key in keys_in_roi:
                #Calculate the global coordinate of marker points. The points are returned in (X(Col),Y(Row)) coordinate.
                marker_points.append([key.pt[0]+self.ROIs.astype('float32')[i,2],key.pt[1]+self.ROIs.astype('float32')[i,0]])
        return np.array(marker_points)

    def extract_from_video(self,video_file):
        cap=cv2.VideoCapture(video_file)
        N_frames=int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        extracted_points=[]
        for i in tqdm(range(N_frames)):
            ret,img=cap.read()
            if(img is not None):
                points=self.detect(img)
                if (points is not None):
    #                 Add points to the list only if there are at least one and at most max_markers number of markers in the image
                    if (points.shape[0]<=self.hparams.max_markers) and (points.shape[0]>0):
                        extracted_points.append(points.squeeze())
                    else:
                        extracted_points.append(np.nan)
                else:
                    extracted_points.append(np.nan)
            else:
                extracted_points.append(np.nan)
        cap.release()
        return extracted_points

class undistrodMarkers:
    '''
    This class exteracts the camera calibration parameters stored in the config_file and converts the pixel
    coordinates if the original camera into that of the undistorded image.
    '''
    def __init__(self,config_file_name):
        with open(config_file_name, 'r') as f:
            calib = yaml.safe_load(f.read())
        self.K = np.array(calib['camera_matrix']['data']).reshape(calib['camera_matrix']['rows'],calib['camera_matrix']['cols'])
        self.D = np.array(calib['distortion_coefficients']['data']).reshape(-1, 5)
        self.P = np.array(calib['projection_matrix']['data']).reshape(3, 4)
        #elf.R = np.array(calib['rectification_matrix']['data']).reshape(3, 3)
#        self.img_width = calib['image_width']
#        self.img_height = calib['image_height']
    def process(self,points):
        lpts_ud=cv2.undistortPoints(points.reshape(-1,1,2).astype(np.float32), self.K, self.D,P=self.K)
        return np.squeeze(cv2.convertPointsToHomogeneous(np.float32(lpts_ud))).reshape(-1,3,1)
    def process_normalized(self,points):
        lpts_ud=cv2.undistortPoints(points.reshape(-1,1,2).astype(np.float32), self.K, self.D,P=None)
        return np.squeeze(cv2.convertPointsToHomogeneous(np.float32(lpts_ud))).reshape(-1,3,1)
    def undistort_point_list(self,point_list):
        undistorted_points=[]
        for point in point_list:
            if ~np.isnan(point).all():
                undistorted_points.append(self.process(point))
            else:
                undistorted_points.append(np.nan)
        return undistorted_points
    def undistort_point_list_normalized(self,point_list):
        undistorted_points=[]
        for point in point_list:
            if ~np.isnan(point).all():
                undistorted_points.append(self.process_normalized(point))
            else:
                undistorted_points.append(np.nan)
        return undistorted_points

def remove_invalid_pairs(list1,list2):
    extracted_items1=[]
    extracted_items2=[]
    num_items=min(len(list1),len(list2))
    for i in range(num_items):
        if ~np.isnan(list1[i]).any() and ~np.isnan(list2[i]).any():
            extracted_items1.append(list1[i])
            extracted_items2.append(list2[i])
    return extracted_items1,extracted_items2

# Class for estimating the essential matrix using the observed set of points in the two images.
#The data object is a list costructed as data=[img2_points,img1_points]
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
        for i in range(data[0].shape[0]):
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
    def fit_ransac(self,data, max_iters=100, samples_to_fit=8,inlierThreshold=None):
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
