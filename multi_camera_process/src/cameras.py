import pickle
import cv2
import numpy as np
import yaml
import os


#A data manager class to hold the marker data and thier timestamps plus the parameters of the camera and a set of util functions
class cameraDataManager():
    def __init__(self, data_file, params_path, ports=[5000, 5001, 5002], data_idx = 1):
#         data_idx = 1 #Data idx is the index of the markers locations in the data.values() items
        self.ports = ports
        self.params_path = params_path
        #Load the maker pixel locations from the cameras
        with open(data_file,'rb') as f:
            self.data = pickle.load(f)
            
#         print(self.data[5003])
        #How many cameras do we have per each port?
        self.cams_per_port = [len(self.data[port][1][data_idx]) for port in self.ports]
#         self.cams_per_port = [2, 2, 1, 1]
        #Extract the timestamps per for each port
#         self.stamps_per_port = {port:np.array([d[0] for d in self.data[port]]) for port in self.ports}
        self.stamps_per_port = {port:np.array([d[1]/1000000.0 for d in self.data[port]]) for port in self.ports}
        #Extract the timestamps for each camera
        self.stamp_per_camera = {}
        for port, n_cams in zip(self.stamps_per_port.keys(), self.cams_per_port):
            for n in range(n_cams):
                key = f'{port}-{n}'
                self.stamp_per_camera[key] = self.stamps_per_port[port]
        #Load Parameters
        names = []
        keys = []
        for port, n in zip(self.ports,self.cams_per_port): 
            names += [f'{port}-{i}.yaml' for i in range(n)]
            keys += [f'{port}-{i}' for i in range(n)]
        self.camModels = {key:RadTanPinholeCamera(os.path.join(self.params_path, name)) for name,key in zip(names,keys)}
        #Extract pixel locations
        self.pixel_locations = {}
        for i, key in enumerate(self.ports):
            for j in range(self.cams_per_port[i]):
                #The detection is valid only if there is one marker in the frame
        #         path=np.vstack([v[3][i] if v[3][0].shape[0]==1 else np.array([-1,-1]).reshape(1,2) for v in data[key]] ).squeeze()
                pixel_traj = np.vstack([v[data_idx][j][0] for v in self.data[key]] ).squeeze() #My own code
                #flag the undetected frames with [0,0] dtections
                idx=np.where(pixel_traj == [-1,-1])
#                 path[idx,:]=-np.ones((1,2))
                key_name = f'{key}-{j}'
                self.pixel_locations[key_name] = pixel_traj
        # Undistort the raw pixel locations and store them for later usage
        self.pixel_locations_udist = {}
        for traj_key in self.pixel_locations.keys():
            idx=np.where(self.pixel_locations[traj_key]==[-1,-1])[0]
            self.pixel_locations[traj_key][idx,:]=-np.ones((1,2))
            self.pixel_locations_udist[traj_key] = self.camModels[traj_key].undist(self.pixel_locations[traj_key])[:,0:2]
            self.pixel_locations_udist[traj_key][idx,:]=-np.ones((1,2))
        self.syncMaps = {}
        
    def syncMap(self, cam_i, cam_j):
        try:
            #Use the cached map if it's computed before
            map = self.syncMaps[f'{cam_i}->{cam_j}']
        except:
            if self.stamp_per_camera[cam_i].shape[0] > self.stamp_per_camera[cam_j].shape[0]:
                N = self.stamp_per_camera[cam_i].shape[0]
            else:
                N = self.stamp_per_camera[cam_j].shape[0]
                
            self.syncMaps[f'{cam_i}->{cam_j}'] = self.getMap(self.stamp_per_camera[cam_i][:N], 
                                                                self.stamp_per_camera[cam_j][:N])
            map = self.syncMaps[f'{cam_i}->{cam_j}']
        return map
    
    def getSynchronizedSamples(self, ref_cam, undist = True):
        keys = [key for key in self.pixel_locations.keys()]
        maps = {f'{ref_cam}->{key}':self.syncMap(ref_cam, key) for key in keys}

        #The returned sequence can not be longer than the shortest camera stream, so truncate to this lengths       
        N = min([map.shape[0] for map in maps.values()])
        maps = {key:maps[key][:N] for key in maps.keys()}
        #Generate the synchronized pixel locations using the map files
        if undist:
            synchronized_paths = {key:self.pixel_locations_udist[key][maps[f'{ref_cam}->{key}'][:,1], :] for key in keys}
        else:
            synchronized_paths = {key:self.pixel_locations[key][maps[f'{ref_cam}->{key}'][:,1], :] for key in keys}

        return synchronized_paths
    
    def getSynchronizedStamps(self, ref_cam):
        keys = [key for key in self.pixel_locations.keys()]
        maps = {f'{ref_cam}->{key}':self.syncMap(ref_cam, key) for key in keys}
        #The returned sequence can not be longer than the shortest camera stream, so truncate to this lengths       
        N = min([map.shape[0] for map in maps.values()])
        maps = {key:maps[key][:N] for key in maps.keys()}
        #Generate the synchronized pixel locations using the map files
        stamps = {key:self.stamp_per_camera[key][maps[f'{ref_cam}->{key}'][:,1]] for key in keys}
        return stamps
    
    def generateVideo(self, markers, camera_id, save_path, undist = True):
        size = self.camModels[camera_id].size
        K = self.camModels[camera_id].K
        P = self.camModels[camera_id].P
        D = self.camModels[camera_id].D
        
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        vid = cv2.VideoWriter(save_path, fourcc, 100, size)
        frame_counter=0;
        for point in tqdm(markers.tolist()):
            img1=np.zeros((size[1],size[0],3)).astype(np.uint8)
            if point[0] != -1:
                cv2.circle(img1,(int(round(point[0])), int(round(point[1]))), 3, (255,255,255), -1)
            if undist:
                img1=cv2.undistort(img1, K, D, None, P)
            vid.write(img1)
    
    def generateSynchedVideos(self, cam_i, cam_j, save_path, undist = False):
        map = self.syncMap(cam_i, cam_j)        
        markers_i = self.pixel_locations[cam_i][map[:,0],:]
        markers_j = self.pixel_locations[cam_j][map[:,1],:]
        self.generateVideo(markers_i, cam_i, os.path.join(save_path, f'vid_{cam_i}_{cam_j}:{cam_i}.avi'),undist)
        self.generateVideo(markers_j, cam_j, os.path.join(save_path, f'vid_{cam_i}_{cam_j}:{cam_j}.avi'),undist)
        
    def getCovisibleMarkerPairs(self, cam_i, cam_j, undist = False, normalized = True):
        #Get the correspounding frames
        map = self.syncMap(cam_i, cam_j) 
        markers_i = self.pixel_locations[cam_i][map[:,0],:]
        markers_j = self.pixel_locations[cam_j][map[:,1],:]
        #Find where markers are not visible and get instances where both cameras see the marker
        both_visible_idx = np.where( np.logical_and( (markers_i!=[-1,-1])[:,0], (markers_j!=[-1,-1])[:,0]) )[0]
        if undist:
            if normalized:
                markers_i = self.camModels[cam_i].undistNormal(markers_i)[both_visible_idx,:-1]
                markers_j = self.camModels[cam_j].undistNormal(markers_j)[both_visible_idx,:-1]
            else:
                markers_i = self.camModels[cam_i].undist(markers_i)[both_visible_idx,:-1]
                markers_j = self.camModels[cam_j].undist(markers_j)[both_visible_idx,:-1]
            return markers_i, markers_j
        else:
            return markers_i[both_visible_idx,:],markers_j[both_visible_idx,:]
        
    
    def getMap(self, timestamps_first, timestamps_second, threshold=0.15, add_nans=False): 
        '''
        This function gets two list of time stamps and returns a list of synchronizing maps
        [ ...[first_index,corresponding_synchronizing_second_index]...]. if there are no indices
        in the second timestamp lists that is close enough to the indices in the first list (dt<threshold),
        nan will be used to indicate the situation.
        '''
        N1 = timestamps_first.shape[0]
        N2 = timestamps_second.shape[0]
        
        if N1 < N2:
            N = N1
            p1 = timestamps_first
            p2 = timestamps_second
            reverse = False
        else:
            N = N2
            p1 = timestamps_second
            p2 = timestamps_first
            reverse = True

        map_list=[]
        for i in range(N):
            corresponding_second_index=np.argmin(np.abs(p2 -p1[i]))
            min_dt=np.min(np.abs(p2 -p1[i]))
            if min_dt<threshold:
                map_list.append((i,corresponding_second_index))
            elif add_nans:
                 map_list.append((i,np.nan))
        maps = np.array(map_list)
        if reverse:
            return np.vstack([maps[:,1],maps[:,0]]).T
        else:
            return maps

class RadTanPinholeCamera:
    def __init__(self, intrinsics_yaml):
        #Load the camera calibration yaml file obtained using the ROS calibration tool
        with open(intrinsics_yaml, 'r') as f:
            calib = yaml.safe_load(f.read())
        #Extract the important paramters from the calibration file
        # Camera Matrix
        self.K = np.array(calib['camera_matrix']['data']).reshape(calib['camera_matrix']['rows'],calib['camera_matrix']['cols'])
        # RadTan Distorsion Parameters
        self.D = np.array(calib['distortion_coefficients']['data']).reshape(-1, 5)
        # Reprojection Matrix
        self.P = np.array(calib['projection_matrix']['data']).reshape(3, 4)
        # Camera Resolution
        self.size = (calib['image_width'], calib['image_height'])
        
    def undist(self,points):
        lpts_ud=cv2.undistortPoints(points.reshape(-1,1,2).astype(np.float32), self.K, self.D, P=self.P)
        return np.squeeze(cv2.convertPointsToHomogeneous(np.float32(lpts_ud)))
    
    def undistNormal(self,points):
        lpts_ud=cv2.undistortPoints(points.reshape(-1,1,2).astype(np.float32), self.K, self.D, P=None)
        return np.squeeze(cv2.convertPointsToHomogeneous(np.float32(lpts_ud)))
        
class StereoCamera:
    def __init__(self, cam1_yaml, cam2_yaml, P1, P2):
        #Load the camera calibration yaml file obtained using the ROS calibration tool
        self.cam1 = RadTanPinholeCamera(cam1_yaml)
        self.cam2 = RadTanPinholeCamera(cam2_yaml)
        #Reprojection matrices from the extrinsic calibration rutine
        self.P1 = P1
        self.P2 = P2
        
    def triangulate(self, x1, x2, undist = False):
        if undist:
            x1_indist = self.cam1.undist(x1.reshape(-1,1,2))
            x2_indist = self.cam2.undist(x2.reshape(-1,1,2))
        else:
            x1_undist = x1
            x2_undist = x2
        landmarks = cv2.triangulatePoints(self.P1, self.P2,x1_undist[...,0:2].reshape(-1,1,2),
                                                           x2_undist[...,0:2].reshape(-1,1,2))
        return (landmarks/landmarks[-1,:]).T.squeeze()
    
    def reproject(self, landmarks):
        #Reproject the landmark on the first camera
        x1_reprojected = self.P1 @ landmarks.T.reshape(4,-1)
        x1_reprojected /= x1_reprojected[-1,:]
        #Reproject the landmark on the second camera
        x2_reprojected = self.P2 @ landmarks.T.reshape(4,-1)
        x2_reprojected /= x2_reprojected[-1,:]
        return x1_reprojected.T, x2_reprojected.T
    
    def undist(self, x1, x2):
        x1_undist = self.cam1.undist(x1)
        x2_undist = self.cam2.undist(x2)
        return x1_undist, x2_undist
