import cv2, sys, time, copy, rospy, PySpin
from copy import deepcopy
import numpy as np
from numpy.linalg import inv
from PIL import Image
from sklearn.cluster import DBSCAN
from sklearn.neighbors import LocalOutlierFactor
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from std_msgs.msg import Char
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
import gc


class Vision:
    
    '''Class init function'''
    def __init__(self):
        
        # Class Variable Initialization
        self.pattern_former_state = False
        self.dir_ = 'C:/catkin_ws/src/ScanSewing/scan_sewing_vision/images/'
        self.exposure_time = 30000 #orange:30000
        self.stitch_length = 3.0
        self.cb_pos = np.array([-1.0,-1.0])
        self.A = np.identity(2)
        self.B = np.zeros([2,1])
        self.pf_pos = -np.ones([2,3])
        self.sl_to_ts = 1.588
        self.area_num = 1
        self.capture_image_num = 2
        self.rt_image = False
        self.capture_num = 48
        
        self.pf_pos[0][0] = 590.0
        self.pf_pos[1][0] = 50.0
        self.pf_pos[0][1] = 710.0
        self.pf_pos[1][1] = 50.0
        
        self.system = PySpin.System.GetInstance()
        self.cam_list = self.system.GetCameras()
        num_cameras = self.cam_list.GetSize()
        
        if num_cameras == 0:
            self.cam = None       
            self.cam_list.Clear()
            self.system.ReleaseInstance()
            print('Camera is not connected!')
        else :
            self.cam = self.cam_list[0]
            self.image = None
    
    def __del__(self):
        self.release_camera()
    
            
    ''' Release Camera'''
    def release_camera(self):
        del self.cam
        self.cam_list.Clear()
        self.system.ReleaseInstance()   
        print('Releasing Camera...')
        
    ''' Run Camera '''
    def run_camera(self,exposure_time):
        try:    
            self.cam.Init()
            result = True
    
            if not self.configure_exposure(exposure_time):
                return False
    
            result &= self.acquire_images()
            result &= self.reset_exposure()
    
            self.cam.DeInit()
            return result
    
        except PySpin.SpinnakerException as ex:
            print('11111')
            print('Error: %s' % ex)
            self.release_camera()
            return False
    
    ''' Set Camera Exposure Time '''
    def configure_exposure(self,exposure_time):
        try:
            result = True
    
            if self.cam.ExposureAuto.GetAccessMode() != PySpin.RW:
                print('Unable to disable automatic exposure. Aborting...')
                return False
    
            self.cam.ExposureAuto.SetValue(PySpin.ExposureAuto_Off)
    
            if self.cam.ExposureTime.GetAccessMode() != PySpin.RW:
                print('Unable to set exposure time. Aborting...')
                return False
    
            exposure_time_to_set = exposure_time
            exposure_time_to_set = min(self.cam.ExposureTime.GetMax(), exposure_time_to_set)
                
            self.cam.ExposureTime.SetValue(exposure_time_to_set)
            # print('Shutter time set to %s us...' % exposure_time_to_set)
    
        except PySpin.SpinnakerException as ex:
            print('2')
            print('Error: %s' % ex)
            result = False
    
        return result
    
    ''' Capture Images from Camera '''
    def acquire_images(self):
        try:
            result = True
    
            if self.cam.AcquisitionMode.GetAccessMode() != PySpin.RW:
                print('Unable to set acquisition mode to continuous. Aborting...')
                return False
    
            self.cam.AcquisitionMode.SetValue(PySpin.AcquisitionMode_Continuous)
            self.cam.BeginAcquisition()
    
            timeout = 0
            if self.cam.ExposureTime.GetAccessMode() == PySpin.RW or self.cam.ExposureTime.GetAccessMode() == PySpin.RO:
                timeout = (int)(self.cam.ExposureTime.GetValue() / 1000 + 1000)
            else:
                print ('Unable to get exposure time. Aborting...')
                return False
            
            try:
                image_result = self.cam.GetNextImage(timeout)

                if image_result.IsIncomplete():
                    print('Image incomplete with image status %d...' % image_result.GetImageStatus())

                else:
                    image_converted = image_result.Convert(PySpin.PixelFormat_Mono8)
                    # image_converted.Save('C:\catkin_ws\src\ScanSewing\scan_sewing_vision\images\image1.bmp')
                    self.image = image_converted
                    # self.image.Save('C:\catkin_ws\src\ScanSewing\scan_sewing_vision\images\image_test.bmp')
                    
                    # filename = 'ET_%s.bmp' % (self.cam.ExposureTime.GetValue())
                    # image_converted.Save(filename)

                image_result.Release()

            except PySpin.SpinnakerException as ex:
                print('3')
                print('Error: %s' % ex)
                result = False
    
            self.cam.EndAcquisition()
    
        except PySpin.SpinnakerException as ex:
            print('4')
            print('Error: %s' % ex)
            result = False
    
        return result
    
    ''' Reset Exposure to Auto '''
    def reset_exposure(self):
        try:
            result = True
    
            if self.cam.ExposureAuto.GetAccessMode() != PySpin.RW:
                print('Unable to enable automatic exposure (node retrieval). Non-fatal error...')
                return False
    
            self.cam.ExposureAuto.SetValue(PySpin.ExposureAuto_Continuous)
            #print('Automatic exposure enabled...')
    
        except PySpin.SpinnakerException as ex:
            print('5')
            print('Error: %s' % ex)
            result = False
    
        return result   
    
    ''' Make Chessboard Image Clear by Adaptive Thresholding'''
    def chessboard_threshold(self):
        try:
            img = cv2.imread(self.dir_+'ui_calibration.bmp')
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray,(5,5),0)
            # ret,th2 = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
            area = (int)(gray.shape[0]/2)
            if area%2 == 0:
                area = area+1
            th2 = cv2.adaptiveThreshold(blur,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,area,10)
            cv2.imwrite(self.dir_+'ui_cb_threshold.bmp',th2)
            return th2 
        except:
            pass
    
    ''' Calculate Calibration Data  '''
    def chessboard_calibration(self,_img,_x=502.0,_y=213.0):
        try:
            # Save Pattern Former XY Position 
            # if self.cb_pos[0]!=-1:
            #     _x = self.cb_pos[0]
            #     _y = self.cb_pos[1]
                
            # Chessboard Detection
            h = 9
            w = 13
            
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            objp = np.zeros((h*w,3), np.float32)
            objp[:,:2] = np.mgrid[0:w,0:h].T.reshape(-1,2)
            objpoints = []
            imgpoints = []
            ret, corners = cv2.findChessboardCorners(_img, (w,h), None)
    
            if ret==True:
                objpoints.append(objp)
                corners2 = cv2.cornerSubPix(_img, corners, (11,11), (-1,-1), criteria)
                imgpoints.append(corners2)
                origin_img = cv2.imread(self.dir_+'ui_cb_threshold.bmp')
                cv2.drawChessboardCorners(origin_img, (w,h), corners2, ret)
                cv2.imwrite(self.dir_+'ui_calibration.bmp',origin_img)
                time.sleep(0.1)
                self.vision_image_pub.publish(49)
                
                # Find Linear Coordinate Transformation Matrix (y = A*x + B) 
                v_p = np.array([[_x],[_y]])
                v_t = np.array([[300],[-501.5]])
                w_off = 32.0
                h_off = 27.2
                q_1 = np.array([[-w_off - 19.433*(w-1)],[h_off + 19.45*(h-1) ]])
                q_2 = np.array([[-w_off],[q_1[1]]])
                q_3 = np.array([[q_1[0]],[h_off]])
                p_1 = q_1+v_p+v_t
                p_2 = q_2+v_p+v_t
                p_3 = q_3+v_p+v_t
                
                for i in [0,w-1,h*w-w,h*w-1]:
                    if corners2[i][0][0] < 2048 and corners2[i][0][1] < 1500:
                        w_3 = corners2[i].T
                    elif corners2[i][0][0] < 2048 and corners2[i][0][1] > 1500:
                        w_1 = corners2[i].T
                    elif corners2[i][0][0] > 2048 and corners2[i][0][1] > 1500:
                        w_2 = corners2[i].T
                
                w_tot = np.zeros([2,2])
                w_tot[0][0] = (w_2-w_1)[0][0]
                w_tot[1][0] = (w_2-w_1)[1][0]
                w_tot[0][1] = (w_3-w_1)[0][0]
                w_tot[1][1] = (w_3-w_1)[1][0]
                p_tot = np.zeros([2,2])
                p_tot[0][0] = (p_2-p_1)[0][0]
                p_tot[1][0] = (p_2-p_1)[1][0]
                p_tot[0][1] = (p_3-p_1)[0][0]
                p_tot[1][1] = (p_3-p_1)[1][0]
                
                self.A = np.dot(p_tot,inv(w_tot))
                self.B = p_1 - np.dot(self.A,w_1)
                
                # Save Calibration Data in Text File
                with open("C:/catkin_ws/src/ScanSewing/scan_sewing_vision/calibration_data/CB_data.txt", "w") as file:
                    file.write(str(self.A[0][0]))
                    file.write("\n")
                    file.write(str(self.A[0][1]))
                    file.write("\n")
                    file.write(str(self.A[1][0]))
                    file.write("\n")
                    file.write(str(self.A[1][1]))
                    file.write("\n")
                    file.write(str(self.B[0][0]))
                    file.write("\n")
                    file.write(str(self.B[1][0]))
                    file.close() 
                self.vision_check_pub.publish(50)
            else:
                self.vision_error_pub.publish(49)
                print("Calibration failed")
        except:
            pass
        
    ''' Callback Functions '''
            
    ''' Pattern Former Connection Callback '''
    def connectCallback(self,data):
        if self.cam is None :
            self.vision_error_pub.publish(50)
        else:
            try:
                self.rt_image = True
                # Read Calibration Data
                with open("C:/catkin_ws/src/ScanSewing/scan_sewing_vision/calibration_data/CB_data.txt", "r") as file:
                    lines = file.readlines()
                    self.A[0][0] = float(lines[0])
                    self.A[0][1] = float(lines[1])
                    self.A[1][0] = float(lines[2])
                    self.A[1][1] = float(lines[3])
                    self.B[0][0] = float(lines[4])
                    self.B[1][0] = float(lines[5])
            except:
                self.vision_error_pub.publish(48)
    
    ''' Exposure Time Callback '''
    def visionExposuretimeCallback(self,data):
        self.exposure_time = data.data
    
    ''' Offset(SeamLine - Topstitch Line) Callback '''
    def visionOffsetCallback(self,data):
        self.sl_to_ts = data.data
        # print(self.sl_to_ts)
    
    def visionStitchLengthCallback(self,data):
        self.stitch_length = data.data
    
    ''' Pattern Former Position (Calibration) Callback '''
    def visionCalibrationCallback(self,data):
        self.cb_pos[0] = data.x
        self.cb_pos[1] = data.y   
    
    ''' Seam Line Candidate Area Number Callback '''
    def visionAreaNumCallback(self,data):
        self.area_num = data.data
        self.vision_area_num_pub.publish(self.area_num)
        self.image_processing()
        
    ''' Number of Capture Images Callback ''' 
    def visionCaptureImageNumCallback(self,data):
        self.capture_image_num = data.data
        
    ''' Waypoint 1 Pattern Former Position Callback '''
    def visionWaypoint1Callback(self,data):
        self.pf_pos[0][0] = data.x
        self.pf_pos[1][0] = data.y
        
    ''' Waypoint 2 Pattern Former Position Callback '''
    def visionWaypoint2Callback(self,data):
        self.pf_pos[0][1] = data.x
        self.pf_pos[1][1] = data.y
      
    ''' Waypoint 3 Pattern Former Position Callback '''    
    def visionWaypoint3Callback(self,data):
        self.pf_pos[0][2] = data.x
        self.pf_pos[1][2] = data.y
        
    def visionCaptureNumCallback(self,data):
        self.capture_num = data.data
        
    ''' Vision Capture Command Callback '''
    def visionCaptureCallback(self,data):
        try:
            self.rt_image = False
            # Chessboard Calibration Capture
            if self.capture_num == 48:
                time.sleep(0.5)
                result = True
                if self.cam is not None:
                    result &= self.run_camera(self.exposure_time)
                    if self.image is not None:
                        self.image.Save(self.dir_+'ui_calibration.bmp')
                gray_ = self.chessboard_threshold()
                self.chessboard_calibration(gray_)
            # SeamLine Image1 Capture 
            elif self.capture_num == 49:
                time.sleep(1)
                result = True
                if self.cam is not None:
                    result &= self.run_camera(self.exposure_time)
                    if self.image is not None:
                        self.image.Save(self.dir_+'ui_image1.bmp')
                time.sleep(0.1)
                self.vision_image_pub.publish(50)
                self.capture_num = 50
            # SeamLine Image2 Capture     
            elif self.capture_num == 50:
                time.sleep(1)
                result = True
                if self.cam is not None:
                    result &= self.run_camera(self.exposure_time)
                    if self.image is not None:
                        self.image.Save(self.dir_+'ui_image2.bmp')
                time.sleep(0.1)
                self.vision_image_pub.publish(51) 
                self.capture_num = 51
            # SeamLine Image3 Capture    
            elif self.capture_num == 51:
                time.sleep(1)
                result = True
                if self.cam is not None:
                    result &= self.run_camera(self.exposure_time)
                    if self.image is not None:
                        self.image.Save(self.dir_+'ui_image3.bmp')
                time.sleep(0.1)
                self.vision_image_pub.publish(52) 
            self.rt_image = True
        except:
            pass
    
    ''' Image Processing (SeamLine Detection) Command Callback '''
    def visionImageProcessingCallback(self,data):
        self.image_processing()
        
    ''' ROS ShutDown Callback '''
    def rosShutdownCallback(self,data):
        time.sleep(0.5)
        rospy.signal_shutdown("")
        
    ''' Main Vision Algorithm Code '''
    ''' Image Processing (SeamLine Detection) '''
    def image_processing(self):
        try:
            k = 50
            contamination_coeff = 0.2
            p_needle = np.array([[615.85],[-148.11]])
            # p_needle = np.array([[615.85],[-137.75]])
            pix_h = 3000
            pix_w = 4096
            cut_width = 0
            cut_height = 0
            new_image = None
            
            ''' Image Stitch '''
            if self.capture_image_num==3 :
                self.image1 = cv2.imread(self.dir_+'ui_image1.bmp')
                self.image2 = cv2.imread(self.dir_+'ui_image2.bmp')
                self.image3 = cv2.imread(self.dir_+'ui_image3.bmp')
                
                # cut_width = 300
                # cut_height = 100
                
                cut_width = 0
                cut_height = 0
            
                self.image1 = self.image1[cut_height:self.image1.shape[0]-cut_height, cut_width:self.image1.shape[1]-cut_width,:]
                self.image2 = self.image2[cut_height:self.image2.shape[0]-cut_height, cut_width:self.image2.shape[1]-cut_width,:]
                self.image3 = self.image3[cut_height:self.image3.shape[0]-cut_height, cut_width:self.image3.shape[1]-cut_width,:]
                
                
                target_pos = np.dot(self.A,np.array([[cut_width],[pix_h-cut_height]]))+self.B+np.array([[self.pf_pos[0][1]-self.pf_pos[0][0]],[self.pf_pos[1][1]-self.pf_pos[1][0]]])
                target_point = np.dot(inv(self.A),(target_pos - self.B))
                
            elif self.capture_image_num==2 :
                self.image1 = cv2.imread(self.dir_+'ui_image1.bmp')
                self.image2 = cv2.imread(self.dir_+'ui_image2.bmp')
                
                cut_width = 0
                cut_height = 100
                cp_x = 2
                cp_y = -12
                
                self.image1 = self.image1[cut_height:self.image1.shape[0]-cut_height, cut_width:self.image1.shape[1]-cut_width,:]
                self.image2 = self.image2[cut_height:self.image2.shape[0]-cut_height, cut_width:self.image2.shape[1]-cut_width,:]
                
                target_pos = np.dot(self.A,np.array([[cut_width],[pix_h-cut_height]]))+self.B+np.array([[self.pf_pos[0][1]-self.pf_pos[0][0]],[self.pf_pos[1][1]-self.pf_pos[1][0]]])
                target_point = np.dot(inv(self.A),(target_pos - self.B))

                w_p = (int)(round(target_point[0][0])-cut_width)
                h_p = (int)(round(target_point[1][0])-cut_height)+cp_y
                if h_p > pix_h-2*cut_height:
                    
                    target_pos = np.dot(self.A,np.array([[cut_width],[cut_height]]))+self.B+np.array([[self.pf_pos[0][1]-self.pf_pos[0][0]],[self.pf_pos[1][1]-self.pf_pos[1][0]]])
                    target_point = np.dot(inv(self.A),(target_pos - self.B))
                    w_p = (int)(round(target_point[0][0])-cut_width)
                    h_p = (int)(round(target_point[1][0])-cut_height)+cp_y
                    
                    alpha = (int)((self.image1.shape[1]-w_p)/2)
                    new_image = np.zeros([self.image1.shape[0]-h_p,self.image1.shape[1]+w_p+cp_x,self.image1.shape[2]],'uint8')
                    new_image[:,0:w_p+alpha,:] = self.image2[h_p:,0:w_p+alpha,:]
                    new_image[:,w_p+alpha:,:] = self.image1[:self.image1.shape[0]-h_p,alpha-cp_x:,:]
                    
                else:
                    alpha = (int)((self.image1.shape[1]-w_p)/2)
                    new_image = np.zeros([h_p,self.image1.shape[1]+w_p+cp_x,self.image1.shape[2]],'uint8')
                    new_image[:,0:(int)(w_p+alpha),:] = self.image2[0:h_p,0:(int)(w_p+alpha),:]
                    new_image[:,w_p+alpha:,:] = self.image1[self.image1.shape[0]-h_p:,alpha-cp_x:,:]

                
            elif self.capture_image_num==1:
                self.image1 = cv2.imread(self.dir_+'ui_image1.bmp')
                cut_width = 0
                cut_height = 0
                self.image1 = self.image1[cut_height:self.image1.shape[0]-cut_height, cut_width:self.image1.shape[1]-cut_width,:]
                new_image = self.image1
            
            # new_image2 = np.zeros([new_image.shape[0],new_image.shape[1],new_image.shape[2]],'uint8')
            # for i in range(new_image.shape[1]):
            #     new_image2[:,i,:] = new_image[:,new_image.shape[1]-i-1,:]
            # new_image = new_image2
            
            cv2.imwrite(self.dir_+'tot_image.bmp',new_image)
            src = cv2.cvtColor(new_image,cv2.COLOR_BGR2GRAY)
            
            ''' Image Gaussian Blur & 1-Way Filtering '''
            src = cv2.GaussianBlur(src, (5, 5), 5)
            kernel = np.array([[0, k, 0],
                                [0, 0, 0],
                                [0, -k, 0]])
            filtered = cv2.filter2D(src=src, ddepth=-1, kernel=kernel)
            # cv2.imwrite(self.dir_+'filter.bmp',filtered)
            
            ''' Adaptive Thresholding '''
            ret, binary_map = cv2.threshold(filtered,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
            
            # binary_area = (int)(pix_h/2)
            # if binary_area%2 == 0:
            #     binary_area = binary_area + 1
            # binary_map = cv2.adaptiveThreshold(filtered,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY,binary_area,0)
            cv2.imwrite(self.dir_+'binary.bmp',binary_map)
            
            ''' Connected Component Labeling '''
            nlabels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary_map, None,
                                                                                  None, None, 8, cv2.CV_32S)
            areas = stats[1:,cv2.CC_STAT_AREA]
            result = np.zeros((labels.shape), np.uint8)
            areas_list = areas.tolist()
            areas_list.sort(reverse=True)
            for i in range(0, nlabels - 1):
                if areas[i] == areas_list[self.area_num-1]:
                    result[labels == i + 1] = 255   
                    break
            # cv2.imwrite(self.dir_+'area.bmp',result)
            
            result1 = np.zeros((labels.shape), np.uint8)
            for i in range(0, nlabels - 1):
                if areas[i] == areas_list[self.area_num]:
                    result1[labels == i + 1] = 255   
                    break
            skel1 = result1
            arg_y1 = np.where(skel1 > 0)[0]
            arg_x1 = np.where(skel1 > 0)[1]
                
            result2 = np.zeros((labels.shape), np.uint8)
            for i in range(0, nlabels - 1):
                if areas[i] == areas_list[self.area_num+1]:
                    result2[labels == i + 1] = 255   
                    break
            skel2 = result2
            arg_y2 = np.where(skel2 > 0)[0]
            arg_x2 = np.where(skel2 > 0)[1]
            
            
            skel = result  
            arg_y = np.where(skel > 0)[0]
            arg_x = np.where(skel > 0)[1]
            
            D = list()

            for i in range(arg_y.shape[0]) :
                _tmp = [arg_x[i], arg_y[i]]
                D.append(_tmp)
            D = np.array(D)
            
            D1=list()
            for i in range(arg_y1.shape[0]) :
                _tmp = [arg_x1[i], arg_y1[i]]
                D1.append(_tmp)
            D1 = np.array(D1)
            D2=list()
            for i in range(arg_y2.shape[0]) :
                _tmp = [arg_x2[i], arg_y2[i]]
                D2.append(_tmp)
            D2 = np.array(D2)
            # gm = DBSCAN(eps=10, min_samples=5).fit(D)
            # SL = np.where(gm.labels_==1)   
            
            ''' Get SeamLine '''
            #X = D[SL]
            X = D
            X_copy = list()
            tr = np.zeros(X.shape[0])
            for i in range(X.shape[0]) :
                if tr[X[i][0]] == 0:
                    tr[X[i][0]] = X[i][1]
                else:
                    if tr[X[i][0]] < X[i][1] :
                        tr[X[i][0]] = X[i][1]
            for i in range(tr.shape[0]):
                if tr[i] != 0 :
                    X_copy.append([(int(i)),(int(tr[i]))])
            X_copy = np.array(X_copy)
            X = X_copy
            
            ''' Outlier rejection '''
            clf = LocalOutlierFactor(n_neighbors=10, contamination=contamination_coeff)
            y_pred = clf.fit_predict(X)
            PT = list()
            for id, _i in enumerate(y_pred) :
                if _i > 0 :
                    PT.append(X[id])
                    
            ''' Broken line reconstruction '''
            PT.sort(key=lambda x:x[0])
            PT2 = deepcopy(PT)
            prev_x = PT[0][0]
            prev_y = PT[0][1]
            for i in range(1, len(PT)) :
                prev_x = PT[i-1][0]
                prev_y = PT[i-1][1]
                cur_x = PT[i][0]
                cur_y = PT[i][1]
                if cur_x - prev_x > 1 :
                    _tmp = np.arange(prev_x+1, cur_x, 1)
                    _slp = (cur_y - prev_y)/(cur_x - prev_x)
                    x0 = prev_x
                    y0 = prev_y
                    for _x in _tmp :
                        _y = int((_x - x0) * _slp + y0)
                        _res = [_x, _y]
                        PT2.append(_res)  
            PT2.sort(key=lambda x:-x[0])
            size = len(PT2)
            
            ''' Draw SeamLine and Topstitch Line '''
            cnt =0
            PT3 = list()
            left_offset = 0
            right_offset = 0
            for _pt in PT2 :
                if _pt[0] > -1 :
                    cnt += 1
                    if cnt >left_offset and cnt <= size-right_offset:
                        PT3.append(_pt)
                        new_image = cv2.circle(new_image, (_pt[0],_pt[1]), 1,(0, 0, 255), 1)
                        new_image = cv2.circle(new_image, (_pt[0],_pt[1]-24), 1,(255, 0, 0), 1)
            cv2.imwrite(self.dir_+'ui_image_tot.bmp',new_image)
            time.sleep(0.1)
            
            
            
            
            ''' Get SeamLine '''
            #X = D[SL]
            X1 = D1
            X_copy1 = list()
            tr = np.zeros(X1.shape[0])
            for i in range(X1.shape[0]) :
                if tr[X1[i][0]] == 0:
                    tr[X1[i][0]] = X1[i][1]
                else:
                    if tr[X1[i][0]] < X1[i][1] :
                        tr[X1[i][0]] = X1[i][1]
            for i in range(tr.shape[0]):
                if tr[i] != 0 :
                    X_copy1.append([(int(i)),(int(tr[i]))])
            X_copy1 = np.array(X_copy1)
            X1 = X_copy1
            
            ''' Outlier rejection '''
            clf = LocalOutlierFactor(n_neighbors=10, contamination=contamination_coeff)
            y_pred = clf.fit_predict(X1)
            PT11 = list()
            for id, _i in enumerate(y_pred) :
                if _i > 0 :
                    PT11.append(X1[id])
                    
            ''' Broken line reconstruction '''
            PT11.sort(key=lambda x:x[0])
            PT21 = deepcopy(PT11)
            prev_x = PT11[0][0]
            prev_y = PT11[0][1]
            for i in range(1, len(PT11)) :
                prev_x = PT11[i-1][0]
                prev_y = PT11[i-1][1]
                cur_x = PT11[i][0]
                cur_y = PT11[i][1]
                if cur_x - prev_x > 1 :
                    _tmp = np.arange(prev_x+1, cur_x, 1)
                    _slp = (cur_y - prev_y)/(cur_x - prev_x)
                    x0 = prev_x
                    y0 = prev_y
                    for _x in _tmp :
                        _y = int((_x - x0) * _slp + y0)
                        _res = [_x, _y]
                        PT21.append(_res)  
            PT21.sort(key=lambda x:-x[0])
            size = len(PT21)            
            
            ''' Draw SeamLine and Topstitch Line '''
            cnt =0
            PT31 = list()
            left_offset = 0
            right_offset = 0
            for _pt in PT21 :
                if _pt[0] > -1 :
                    cnt += 1
                    if cnt >left_offset and cnt <= size-right_offset:
                        PT31.append(_pt)
                        new_image = cv2.circle(new_image, (_pt[0],_pt[1]), 1,(0, 0, 255), 1)
            cv2.imwrite(self.dir_+'ui_image_tot.bmp',new_image)
            time.sleep(0.1)
            
            
            
            
            
            
            
            ''' Get SeamLine '''
            #X = D[SL]
            X2 = D2
            X_copy2 = list()
            tr = np.zeros(X2.shape[0])
            for i in range(X2.shape[0]) :
                if tr[X2[i][0]] == 0:
                    tr[X2[i][0]] = X2[i][1]
                else:
                    if tr[X2[i][0]] < X2[i][1] :
                        tr[X2[i][0]] = X2[i][1]
            for i in range(tr.shape[0]):
                if tr[i] != 0 :
                    X_copy2.append([(int(i)),(int(tr[i]))])
            X_copy2 = np.array(X_copy2)
            X2 = X_copy2
            
            ''' Outlier rejection '''
            clf = LocalOutlierFactor(n_neighbors=10, contamination=contamination_coeff)
            y_pred = clf.fit_predict(X2)
            PT12 = list()
            for id, _i in enumerate(y_pred) :
                if _i > 0 :
                    PT12.append(X2[id])
                    
            ''' Broken line reconstruction '''
            PT12.sort(key=lambda x:x[0])
            PT22 = deepcopy(PT12)
            prev_x = PT12[0][0]
            prev_y = PT12[0][1]
            for i in range(1, len(PT12)) :
                prev_x = PT12[i-1][0]
                prev_y = PT12[i-1][1]
                cur_x = PT12[i][0]
                cur_y = PT12[i][1]
                if cur_x - prev_x > 1 :
                    _tmp = np.arange(prev_x+1, cur_x, 1)
                    _slp = (cur_y - prev_y)/(cur_x - prev_x)
                    x0 = prev_x
                    y0 = prev_y
                    for _x in _tmp :
                        _y = int((_x - x0) * _slp + y0)
                        _res = [_x, _y]
                        PT22.append(_res)  
            PT22.sort(key=lambda x:-x[0])
            size = len(PT22)            
            
            ''' Draw SeamLine and Topstitch Line '''
            cnt =0
            PT32 = list()
            left_offset = 0
            right_offset = 0
            for _pt in PT22 :
                if _pt[0] > -1 :
                    cnt += 1
                    if cnt >left_offset and cnt <= size-right_offset:
                        PT32.append(_pt)
                        new_image = cv2.circle(new_image, (_pt[0],_pt[1]), 1,(0, 0, 255), 1)
            cv2.imwrite(self.dir_+'ui_image_tot.bmp',new_image)
            time.sleep(0.1)
            
            
            PT4=list()
            hl1_it=0
            hl2_it=0
            for _pt in PT3:
                hl1_n = -1
                hl2_n = -1
                for i in range(hl1_it,len(PT31)):
                    if PT31[i][0]==_pt[0]:
                        hl1_n=PT31[i][1]
                        hl1_it=i+1
                        break
                for i in range(hl2_it,len(PT32)):
                    if PT32[i][0]==_pt[0]:
                        hl2_n=PT32[i][1]
                        hl2_it=i+1
                        break
                PT4.append([_pt[0],_pt[1],hl1_n,hl2_n])
            
            
            
            
            

            
            ''' Save Topstitch Line Coordinate to Text File'''
            min_y_final = 1000
            max_y_final = -1000
            with open("C:/catkin_ws/src/ScanSewing/scan_sewing_ssm/cmdline/SL_Final.txt", "w") as file:
                prev_point = np.zeros([2,1])
                dist = 100
                for _pt in PT4 :
                    if _pt[0] > -1:
                        pt_vec = np.array([[_pt[0]+cut_width],[_pt[1]+cut_height]])
                        # pt_pf = p_needle - (np.dot(self.A,pt_vec)+self.B)
                        pt_pf = np.dot(self.A,pt_vec)+self.B
                        x_pf=y_pf=0
                        if self.capture_image_num==3:
                            x_pf = round(pt_pf[0][0]+self.pf_pos[0][2],2)
                            y_pf = round(pt_pf[1][0]+self.pf_pos[1][2]+self.sl_to_ts,2)
                        elif self.capture_image_num==2:
                            x_pf = round(pt_pf[0][0],2)
                            y_pf = round(pt_pf[1][0]-self.sl_to_ts,2)
                        elif self.capture_image_num==1:
                            x_pf = round(pt_pf[0][0]+self.pf_pos[0][0],2)
                            y_pf = round(pt_pf[1][0]+self.pf_pos[1][0]+self.sl_to_ts,2)
                        pt_pf = np.array([[x_pf],[y_pf]])
                        
                        
                        pt_vec = np.array([[_pt[0]+cut_width],[_pt[2]+cut_height]])
                        pt_pf1 = np.dot(self.A,pt_vec)+self.B
                        x_pf1 = round(pt_pf1[0][0],2)
                        y_pf1 = round(pt_pf1[1][0],2)
                        pt_pf1 = np.array([[x_pf1],[y_pf1]])
                        pt_vec = np.array([[_pt[0]+cut_width],[_pt[3]+cut_height]])
                        pt_pf2 = np.dot(self.A,pt_vec)+self.B
                        x_pf2 = round(pt_pf2[0][0],2)
                        y_pf2 = round(pt_pf2[1][0],2)
                        pt_pf2 = np.array([[x_pf2],[y_pf2]])
                        
                        
                        # pt_pf1=pt_pf
                        # pt_pf2=pt_pf
                        # x_pf1=y_pf1=0
                        # x_pf2=y_pf2=0
                        # for _pt_hl1 in PT31:
                        #     if _pt_hl1[0]== _pt[0]:
                        #         pt_vec = np.array([[_pt_hl1[0]+cut_width],[_pt_hl1[1]+cut_height]])
                        #         # pt_pf = p_needle - (np.dot(self.A,pt_vec)+self.B)
                        #         pt_pf1 = np.dot(self.A,pt_vec)+self.B
                        #         x_pf1 = round(pt_pf1[0][0],2)
                        #         y_pf1 = round(pt_pf1[1][0],2)
                        # pt_pf1 = np.array([[x_pf1],[y_pf1]])
                        # for _pt_hl2 in PT32:
                        #     if _pt_hl2[0]== _pt[0]:
                        #         pt_vec = np.array([[_pt_hl2[0]+cut_width],[_pt_hl2[1]+cut_height]])
                        #         # pt_pf = p_needle - (np.dot(self.A,pt_vec)+self.B)
                        #         pt_pf2 = np.dot(self.A,pt_vec)+self.B
                        #         x_pf2 = round(pt_pf2[0][0],2)
                        #         y_pf2 = round(pt_pf2[1][0],2)
                        # pt_pf2 = np.array([[x_pf2],[y_pf2]])
                        y_pf_max = min((y_pf1,y_pf2))
                        
                        alpha = y_pf_max-(-207)
                        
                        
                        
                        
                        
                        if prev_point[0][0] == 0:
                            prev_point = pt_pf
                            file.write("X")
                            file.write(str(x_pf))
                            # file.write(" Y")
                            # file.write(str(y_pf))
                            # file.write(" HL")
                            # file.write(str(y_pf_max))
                            file.write(" Y")
                            file.write(str(y_pf-alpha))
                            file.write("\n")
                            if y_pf < min_y_final:
                                min_y_final = y_pf
                            if y_pf > max_y_final:
                                max_y_final = y_pf
                        else :
                            new_dist = abs(np.linalg.norm(pt_pf-prev_point)-3)
                            if new_dist > dist and  new_dist < 0.2:
                                l = np.linalg.norm(pt_pf-prev_point)
                                new_pt_pf = prev_point + (pt_pf-prev_point)*(3.0/l)
                                x_pf = round(new_pt_pf[0][0],2)
                                y_pf = round(new_pt_pf[1][0],2)
                                prev_point = new_pt_pf
                                dist = 100
                                file.write("X")
                                file.write(str(x_pf))
                                # file.write(" Y")
                                # file.write(str(y_pf))
                                # file.write(" HL")
                                # file.write(str(y_pf_max))
                                file.write(" Y")
                                file.write(str(y_pf-alpha))
                                file.write("\n")
                                if y_pf < min_y_final:
                                    min_y_final = y_pf
                                if y_pf > max_y_final:
                                    max_y_final = y_pf

                            else :
                                dist = new_dist
                file.close() 
                # self.vision_image_pub.publish(53)
                
                print(max_y_final,min_y_final)
                ''' Check if Detected Line is SeamLine '''
                # if max_y_final - min_y_final > 5 or max_y_final<-207 or min_y_final>-197:
                if max_y_final - min_y_final > 5:
                    self.area_num = self.area_num+1
                    self.vision_area_num_pub.publish(self.area_num)
                    self.image_processing()
                else :
                    self.vision_image_pub.publish(53)
                    self.area_num = 1
                    
                
                
            
                    
        except  Exception:
            self.vision_error_pub.publish(51)
            
    
    ''' ROS Init Function '''
    def init(self):
        rospy.init_node('vision_init', anonymous=True)
		
        self.vision_image_pub = rospy.Publisher('vision_image',Char,queue_size=10)
        self.vision_check_pub = rospy.Publisher('vision_check',Char,queue_size=10)
        self.vision_error_pub = rospy.Publisher('vision_error',Char,queue_size=10)
        self.vision_area_num_pub = rospy.Publisher('vision_area_num',Int32,queue_size=10)
        
        rospy.Subscriber('/scan_sewing_gui/vision_connect',Bool, self.connectCallback)
        rospy.Subscriber('/scan_sewing_gui/vision_exposure_time',Int32,self.visionExposuretimeCallback)
        rospy.Subscriber('/scan_sewing_gui/vision_offset',Float64,self.visionOffsetCallback)
        rospy.Subscriber('/scan_sewing_gui/vision_stitch_length',Float64,self.visionStitchLengthCallback)
        # rospy.Subscriber('/scan_sewing_gui/vision_calibration',Point,self.visionCalibrationCallback)
        # rospy.Subscriber('/scan_sewing_gui/vision_capture',Char,self.visionCaptureCallback)
        rospy.Subscriber('/scan_sewing_gui/vision_capture_num',Char,self.visionCaptureNumCallback)
        rospy.Subscriber('/scan_sewing_gui/vision_image_processing',Empty,self.visionImageProcessingCallback)
        rospy.Subscriber('/scan_sewing_gui/vision_area_num',Int32,self.visionAreaNumCallback)
        rospy.Subscriber('/scan_sewing_gui/vision_capture_image_num',Int32,self.visionCaptureImageNumCallback)
        rospy.Subscriber('/scan_sewing_gui/vision_waypoint1',Point,self.visionWaypoint1Callback)
        rospy.Subscriber('/scan_sewing_gui/vision_waypoint2',Point,self.visionWaypoint2Callback)
        rospy.Subscriber('/scan_sewing_gui/vision_waypoint3',Point,self.visionWaypoint3Callback)
        rospy.Subscriber('/scan_sewing_gui/ros_shutdown',Bool,self.rosShutdownCallback)
        
        rospy.Subscriber('/scan_sewing_pf/pf_loc',Bool,self.visionCaptureCallback)
        # delete
        rospy.Subscriber('/scan_sewing_gui/vision_vc',Bool,self.visionCaptureCallback)
        

    ''' ROS Loop '''
    def run(self):
        rate = rospy.Rate(90)
        while not rospy.is_shutdown():
            try:
                if self.cam is None:
                    self.system = PySpin.System.GetInstance()
                    self.cam_list = self.system.GetCameras()
                    num_cameras = self.cam_list.GetSize()
                    if num_cameras == 0:
                        self.cam = None       
                        self.cam_list.Clear()
                        self.system.ReleaseInstance()
                    else :
                        self.cam = self.cam_list[0]
                        self.vision_check_pub.publish(49)
                        self.connectCallback(None)
                        self.image = None
                else :
                    result = True
                    if self.cam is not None and self.rt_image is True:
                        result &= self.run_camera(self.exposure_time)
                        if self.image is not None:
                            # cv2.imwrite(self.dir_+'ui_rt.bmp',self.image)
                            self.image.Save(self.dir_+'ui_rt.bmp')
                            self.vision_image_pub.publish(48)
                            
            except:
                print("Fail")
                self.rt_image = False
                self.cam = None
            gc.collect()
            rate.sleep()
        self.release_camera()


'''  Main '''
def main():
    vs = Vision()
    try:
        vs.init()
        vs.run()
    except rospy.ROSInterruptException:
        pass


if __name__=='__main__':
	main()


