import cv2, sys, time, copy, rospy, PySpin, random
from copy import deepcopy
import numpy as np
from numpy.linalg import inv
from PIL import Image
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from sklearn.neighbors import LocalOutlierFactor
from sklearn.neighbors import NearestNeighbors
from scipy.optimize import minimize
from scipy.spatial import distance_matrix
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from std_msgs.msg import Char
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point
from scipy.signal import savgol_filter
import skimage
from skimage import filters
from skimage.filters import threshold_sauvola
from scipy.ndimage import uniform_filter1d
from scipy.ndimage import convolve
from mpl_toolkits.axes_grid1.inset_locator import inset_axes, mark_inset
from matplotlib.patches import Rectangle
from matplotlib.lines import Line2D
import gc
import logging
import traceback

logging.basicConfig(filename='C:/catkin_ws/src/ScanSewing/scan_sewing_vision/images/test.log',level=logging.ERROR)

class Vision:
    
    '''Class init function'''
    def __init__(self):
        
        # Class Variable Initialization
        self.dir_ = 'C:/catkin_ws/src/ScanSewing/scan_sewing_vision/images/'
        self.capture_num = 48
        self.exposure_time = 30000 
        self.offset = 0
        self.rt_image = True
        self.released = False
        self.calibration_pf_pos = np.array([-1.0,-1.0])
        self.calibration_temp_pos = np.array([-1.0,-1.0])
        self.path_num = 3
        self.capture_done = False
        self.first_capture = True
        
        self.pf_pos = -np.ones([2,3])
        self.pf_pos[0][0] = 540.0
        self.pf_pos[1][0] = 30.0
        self.pf_pos[0][1] = 760.0
        self.pf_pos[1][1] = 30.0
        
        
        self.A = np.identity(2)
        self.B = np.zeros([2,1])
        try:
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
        
        self.system = PySpin.System.GetInstance()
        self.cam_list = self.system.GetCameras()
        num_cameras = self.cam_list.GetSize()
        
        if num_cameras == 0:
            self.cam = None       
            self.cam_list.Clear()
            self.system.ReleaseInstance()
            print('Camera is not connected!')
            #self.vision_error_pub.publish(50)
        else :
            self.cam = self.cam_list[0]
            self.image = None
        
        

    
    def __del__(self):
        if self.released is False:
            self.released = True
            self.release_camera()
    
            
    ''' Release Camera'''
    def release_camera(self):
        if hasattr(self, 'cam') and self.cam is not None:
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
            
            # GAIN_VALUE = 10
            # self.cam.GainAuto.SetValue(PySpin.GainAuto_Off)
            # self.cam.Gain.SetValue(GAIN_VALUE)
    
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
            # self.cam.GainAuto.SetValue(PySpin.GainAuto_On)
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
    def chessboard_calibration(self,_img,_x=502.0,_y=687.0):
        try:
            _x = self.calibration_pf_pos[0]
            _y = 900 - self.calibration_pf_pos[1]
            
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
                v_t = np.array([[self.calibration_temp_pos[0]],[-self.calibration_temp_pos[1]]])
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
            else:
                self.vision_error_pub.publish(49)
                print("Calibration failed")
        except:
            pass
        
    ''' Callback Functions '''
    
    ''' Exposure Time Callback '''
    def visionExposuretimeCallback(self,data):
        self.exposure_time = data.data
    
    ''' Offset(SeamLine - Topstitch Line) Callback '''
    def visionOffsetCallback(self,data):
        self.offset = data.data
    
    def visionStitchLengthCallback(self,data):
        self.stitch_length = data.data
    
    ''' Pattern Former Position (Calibration) Callback '''
    def visionCalibrationCallback(self,data):
        self.calibration_pf_pos[0] = data.data[0]
        self.calibration_pf_pos[1] = data.data[1]
        self.calibration_temp_pos[0] = data.data[2]
        self.calibration_temp_pos[1] = data.data[3]

    ''' Vision Capture Number Callback '''
    def visionCaptureNumCallback(self,data):
        self.capture_num = data.data
        
        
    ''' Vision Capture Command Callback '''
    def visionCaptureCallback(self,data):
        try:
            if data.data == True: # Always True
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
                        if self.image is not None and self.capture_done == False:
                            self.image.Save(self.dir_+'ui_image2.bmp')
                            self.capture_done = True
                    time.sleep(0.1)
                    self.vision_image_pub.publish(51) 
                # self.rt_image = True
        except:
            pass
    
    ''' Image Processing (SeamLine Detection) Command Callback '''
    def visionImageProcessingCallback(self,data):
        self.image_processing()
        
    def visionRefreshCallback(self,data):
        self.rt_image = True

    def pathNumCallback(self,data):
        self.path_num = data.data
        
    ''' ROS ShutDown Callback '''
    def rosShutdownCallback(self,data):
        time.sleep(0.5)
        rospy.signal_shutdown("")
    
    def preprocessing_img(self,_img,_iter = 1):
        try:
            src = cv2.cvtColor(_img,cv2.COLOR_BGR2GRAY)
            cv2.imwrite(self.dir_+'fig1.bmp',src[1650:1900, 1100:1900])
            clahe = cv2.createCLAHE(clipLimit=4.0, tileGridSize=(32, 32))
            clahe_image = clahe.apply(src)
            cv2.imwrite(self.dir_+'fig2.bmp',clahe_image[1650:1900, 1100:1900])
            #clahe_image = src
            
            ''' Image Gaussian Blur & 1-Way Filtering '''
            src = cv2.GaussianBlur(clahe_image, (5, 5), 5)
            cv2.imwrite(self.dir_+'fig3.bmp',src[1650:1900, 1100:1900])
            # k = 50
            # k2 = 0
            # kernel = np.array([[k2, k, k2],
            #                     [0, 0, 0],
            #                     [-k2, -k, -k2]])
            # filtered1 = cv2.filter2D(src=src, ddepth=-1, kernel=kernel)
            # cv2.imwrite(self.dir_+'thre_sobel.bmp',filtered1)
            filtered = src
            ''' Adaptive Thresholding '''
            ret, binary_map = cv2.threshold(filtered,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
            cv2.imwrite(self.dir_+'thre_otsu.bmp',binary_map)
            window_size = 25
            threshold_sauvola_result = threshold_sauvola(filtered, window_size=window_size)

            binary_map = filtered > threshold_sauvola_result
            binary_map = 255 - binary_map.astype(np.uint8) * 255
            cv2.imwrite(self.dir_+'thre_sau.bmp',binary_map)
            cv2.imwrite(self.dir_+'fig4.bmp',binary_map[1650:1900, 1100:1900])
            
            k = cv2.getStructuringElement(cv2.MORPH_RECT, (7,7))
            closing = cv2.morphologyEx(binary_map, cv2.MORPH_CLOSE, k,iterations=_iter)
            cv2.imwrite(self.dir_+'thre_sau_post.bmp',closing)
            cv2.imwrite(self.dir_+'fig5.bmp',closing[1650:1900, 1100:1900])
            
            if self.path_num == 3:
                binary_img = closing

                window_size = 61  
                smoothed_img = uniform_filter1d(binary_img, size=window_size, axis=1, mode='constant')
                
                threshold = 2 * 255 / 3
                new_binary_img = np.where(smoothed_img >= threshold, 1, 0)
                
                output_img = (new_binary_img * 255).astype(np.uint8)
                if self.first_capture == True:
                    top_left = (1300, 2800)  
                    bottom_right = (1700, 3200) 
                    self.first_capture = False
                else :
                    top_left = (1300, 700)  
                    bottom_right = (1700, 1100) 
                
                    # border_thickness=5
                    # output_img[top_left[0]:top_left[0] + border_thickness, top_left[1]:bottom_right[1]] = 255
    
                    # output_img[bottom_right[0] - border_thickness:bottom_right[0], top_left[1]:bottom_right[1]] = 255
    
                    # output_img[top_left[0]:bottom_right[0], top_left[1]:top_left[1] + border_thickness] = 255
    
                    # output_img[top_left[0]:bottom_right[0], bottom_right[1] - border_thickness:bottom_right[1]] = 255
                
                output_img[top_left[0]:bottom_right[0], top_left[1]:bottom_right[1]] = binary_img[top_left[0]:bottom_right[0], top_left[1]:bottom_right[1]]

                cv2.imwrite(self.dir_+'thre_sau_post_post.bmp',output_img)
                closing = output_img

            return closing
        except:
            logging.error(traceback.format_exc())
        
    def line_detecting(self,_img,_area_num, _length):
        try:
            contamination_coeff = 0.2
            
            binary_map = _img

            ''' Connected Component Labeling '''
            nlabels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary_map, None,
                                                                                  None, None, 8, cv2.CV_32S)
            
            # if _area_num==0:
            #     if nlabels == 9293:
            #         binary_map_copy = binary_map.copy()
            #         for i in range(1, nlabels):
            #             mask = (labels == i)
            #             binary_map_copy[mask] = 128
            #         cv2.imwrite(self.dir_ + 'fig6.bmp', binary_map_copy[1650:1900, 1100:1900])
            
            
            
            
            
            
            areas = stats[1:, cv2.CC_STAT_WIDTH]
            #areas = stats[1:,cv2.CC_STAT_AREA]
            result = np.zeros((labels.shape), np.uint8)
            areas_list = areas.tolist()
            areas_list.sort(reverse=True)
            
            
            
            
            
            # if _area_num==1:
            #     if nlabels == 9293:
            #         binary_map_rgb = cv2.cvtColor(binary_map, cv2.COLOR_GRAY2BGR)
                    
            #         green_color = (0, 255, 0) 
            #         red_color = (0, 0, 255) 
                    
            #         _area_num = 0
            #         result = np.zeros(labels.shape, np.uint8)
            #         if areas_list[_area_num] >= _length and _area_num < len(areas_list):
            #             for i in range(0, nlabels - 1):
            #                 if areas[i] == areas_list[_area_num]:
            #                     result[labels == i + 1] = 255
            #                     break
            #         binary_map_rgb[result == 255] = green_color
                    
            #         _area_num = 1
            #         result = np.zeros(labels.shape, np.uint8)
            #         if areas_list[_area_num] >= _length and _area_num < len(areas_list):
            #             for i in range(0, nlabels - 1):
            #                 if areas[i] == areas_list[_area_num]:
            #                     result[labels == i + 1] = 255
            #                     break
            #         binary_map_rgb[result == 255] = red_color
                    
            #         cv2.imwrite(self.dir_ + 'fig7.bmp', binary_map_rgb[1650:1900, 1100:1900])
            
            
            
            
            
            
            if areas_list[_area_num] < _length:
                return list()
            if _area_num >= len(areas_list):
                return list()
            for i in range(0, nlabels - 1):
                if areas[i] == areas_list[_area_num]:
                    result[labels == i + 1] = 255  
                    break
            if _area_num==1:
                cv2.imwrite(self.dir_+'area.bmp',result)
            skel = result
            arg_y = np.where(skel > 0)[0]
            arg_x = np.where(skel > 0)[1]
            
            D = list()
            
            for i in range(arg_y.shape[0]) :
                _tmp = [arg_x[i], arg_y[i]]
                D.append(_tmp)
            D = np.array(D)
            
            ''' Get SeamLine '''
            #X = D[SL]
            X = D
            X_copy = list()
            tr = np.zeros(1)
            for i in range(X.shape[0]) :
                if X[i][0] >= tr.shape[0]:
                    new_size = X[i][0] + 1
                    tr = np.pad(tr, (0, new_size - tr.shape[0]), mode='constant', constant_values=0)

                
                if tr[X[i][0]] == 0:
                    tr[X[i][0]] = X[i][1]
                else:
                    if tr[X[i][0]] > X[i][1] :
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
            PT2.sort(key=lambda x:-x[0], reverse=True)
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
            # print(_area_num,len(PT3))
            return PT3
           
        except:
            logging.error(traceback.format_exc())
    
        
    def line_filtering(self,_PT,_po=2):
        line = None
        _wl = 101
        
        if len(_PT)>0:
            if len(_PT)>_wl:
                line = np.array(_PT)

                filtered_line = savgol_filter(line[:,1], _wl, polyorder=_po)
                line[:,1] = filtered_line
            else:
                line = np.array(_PT)
        return line
    
    def border_detecting(self,_img,_pt,_dir=True,thr_size=15,crop_size=150):
        crop_image = cv2.cvtColor(_img, cv2.COLOR_BGR2GRAY)
        crop_image = crop_image[int(_pt[1]-crop_size):int(_pt[1]+crop_size),int(_pt[0]-crop_size):int(_pt[0]+crop_size)]
        cv2.imwrite(self.dir_+'crop.bmp',crop_image)
        avg_list = list()
        for i in range(crop_image.shape[1]):
            sum_ = 0
            for j in range(crop_image.shape[0]):
                sum_ += crop_image[j][i]
            avg_list.append(sum_/crop_image.shape[0])
        thr = min(avg_list)+thr_size
        index = 0
        if _dir:
            for i, val in enumerate(avg_list):
                if val < thr:
                    index = i
        else:
            for i in range(len(avg_list)-1,-1,-1):
                if avg_list[i] < thr:
                    index = i
        return index+_pt[0]-crop_size
    
    
    def calculate_overlap(self,shifted_points1, points2):
        """Calculate the overlap score between shifted_points1 and points2."""
        # Calculate distance matrix between shifted_points1 and points2
        dist_matrix = distance_matrix(shifted_points1, points2)
        # The overlap score is the sum of the minimum distances for each point in shifted_points1
        overlap_score = np.sum(np.min(dist_matrix, axis=1))
        return overlap_score
    
    def find_best_translation(self,points1, points2, steps=100, shift_range=(-10, 10)):
        """Find the best translation for points1 to overlap with points2."""
        best_shift = None
        best_overlap = -np.inf
        step_size = (shift_range[1] - shift_range[0]) / float(steps)
        # print((shift_range[1] - shift_range[0]),steps,step_size)
        
        for dx in np.arange(shift_range[0], shift_range[1], step_size):
            for dy in np.arange(shift_range[0], shift_range[1], step_size):
                shifted_points1 = points1 + np.array([dx, dy])
                overlap = self.calculate_overlap(shifted_points1, points2)
                if overlap > best_overlap:
                    best_overlap = overlap
                    best_shift = (dx, dy)
        return best_shift
        
    ''' Main Vision Algorithm Code '''
    ''' Image Processing (SeamLine Detection) '''
    def image_processing(self):
        try:
            new_image = None
            
            #self.path_num = 2 # DELETE THIS
            self.image1 = cv2.imread(self.dir_+'ui_image1.bmp')
            self.image2 = cv2.imread(self.dir_+'ui_image2.bmp')
            #self.image1 = cv2.imread(self.dir_+'ruler_cal1.bmp')
            #self.image2 = cv2.imread(self.dir_+'ruler_cal2.bmp')
           
            cut_width = 0
            cut_height = 50
            cp_x = 60
           
            self.image1 = self.image1[cut_height:self.image1.shape[0]-cut_height, cut_width:self.image1.shape[1]-cut_width,:]
            self.image2 = self.image2[cut_height:self.image2.shape[0]-cut_height, cut_width:self.image2.shape[1]-cut_width,:]
            
            target_pos = np.dot(self.A,np.array([[cut_width],[cut_height]]))+self.B+np.array([[self.pf_pos[0][1]-self.pf_pos[0][0]],[self.pf_pos[1][1]-self.pf_pos[1][0]]])
            target_point = np.dot(inv(self.A),(target_pos - self.B))
            w_p = (int)(round(target_point[0][0])-cut_width)
            alpha = (int)((self.image1.shape[1]-w_p)/2)
            
            area_num1 = 0
            trig = [False, False, False]
            line1 = line12 = line13 = None
            
            boundary_limit1 = 0
            boundary_limit2 = 3000
            if self.path_num == 0:
                boundary_limit1 = 1000
                boundary_limit2 = 1700
            elif self.path_num == 1:
                boundary_limit1 = 2100
                boundary_limit2 = 2700
            elif self.path_num == 2:
                boundary_limit1 = 1500
                boundary_limit2 = 2300
            elif self.path_num == 3:
                # boundary_limit1 = 1250
                # boundary_limit2 = 1550
                boundary_limit1 = 650
                boundary_limit2 = 1050
                
            binary_img1 = self.preprocessing_img(self.image1)
            while True:
                PT = self.line_detecting(binary_img1,area_num1, self.image1.shape[1]/3)
                # print("First Point Check")
                # print(PT[0])
                # print(PT[-1])
                if len(PT)> self.image1.shape[1]/3 and PT[0][1]>boundary_limit1 and PT[0][1]<boundary_limit2:
                    if trig[0] == False:
                        # border_index = self.border_detecting(self.image1,PT[-1])
                        # PT = [elem for elem in PT if elem[0] <= border_index]
                        line1 = self.line_filtering(PT)
                        trig[0] = True
                    elif trig[1] == False:
                        PT = [[x, y] for x, y in PT if x <= line1[-1][0]]
                        if len(PT)> self.image1.shape[1]/3:
                            line12 = self.line_filtering(PT)
                            trig[1] = True
                    elif trig[2] == False:
                        PT = [[x, y] for x, y in PT if x <= line12[-1][0]]
                        if len(PT)> self.image1.shape[1]/3:
                            line13 = self.line_filtering(PT)
                            trig[2] = True
                            break
                area_num1 += 1
                if area_num1 == 10:
                    break
                
            if trig[2] == False:
                if line1[0][1] > line12[0][1]:
                    line13 = line1
                    line1 = line12
                else:
                    line13 = line12
            else:
                if line1[0][1] > line12[0][1]:
                    temp = line1
                    line1 = line12
                    line12 = temp
                if line1[0][1] > line13[0][1]:
                    temp = line1
                    line1 = line13
                    line13 = temp
                if line12[0][1] < line13[0][1]:
                    temp = line12
                    line12 = line13
                    line13 = temp
            
            # if trig[0]==True and trig[1]==True:
            #     print_image = self.image1.copy()
            #     for i in range(len(line1)):
            #         print_image = cv2.circle(print_image, (line1[i][0],line1[i][1]), 1,(0, 0, 255), 1)
            #     for i in range(len(line12)):
            #         print_image = cv2.circle(print_image, (line12[i][0],line12[i][1]), 1,(0, 255, 0), 1)
            #     if trig[2]==True:
            #         for i in range(len(line13)):
            #             print_image = cv2.circle(print_image, (line13[i][0],line13[i][1]), 1,(255, 0, 0), 1)
            #     cv2.imwrite(self.dir_+'1111_3.bmp',print_image)
            #     pass
            # else:
            #     print('image1 line detection failed')
            #     raise

            print_image1 = self.image1.copy()
            for i in range(len(line1)):
                print_image1 = cv2.circle(print_image1, (line1[i][0],line1[i][1]), 1,(0, 255, 0), 1)
            if trig[2]==True:
                for i in range(len(line12)):
                    print_image1 = cv2.circle(print_image1, (line12[i][0],line12[i][1]), 1,(0, 255, 255), 1)     
            for i in range(len(line13)):
                print_image1 = cv2.circle(print_image1, (line13[i][0],line13[i][1]), 1,(0, 0, 255), 1)     
            
            cv2.imwrite(self.dir_+'image1_det1.bmp',print_image1)
            
            # return
            boundary_limit1 = 0
            boundary_limit2 = 3000
            if self.path_num == 0:
                boundary_limit1 = 1000
                boundary_limit2 = 1700
            elif self.path_num == 1:
                boundary_limit1 = 2300
                boundary_limit2 = 3000
            elif self.path_num == 2:
                boundary_limit1 = 2000
                boundary_limit2 = 2700
            elif self.path_num == 3: # 1300,1600
                boundary_limit1 = 700
                boundary_limit2 = 1100   
            
            area_num2 = 0
            trig = [False, False, False]
            line2 = line22 = line23 = None
            binary_img2 = self.preprocessing_img(self.image2)
            while True:
                PT = self.line_detecting(binary_img2,area_num2,self.image2.shape[1]/3)
                # print("Second Point Check")
                # print(PT[0])
                # print(PT[-1])
                if len(PT)> self.image2.shape[1]/3 and PT[-1][1]>boundary_limit1 and PT[-1][1]<boundary_limit2:
                    if trig[0] == False:
                        # border_index = self.border_detecting(self.image2,PT[0],False)
                        # PT = [elem for elem in PT if elem[0] >= border_index]
                        line2 = self.line_filtering(PT)
                        trig[0] = True
                    elif trig[1] == False:
                        PT = [[x, y] for x, y in PT if x >= line2[0][0]]
                        if len(PT)> self.image2.shape[1]/3:
                            line22 = self.line_filtering(PT)
                            trig[1] = True
                    elif trig[2] == False:
                        PT = [[x, y] for x, y in PT if x >= line2[0][0]]
                        if len(PT)> self.image2.shape[1]/3:
                            line23 = self.line_filtering(PT)
                            trig[2] = True
                            break
                area_num2 += 1
                if area_num2 == 10:
                    break
                
            if trig[2] == False:
                if line2[-1][1] > line22[-1][1]:
                    line23 = line2
                    line2 = line22
                else:
                    line23 = line22
            else:
                if line2[-1][1] > line22[-1][1]:
                    temp = line2
                    line2 = line22
                    line22 = temp
                if line2[-1][1] > line23[-1][1]:
                    temp = line2
                    line2 = line23
                    line23 = temp
                if line22[-1][1] < line23[-1][1]:
                    temp = line22
                    line22 = line23
                    line23 = temp
                
            # if trig[0]==True and trig[1]==True:
            #     print_image = self.image2.copy()
            #     for i in range(len(line2)):
            #         print_image = cv2.circle(print_image, (line2[i][0],line2[i][1]), 1,(0, 0, 255), 1)
            #     for i in range(len(line22)):
            #         print_image = cv2.circle(print_image, (line22[i][0],line22[i][1]), 1,(0, 255, 0), 1)
            #     if trig[2]==True:
            #         for i in range(len(line23)):
            #             print_image = cv2.circle(print_image, (line23[i][0],line23[i][1]), 1,(255, 0, 0), 1)
            #     cv2.imwrite(self.dir_+'2222_3.bmp',print_image)
            #     pass
            # else:
            #     print('image2 line detection failed')
            #     raise
            

            print_image2 = self.image2.copy()
            for i in range(len(line2)):
                print_image2 = cv2.circle(print_image2, (line2[i][0],line2[i][1]), 1,(0, 255, 0), 1)
            if trig[2]==True:
                for i in range(len(line22)):
                    print_image2 = cv2.circle(print_image2, (line22[i][0],line22[i][1]), 1,(0, 255, 255), 1)  
            for i in range(len(line23)):
                print_image2 = cv2.circle(print_image2, (line23[i][0],line23[i][1]), 1,(0, 0, 255), 1)  

            cv2.imwrite(self.dir_+'image2_det.bmp',print_image2)
            cv2.imwrite(self.dir_+'fig8.bmp',print_image2[1650:1900, 1100:1900])

            # np.save(self.dir_+'line1.npy',line1)  
            # np.save(self.dir_+'line12.npy',line12)  
            # np.save(self.dir_+'line13.npy',line13)  
            # np.save(self.dir_+'line2.npy',line2)  
            # np.save(self.dir_+'line22.npy',line22)  
            # np.save(self.dir_+'line23.npy',line23)  
            
            # line1 = np.load(self.dir_+'line1.npy')
            # line12 = np.load(self.dir_+'line12.npy')
            # line13 = np.load(self.dir_+'line13.npy')
            # line2 = np.load(self.dir_+'line2.npy')
            # line22 = np.load(self.dir_+'line22.npy')
            # line23 = np.load(self.dir_+'line23.npy')
            
            
            def transform_points(points, params):
                tx, ty, theta = params
                R = np.array([[np.cos(theta), -np.sin(theta)],
                              [np.sin(theta), np.cos(theta)]])
                transformed = np.dot(points, R.T) + np.array([tx, ty])
                return transformed

            def find_nearest_neighbors(C1, C2):
                nbrs = NearestNeighbors(n_neighbors=1, algorithm='auto').fit(C1)
                distances, indices = nbrs.kneighbors(C2)
                return C1[indices.flatten()]
            
            def cost_function(params, C1, C2):
                C2_transformed = transform_points(C2, params)
                C1_nearest = find_nearest_neighbors(C1, C2_transformed)
                return np.sum(np.linalg.norm(C1_nearest - C2_transformed, axis=1)) 
            



            cp_x = 19 # +:spread,-:squeeze
            index2 = np.where(line2[:, 0] == w_p+alpha)[0][0]
            index1 = np.where(line1[:, 0] == alpha-cp_x)[0][0]
            h_index2 = line2[index2][1]
            h_index1 = line1[index1][1]
            # print(h_index1,h_index2)
            l1_div = 0
            l2_div = 0
            if h_index2 > h_index1:
                h_p = h_index2-h_index1
                new_image = np.zeros([self.image1.shape[0]-h_p,self.image1.shape[1]+w_p+cp_x,self.image1.shape[2]],'uint8')
                new_image[:,0:w_p+alpha,:] = self.image2[h_p:,0:w_p+alpha,:]
                new_image[:,w_p+alpha:,:] = self.image1[:self.image1.shape[0]-h_p,alpha-cp_x:,:]
                l2_div = -(h_index2-h_index1)
            else:
                h_p = self.image1.shape[0] - (h_index1-h_index2)
                new_image = np.zeros([h_p,self.image1.shape[1]+w_p+cp_x,self.image1.shape[2]],'uint8')
                new_image[:,0:(int)(w_p+alpha),:] = self.image2[0:h_p,0:(int)(w_p+alpha),:]
                new_image[:,w_p+alpha:,:] = self.image1[self.image1.shape[0]-h_p:,alpha-cp_x:,:]
                l1_div = -(h_index1-h_index2)
            cv2.imwrite(self.dir_+'ui_image_tot_orig.bmp',new_image)
            
            
            final_PT =list()
            index2 = np.where(line23[:, 0] == w_p+alpha)[0][0]
            index1 = np.where(line13[:, 0] == alpha-cp_x)[0][0]
            for i in range(len(line23)):
                if i <= index2:
                    new_line = line23[i] +[0,l2_div]
                    final_PT.append(new_line)
           
            for i in range(len(line13)):
                if i >= index1:
                    new_line = line13[i] +[cp_x+w_p+1,l1_div]
                    final_PT.append(new_line)
            
            
            # final_PT =list()
            # for i in range(len(line2)):
            #     if i <= index2:
            #         new_line = line2[i] +[0,l2_div]
            #         final_PT.append(new_line)
           
            # for i in range(len(line1)):
            #     if i >= index1:
            #         new_line = line1[i] +[cp_x+w_p+1,l1_div]
            #         final_PT.append(new_line)
            
            if self.path_num==3:
                seam_line = self.line_filtering(final_PT,1)
            else:
                seam_line = self.line_filtering(final_PT)
            
            
            # l2_div = -48
            #l2_div = -58
           
            
            final_PT =list()
            for i in range(len(line2)):
                if line2[i][0] <= line23[index2][0]:
                    new_line = line2[i] +[0,l2_div]
                    # final_PT.append(line22[i])
                    final_PT.append(new_line)
           
            for i in range(len(line1)):
                if line1[i][0] >= line13[index1][0]:
                    new_line = line1[i] +[cp_x+w_p+1,l1_div]
                    final_PT.append(new_line)
           
            if self.path_num == 0 or self.path_num == 3:
                template_line = self.line_filtering(final_PT)
            else:
                template_line = self.line_filtering(final_PT)
           
            PT4 = list()
            i = j = 0
            while i < len(seam_line) and j < len(template_line):
                if seam_line[i][0] < template_line[j][0]:
                    i += 1
                elif seam_line[i][0] > template_line[j][0]:
                    j += 1
                else:
                    PT4.append([seam_line[i][0], seam_line[i][1], template_line[j][1]])
                    i += 1
                    j += 1
                    
            
                    
            for i in range(len(seam_line)):
                new_image = cv2.circle(new_image, (seam_line[i][0],seam_line[i][1]), 1,(0, 0, 255), 1)
            for i in range(len(template_line)):
                new_image = cv2.circle(new_image, (template_line[i][0],template_line[i][1]), 1,(0, 255, 0), 1)  
           
            cv2.imwrite(self.dir_+'ui_image_tot.bmp',new_image)
            
    
            PT4.reverse()
            
            
            optimal_params = None
            if self.path_num==0 or self.path_num==3:
                points1 = []
                points2 = []
                points3 = []
                
                center1 = np.array([760 - 175, -255 + 5 + 30])
                radius1 = 5
                theta_start1 = np.deg2rad(0)
                theta_end1 = np.deg2rad(90)
                theta1 = np.linspace(theta_start1, theta_end1, 10)
                
                xx1 = center1[0] - radius1 * np.cos(theta1)
                yy1 = center1[1] - radius1 * np.sin(theta1)
                
                center2 = np.array([760 + 175, -255 + 5 + 30])
                radius2 = 5
                theta_start2 = np.deg2rad(0)
                theta_end2 = np.deg2rad(90)
                theta2 = np.linspace(theta_start2, theta_end2, 10)
                
                xx2 = center2[0] + radius2 * np.sin(theta2)
                yy2 = center2[1] - radius2 * np.cos(theta2)
                
                
                
                line_start = np.array([760 - 175, -255 + 30])
                line_end  = np.array([760 + 175, -255 + 30])
                line_x = np.linspace(line_start[0], line_end[0], 200)
                line_y = np.linspace(line_start[1], line_end[1], 200)
                
                points2 = np.column_stack((np.concatenate((xx1, line_x[1:], xx2[1:])), np.concatenate((yy1, line_y[1:], yy2[1:]))))

                xx_2 = np.linspace(line_start[0], line_end[0], 200)
                yy_2 = np.linspace(line_start[1]+10, line_end[1]+10, 200)
                for _pt in template_line:
                    pt_vec = np.array([[_pt[0]+cut_width],[_pt[1]+cut_height]])
                    pt_pf1 = np.dot(self.A,pt_vec)+self.B
                    x_pf1 = round(pt_pf1[0][0],2)
                    y_pf1 = round(pt_pf1[1][0],2)
                    pt_pf1 = np.array([[x_pf1],[y_pf1]])
                    points1.append(pt_pf1.flatten())
                for _pt in PT4:
                    # pt_vec = np.array([[_pt[0]+cut_width],[_pt[2]+cut_height]])
                    # pt_pf1 = np.dot(self.A,pt_vec)+self.B
                    # x_pf1 = round(pt_pf1[0][0],2)
                    # y_pf1 = round(pt_pf1[1][0],2)
                    # pt_pf1 = np.array([[x_pf1],[y_pf1]])
                    # points1.append(pt_pf1.flatten())
                    pt_vec = np.array([[_pt[0]+cut_width],[_pt[1]+cut_height]])
                    pt_pf1 = np.dot(self.A,pt_vec)+self.B
                    x_pf1 = round(pt_pf1[0][0],2)
                    y_pf1 = round(pt_pf1[1][0],2)
                    pt_pf1 = np.array([[x_pf1],[y_pf1]])
                    points3.append(pt_pf1.flatten())
                    
                points1 = np.array(points1)
                points3 = np.array(points3)
                
                initial_params = [0, 0, 0]
                result = minimize(cost_function, initial_params, args=(points2, points1))
                print("asdffasfdasdf")
                print(result.fun)
                optimal_params = result.x
                
                points1_t = transform_points(points1, optimal_params)
                points3_t = transform_points(points3, optimal_params)
                
                points3_nn_parel = np.zeros_like(points3_t) 
                
                width_of_line = 1.0
                if self.path_num==3:
                    width_of_line = 0.0
                    for i in range(points3_t.shape[0]):
                        #points3_nn_parel[i] = points3_t[i] + [0, correction_value]
                        points3_nn_parel[i] = points3_t[i] + [0,width_of_line]
                else:
                    for i in range(points3_t.shape[0]):
                        points3_nn_parel[i] = points3_t[i] + [0,width_of_line]
                
            elif self.path_num==1:
                center = np.array([760, -589.856])
                radius = 445
                
                points1 = []
                points2 = []
                points3 = []
                theta_start = np.deg2rad(-30)
                theta_end = np.deg2rad(30)
                theta = np.linspace(theta_start,theta_end,200)
                xx = center[0] + radius * np.sin(theta)
                yy = center[1] + radius * np.cos(theta)
                xx_2 = center[0] + (radius+10) * np.sin(theta)
                yy_2 = center[1] + (radius+10) * np.cos(theta)
                points2 = np.column_stack((xx, yy))
                for _pt in PT4:
                    pt_vec = np.array([[_pt[0]+cut_width],[_pt[2]+cut_height]])
                    pt_pf1 = np.dot(self.A,pt_vec)+self.B
                    x_pf1 = round(pt_pf1[0][0],2)
                    y_pf1 = round(pt_pf1[1][0],2)
                    pt_pf1 = np.array([[x_pf1],[y_pf1]])
                    points1.append(pt_pf1.flatten())
                    pt_vec = np.array([[_pt[0]+cut_width],[_pt[1]+cut_height]])
                    pt_pf1 = np.dot(self.A,pt_vec)+self.B
                    x_pf1 = round(pt_pf1[0][0],2)
                    y_pf1 = round(pt_pf1[1][0],2)
                    pt_pf1 = np.array([[x_pf1],[y_pf1]])
                    points3.append(pt_pf1.flatten())
                
                points1 = np.array(points1)
                points3 = np.array(points3)
                
                initial_params = [0, 0, 0]
                result = minimize(cost_function, initial_params, args=(points2, points1))
                optimal_params = result.x
                
                points1_t = transform_points(points1, optimal_params)
                points3_t = transform_points(points3, optimal_params)
                
                points3_nn_parel = np.zeros_like(points3_t) 
                width_of_line = 0.5
                half_point = points3_t.shape[0] // 2
                width_of_line_first_half = np.linspace(0, 1.0, half_point)
                width_of_line_second_half = np.linspace(1.0, 1.0, points3_t.shape[0] - half_point)
                width_of_line = np.concatenate((width_of_line_first_half,width_of_line_second_half))
                # width_of_line = np.linspace(0, 1.8, points3_t.shape[0])
                for i in range(points3_t.shape[0]):
                    vector = points3_t[i] - center
                    distance = np.linalg.norm(vector)
                    if distance != 0:  
                        unit_vector = vector / distance
                        points3_nn_parel[i] = points3_t[i] + unit_vector * width_of_line[i]
            elif self.path_num==2:
                points1 = []
                points2 = []
                points3 = []
                
                center1 = np.array([760 - 112.5, -5.144 + 30])
                radius1 = 230
                theta_start1 = np.deg2rad(-30)
                theta_end1 = np.deg2rad(30)
                theta1 = np.linspace(theta_start1, theta_end1, 100)
                
                xx1 = center1[0] + radius1 * np.sin(theta1)
                yy1 = center1[1] - radius1 * np.cos(theta1)
                radius1 = 220
                xx1_2 = center1[0] + radius1 * np.sin(theta1)
                yy1_2 = center1[1] - radius1 * np.cos(theta1)
                
                center2 = np.array([760 + 112.5, -394.856 + 30])
                radius2 = 220
                theta_start2 = np.deg2rad(-30)
                theta_end2 = np.deg2rad(30)
                theta2 = np.linspace(theta_start2, theta_end2, 100)
                
                xx2 = center2[0] + radius2 * np.sin(theta2)
                yy2 = center2[1] + radius2 * np.cos(theta2)
                radius2 = 230
                xx2_2 = center2[0] + radius2 * np.sin(theta2)
                yy2_2 = center2[1] + radius2 * np.cos(theta2)
                
                points2 = np.column_stack((np.concatenate((xx1, xx2[1:])), np.concatenate((yy1, yy2[1:]))))
                xx_2 = np.concatenate((xx1_2, xx2_2[1:]))
                yy_2 = np.concatenate((yy1_2, yy2_2[1:]))
                for _pt in PT4:
                    pt_vec = np.array([[_pt[0]+cut_width],[_pt[2]+cut_height]])
                    pt_pf1 = np.dot(self.A,pt_vec)+self.B
                    x_pf1 = round(pt_pf1[0][0],2)
                    y_pf1 = round(pt_pf1[1][0],2)
                    pt_pf1 = np.array([[x_pf1],[y_pf1]])
                    points1.append(pt_pf1.flatten())
                    pt_vec = np.array([[_pt[0]+cut_width],[_pt[1]+cut_height]])
                    pt_pf1 = np.dot(self.A,pt_vec)+self.B
                    x_pf1 = round(pt_pf1[0][0],2)
                    y_pf1 = round(pt_pf1[1][0],2)
                    pt_pf1 = np.array([[x_pf1],[y_pf1]])
                    points3.append(pt_pf1.flatten())
                
                points1 = np.array(points1)
                points3 = np.array(points3)
                
                initial_params = [0, 0, 0]
                result = minimize(cost_function, initial_params, args=(points2, points1))
                optimal_params = result.x
                
                points1_t = transform_points(points1, optimal_params)
                points3_t = transform_points(points3, optimal_params)
                
                points3_nn_parel = np.zeros_like(points3_t) 
                
                width_of_line = 0.5
                half_point = points3_t.shape[0] // 2
                qp = points3_t.shape[0] // 16
                width_of_line_first_half = np.linspace(0.7, 1.2, half_point)
                width_of_line_fs_half = np.linspace(1.2, 0.5, qp)
                width_of_line_second_half = np.linspace(0.5, -0.5, points3_t.shape[0] - qp - half_point)
                width_of_line = np.concatenate((width_of_line_first_half, width_of_line_fs_half,width_of_line_second_half))
                for i in range(points3_t.shape[0]):
                    if points3_t[i, 0] < 760:
                        vector = points3_t[i] - center1 
                        distance = np.linalg.norm(vector)
                        
                        if distance != 0:
                            unit_vector = vector / distance
                            points3_nn_parel[i] = points3_t[i] - unit_vector * width_of_line[i] 
                    else:
                        vector = points3_t[i] - center2 
                        distance = np.linalg.norm(vector)
                        
                        if distance != 0:
                            unit_vector = vector / distance
                            points3_nn_parel[i] = points3_t[i] + unit_vector * width_of_line[i] 
            
            with open("C:/catkin_ws/src/ScanSewing/scan_sewing_ssm/cmdline/SL_Final.txt", "w") as file:
                x_values = []
                y_values = []
                pattern_former_path = []
                if self.path_num==0 or self.path_num==3:
                    file_path = 'C:/catkin_ws/src/ScanSewing/scan_sewing_mini/preset_path_00.txt'
                elif self.path_num==1:
                    file_path = 'C:/catkin_ws/src/ScanSewing/scan_sewing_mini/preset_path_01.txt'
                elif self.path_num==2:
                    file_path = 'C:/catkin_ws/src/ScanSewing/scan_sewing_mini/preset_path_02.txt'

                with open(file_path, 'r') as file2:
                    for line in file2:
                        line = line.strip()
                        x_value, y_value = map(float, line.split())
                        x_value = -x_value + 760
                        y_value = y_value + 30
                        pfp = np.array([[x_value],[y_value]])
                        pattern_former_path.append(pfp.flatten())
                pattern_former_path = np.array(pattern_former_path)

                def downsize_to_nearest(D1, D2):
                    nbrs = NearestNeighbors(n_neighbors=1, algorithm='auto', metric='euclidean').fit(D2) 
                    distances, indices = nbrs.kneighbors(D1) 
                    D2_prime = D2[indices.flatten()] 
                    return D2_prime    
                points3_nn = downsize_to_nearest(pattern_former_path, points3_nn_parel)
                cnt = 1
                for _pt in points3_nn:
                    file.write("X")
                    file.write(str(round(_pt[0],2)))
                    file.write(" Y")
                    if self.path_num==3:
                        # correction_value = 1.3726*(10**-7) * (cnt**4) - 3.0469 * (10**-5) * (cnt**3) + 0.0021 * (cnt**2) - 0.0347 * cnt + 0.3423
                        correction_value = -0.3229
                    else:
                        correction_value = 0
                    file.write(str(round(_pt[1]-self.offset+correction_value,2)))
                    file.write("\n")
                    cnt = cnt +1
                # plt.plot(points1[:,0],points1[:,1], color='r')
                # plt.plot(points2[:,0],points2[:,1], color='g')
                # plt.plot(points3[:,0],points3[:,1], color = 'b')
                # plt.plot(points3_t[:,0],points3_t[:,1], linewidth=0.5)
                # plt.plot(xx_2,yy_2, color = 'k')
                # plt.plot(x_values,y_values)
                
                plt.figure(figsize=(10, 6), dpi=600)
                plt.rcParams['font.family'] = 'Arial'
                plt.rcParams['font.size'] = 20
                ax_main = plt.gca()
                ax_main.invert_xaxis()
                
                ax_main.plot(points1_t[:, 0], points1_t[:, 1], 'g', linewidth=1, label='Template slit line')
                ax_main.scatter(points3_nn[:, 0], points3_nn[:, 1], marker='o', s=1, zorder=2, label='Desired stitch positions')
                ax_main.plot(points3_nn_parel[:, 0], points3_nn_parel[:, 1], color='r', linewidth=0.5, zorder=1, label='Seam line')
                ax_main.scatter(pattern_former_path[:, 0], pattern_former_path[:, 1], marker='o', color='k', s=1, label='Pattern-former stitch positions')
                
                ax_main.set_xlabel('X coordinate (mm)')
                ax_main.set_ylabel('Y coordinate (mm)')
                ax_main.legend(fontsize=15,markerscale=4, handlelength=4, loc='lower left')
                x_min, x_max = 625, 675
                y_min, y_max = -206, -196
                rect = Rectangle((x_min, y_min), x_max - x_min, y_max - y_min,
                  linewidth=1, edgecolor='black', facecolor='none')
                ax_main.add_patch(rect)
                
                ax_inset = inset_axes(ax_main, width="30%", height="30%", loc='upper right', borderpad=2)
                ax_inset.plot(points1_t[:, 0], points1_t[:, 1], 'g', linewidth=3)
                ax_inset.scatter(points3_nn[:, 0], points3_nn[:, 1], marker='o', s=5, zorder=2)
                ax_inset.plot(points3_nn_parel[:, 0], points3_nn_parel[:, 1], color='r', linewidth=1.5, zorder=1)
                ax_inset.scatter(pattern_former_path[:, 0], pattern_former_path[:, 1], marker='o', color='k', s=5)
                ax_inset.set_xlim(x_min, x_max)
                ax_inset.set_ylim(y_min, y_max)
                ax_inset.invert_xaxis()
                ax_inset.tick_params(axis='both', which='major', labelsize=12)
                
                    
                plt.savefig(self.dir_+'result.png')
                plt.close()

                file.close() 
                
                self.vision_image_pub.publish(52)
                    
        except  Exception:
            print("Failed!!!!!!!")
            #logging.error(traceback.format_exc())
            self.vision_error_pub.publish(51)
            
    
    ''' ROS Init Function '''
    def init(self):
        rospy.init_node('vision_init', anonymous=True)
        
        #Publisher
        self.vision_image_pub = rospy.Publisher('vision_image',Char,queue_size=10)
        self.vision_error_pub = rospy.Publisher('vision_error',Char,queue_size=10)
        # Subscriber
        rospy.Subscriber('/scan_sewing_gui/vision_capture_num',Char,self.visionCaptureNumCallback)
        rospy.Subscriber('/scan_sewing_gui/vision_capture',Bool,self.visionCaptureCallback)
        rospy.Subscriber('/scan_sewing_gui/vision_exposure_time',Int32,self.visionExposuretimeCallback)
        rospy.Subscriber('/scan_sewing_gui/vision_offset',Float64,self.visionOffsetCallback)
        rospy.Subscriber('/scan_sewing_gui/vision_image_processing',Empty,self.visionImageProcessingCallback)
        rospy.Subscriber('/scan_sewing_gui/vision_refresh',Empty,self.visionRefreshCallback)
        rospy.Subscriber('/scan_sewing_gui/ros_shutdown',Bool,self.rosShutdownCallback)        
        rospy.Subscriber('/scan_sewing_gui/vision_calibration',Float64MultiArray,self.visionCalibrationCallback)        
        rospy.Subscriber('/scan_sewing_gui/path_num',Int32,self.pathNumCallback)
        rospy.Subscriber('/scan_sewing_pf/pf_breakline',Bool,self.visionCaptureCallback)
        


    ''' ROS Loop '''
    def run(self):
        rate = rospy.Rate(90) #Dafault:90
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
                print("Camera Connection Failed... Reconnecting...")
                self.rt_image = False
                self.cam = None
            # gc.collect()
            rate.sleep()
            
        if self.released is False:
            self.released = True
            self.release_camera()

'''  Main '''
def main():
    vs = Vision()
    try:
        vs.init()
        #vs.image_processing()
        vs.run()
    except rospy.ROSInterruptException:
        logging.error(traceback.format_exc())


if __name__=='__main__':
	main()


