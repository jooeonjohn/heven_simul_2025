#! /usr/bin/env python3

import rospy, cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from mm_common.msg import lane_info
from math import *

def color_filter_white(image):
    """
        HLS 필터 사용
        
        lower & upper : 흰색으로 판단할 minimum pixel 값
        white_mask : lower과 upper 사이의 값만 남긴 mask
        masked : cv2.bitwise_and() 함수를 통해 흰색인 부분 제외하고는 전부 검정색 처리
    """
    hls = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    
    lower = np.array([170, 170, 150])
    upper = np.array([255, 255, 255])

    white_mask = cv2.inRange(hls, lower, upper)
    masked = cv2.bitwise_and(image, image, mask = white_mask)

    return masked

def color_filter_yellow(image):
    """
        HLS 필터 사용
        
        lower & upper : 흰색으로 판단할 minimum pixel 값
        white_mask : lower과 upper 사이의 값만 남긴 mask
        masked : cv2.bitwise_and() 함수를 통해 흰색인 부분 제외하고는 전부 검정색 처리
    """
    hls = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    
    lower = np.array([150, 100, 0])
    upper = np.array([255, 220, 80])

    white_mask = cv2.inRange(hls, lower, upper)
    masked = cv2.bitwise_and(image, image, mask = white_mask)

    return masked

class lane_detect():
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node('lane_detection_node', anonymous=False)
        # 좌상 - 좌하 - 우하 - 우상
        self.source = [[260, 270], [190, 360], [510, 360], [395, 270]]
        
        self.destination = [[80, 0], [80, 480], [560, 480], [560, 0]]
        self.image_orig = None
        self.cluster_threshold = 30

        rospy.Subscriber('/image_jpeg2/compressed', CompressedImage, self.camera_orig_callback)

        self.pub = rospy.Publisher("/lane_result", lane_info, queue_size=1)
    
    def camera_orig_callback(self, data):
        self.image_orig = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
        pub= self.lane_detect()
        self.pub.publish(pub)

    def warpping(self, image):
        """
            차선을 BEV로 변환하는 함수
            
            Return
            1) _image : BEV result image
            2) minv : inverse matrix of BEV conversion matrix
        """

        source = np.float32(self.source)
        destination = np.float32(self.destination)
        
        M = cv2.getPerspectiveTransform(source, destination)
        Minv = cv2.getPerspectiveTransform(destination, source)
        
        warp_image = cv2.warpPerspective(image, M, (640, 480), flags=cv2.INTER_LINEAR)

        return warp_image, Minv

    def get_cluster(self, positions):
        '''
        group positions that are close to each other
        '''
        clusters = []
        for position in positions:
            if 5 <= position < 315:
                for cluster in clusters:
                    if abs(cluster[0] - position) < self.cluster_threshold:
                        cluster.append(position)
                        break
                else:
                    clusters.append([position])
        lane_candidates = [np.mean(cluster) for cluster in clusters]

        return lane_candidates


    
    def high_level_detect(self, hough_img):
        
        nwindows = 10       # window 개수
        margin = 50
        minpix = 30          # 차선 인식을 판정하는 최소 픽셀 수

        # 아래 30%만 histogram 계산
        histogram = np.sum(hough_img[hough_img.shape[0]//10*8:,:],   axis=0)
        # plt.plot(histogram)
        # plt.show()
        
        midpoint = np.int32(histogram.shape[0]/2)

        # 왼쪽 절반에서 0이 아닌 값만 선택
        left_non_zero_indices = np.where(histogram[:midpoint] != 0)[0]
        if len(left_non_zero_indices) == 0:
            leftx_current = 160
        else:
            left_top_five_indices = np.argpartition(histogram[:midpoint], -5)[-5:]
            left_top_five_indices = left_top_five_indices[np.argsort(histogram[:midpoint][left_top_five_indices])][::-1]
            leftx_current_list = np.array([left_top_five_indices[i] for i in range(5)])
            leftx_current = np.max(leftx_current_list)


        # 오른쪽 절반에서 0이 아닌 값만 선택
        right_non_zero_indices = np.where(histogram[midpoint:] != 0)[0]
        if len(right_non_zero_indices) == 0:
            rightx_current = 480
        else:
            right_top_five_indices = np.argpartition(histogram[midpoint:], -5)[-5:]
            right_top_five_indices = right_top_five_indices[np.argsort(histogram[midpoint:][right_top_five_indices])][::-1]
            rightx_current_list = np.array([right_top_five_indices[i] for i in range(5)])
            rightx_current = np.min(rightx_current_list) + midpoint

        # 차량 주행 중에 차선인식이 일어나지 않는 경우,
        # 해당 차선을 인식할 때 사용되던 window들의 default 위치를 조정.
        if midpoint - 5 <= leftx_current or leftx_current <= 5:
            leftx_current = int(midpoint - 320 / 2)
                        
        if 635 <= rightx_current or rightx_current <= midpoint + 5:
            rightx_current = int(midpoint + 320 / 2)

        save_leftx = leftx_current
        save_rightx = rightx_current
        # 쌓을 window의 height 설정
        window_height = np.int32(hough_img.shape[0]/nwindows)
        
        # 240*320 픽셀에 담긴 값중 0이 아닌 값을 저장한다.
        # nz[0]에는 index[row][col] 중에 row파트만 담겨있고 nz[1]에는 col이 담겨있다.
        nz = hough_img.nonzero()

        left_lane_inds = []
        right_lane_inds = []
        
        global lx, ly, rx, ry
        lx, ly, rx, ry = [], [], [], []

        global out_img
        out_img = np.dstack((hough_img, hough_img, hough_img))*255

        cnt = 0
        
        left_sum = 0
        right_sum = 0

        total_loop = 0

        for window in range(nwindows-4):
            
            # bounding box 크기 설정
            win_yl = hough_img.shape[0] - (window+1)*window_height
            win_yh = hough_img.shape[0] - window*window_height

            win_xll = leftx_current - margin
            win_xlh = leftx_current + margin
            win_xrl = rightx_current - margin
            win_xrh = rightx_current + margin

            # print(leftx_current, rightx_current)

            # out image에 bounding box 시각화
            cv2.rectangle(out_img,(win_xll,win_yl),(win_xlh,    win_yh),    (0,255,0), 2) 
            cv2.rectangle(out_img,(win_xrl,win_yl),(win_xrh,    win_yh),    (0,255,0), 2) 

            # 흰점의 픽셀들 중에 window안에 들어오는 픽셀인지 여부를 판단하여 
            # good_left_inds와 good_right_inds에 담는다.
            good_left_inds  = ((nz[0] >= win_yl)&(nz[0] < win_yh)&(nz[1] >= win_xll)&(nz[1] < win_xlh)).nonzero()[0]
            good_right_inds = ((nz[0] >= win_yl)&(nz[0] < win_yh)&(nz[1] >= win_xrl)&(nz[1] < win_xrh)).nonzero()[0]

            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            # print(self.get_cluster(nz[1]    [good_left_inds]))
            cluster_left = self.get_cluster(nz[1][good_left_inds])
            # nz[1]값들 중에 good_left_inds를 index로 삼는 nz[1]들의 평균을 구해서 leftx_current를 갱신한다.
            if len(good_left_inds) > minpix and len(cluster_left) > 0:
                # leftx_current = np.int32(np.mean(nz[1]    [good_left_inds])   )
                leftx_current = np.int32(np.max(cluster_left))
            elif len(good_left_inds) > minpix and len(cluster_left) == 0:
                leftx_current = np.int32(np.mean(nz[1][good_left_inds]))

            #lx ly rx ry에 x,y좌표들의 중심점들을 담아둔다.
            lx.append(leftx_current)
            ly.append((win_yl + win_yh)/2)

            left_sum += leftx_current
            
            cluster_right = self.get_cluster(nz[1][good_right_inds])
            # nz[1]값들 중에 good_right_inds를 index로 삼는 nz[1]들의 평균을 구해서 rightx_current를 갱신한다.            
            if len(good_right_inds) > minpix and len(cluster_right) > 0:        
                # rightx_current = np.int32(np.mean(nz[1]       [good_right_inds]))
                rightx_current = np.int32(np.min(cluster_right))
            elif len(good_right_inds) > minpix and len(cluster_right) == 0:
                rightx_current = np.int32(np.mean(nz[1][good_right_inds]))

            rx.append(rightx_current)
            ry.append((win_yl + win_yh)/2)

            right_sum += rightx_current
            
            total_loop += 1

        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        # lfit = np.polyfit(np.array(ly[1:]),np.array(lx[1:]),2)
        # rfit = np.polyfit(np.array(ry[1:]),np.array(rx[1:]),2)
        lfit = np.polyfit(np.array(ly),np.array(lx),1)
        rfit = np.polyfit(np.array(ry),np.array(rx),1)

        #out_img에서 왼쪽 선들의 픽셀값을 BLUE로, 
        #오른쪽 선들의 픽셀값을 RED로 바꿔준다.
        out_img[nz[0][left_lane_inds], nz[1][left_lane_inds]] = [255, 0, 0]
        out_img[nz[0][right_lane_inds] , nz[1][right_lane_inds]]= [0, 0, 255]    

        left_avg = left_sum / total_loop
        right_avg = right_sum / total_loop

        # print(save_leftx, save_rightx)

        left_det = True
        right_det = True

        if save_leftx == 160:
            left_avg = right_avg - 450
            left_det = False
                        
        if save_rightx == 480:
            right_avg = left_avg + 450
            right_det = False

        return lfit, rfit, left_det, right_det, left_avg, right_avg
    
    def lane_detect(self):
        if self.image_orig is not None:
            self.image_orig = cv2.resize(self.image_orig, (640, 480))
            self.image_copy = self.image_orig.copy()
            # cv2.polylines(self.image_copy, [np.array(self.source)], True, (255, 0, 255), 2)
            # cv2.namedWindow('Original_cam')
            # cv2.moveWindow('Original_cam', 0, 0)
            # cv2.imshow('Original_cam', self.image_copy)
            warpped_img_orig, minv_orig = self.warpping(self.image_orig)
            # cv2.imshow('warp', warpped_img_orig)

            blurred_img = cv2.GaussianBlur(warpped_img_orig, (0, 0), 1)
            # cv2.imshow('Gaussian Blur', blurred_img)

            w_f_img_white = color_filter_white(blurred_img)
            w_f_img_yellow = color_filter_yellow(blurred_img)
            # w_f_img_yellow_cropped = w_f_img_yellow[:, 0:450]
            # w_f_img_white_cropped = w_f_img_white[:, 450:640]
            w_f_img = cv2.bitwise_or(w_f_img_white, w_f_img_yellow)
            # cv2.imshow('Color filter1', w_f_img)
            # cv2.imshow('Color filter yellow', w_f_img_yellow)
            # cv2.imshow('Color filter white', w_f_img_white)

            ## BEV에서 원근법에 의한 차선 팽창 현상 없애기
            # warpped_img = cv2.Sobel(w_f_img, cv2.CV_8U, 1, 0, ksize=5)
            # kk2 = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            # warpped_img = cv2.morphologyEx(warpped_img, cv2.MORPH_OPEN, kk2)
            # warpped_img = cv2.dilate(warpped_img, kk2)

            # kk2 = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))


            # cv2.namedWindow('BEV')
            # cv2.moveWindow('BEV', 700, 0)
            # cv2.imshow('BEV1', warpped_img)
            
            # blurred_img = cv2.GaussianBlur(warpped_img, (7, 7), 5)
            # cv2.namedWindow('Blurred')
            # cv2.moveWindow('Blurred', 350, 0)
            # cv2.imshow('Blurred', blurred_img)
            
            # w_f_img = color_filter(blurred_img)
            # cv2.namedWindow('Color filter')
            # cv2.moveWindow('Color filter', 0, 550)
            # cv2.imshow('Color filter', w_f_img)
            
            grayscale = cv2.cvtColor(w_f_img, cv2.COLOR_BGR2GRAY)
            
            ret, thresh = cv2.threshold(grayscale, 140, 255, cv2.THRESH_BINARY) #170, 255
            
            canny_img = cv2.Canny(thresh, 10, 100)
            # cv2.namedWindow('Canny')
            # cv2.moveWindow('Canny', 700, 600)
            # cv2.imshow('Canny', canny_img)
            
            
            lines = cv2.HoughLines(canny_img, 1, np.pi/180, 50)

            hough_img = np.zeros((480, 640, 3), dtype=np.uint8)

            if lines is not None:
                for line in lines:
                    rho, theta = line[0]
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a * rho
                    y0 = b * rho
                    x1 = int(x0 + 1000 * (-b))
                    y1 = int(y0 + 1000 * (a))
                    x2 = int(x0 - 1000 * (-b))
                    y2 = int(y0 - 1000 * (a))

                    # Convert angle to degrees for filtering
                    angle_deg = theta * 180 / np.pi

                    # Optional filter: only near-vertical lines
                    # if abs(angle_deg - 90) < 30:
                    cv2.line(hough_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # cv2.imshow('Hough Lines', hough_img)

            # left_fit, right_fit, ignore_right, l_avg, r_avg = self.high_level_detect(hough_img)
            left_fit, right_fit, left_det, right_det, l_avg, r_avg = self.high_level_detect(canny_img)
            
            # left_fit = np.polyfit(np.array(ly),np.array(lx),1)
            # right_fit = np.polyfit(np.array(ry),np.array(rx),1)
            # print("left_fit: ", left_fit)
            # print("right_fit: ", right_fit)
            
            if left_det and right_det :
                line_left = np.poly1d(left_fit)
                line_right = np.poly1d(right_fit)
            elif left_det:
                line_left = np.poly1d(left_fit)
                line_right = np.poly1d(left_fit)
            elif right_det:
                line_right = np.poly1d(right_fit)
                line_left = np.poly1d(right_fit)
            else : 
                pub_msg = lane_info()
                pub_msg.left_x = 225
                pub_msg.right_x = 225
                pub_msg.left_theta = 90
                pub_msg.right_theta = 90
                return pub_msg
            # print("line_left: ", line_left)
            # print("line_right: ", line_right)


            # 좌,우측 차선의 휘어진 각도
            left_line_angle = degrees(atan(line_left[1])) + 90
            right_line_angle = degrees(atan(line_right[1])) + 90
            
            shift_const = 320
    
            final_left_angle = left_line_angle  
            final_right_angle = right_line_angle
            cv2.imshow("Sliding Window", out_img)
            cv2.waitKey(1)

            pub_msg = lane_info()
            pub_msg.left_x = int(abs(shift_const - l_avg))
            pub_msg.right_x = int(abs(r_avg - shift_const))
            pub_msg.left_theta = final_left_angle
            pub_msg.right_theta = final_right_angle

            print("----------outdoor----------")
            print("left avg : %3f   right avg : %3f" %(l_avg, r_avg))
            print("left_th : %3f   right_th : %3f" %(left_line_angle, right_line_angle))
            print("left : %3f   right : %3f    sum : %3f" %(pub_msg.left_x, pub_msg.right_x, pub_msg.left_x + pub_msg.right_x))

            return pub_msg

if __name__ == "__main__":

    if not rospy.is_shutdown():
        lane_detect()
        rospy.spin()