# import importlib
# import time
# from collections import OrderedDict
# from pathlib import Path
# import numpy as np
# import cv2
# from datetime import datetime
# import glob
# from dronekit import connect

# from lib.config.default import _update_config
from lib.test.tracker.siamtpn import SiamTPN
from lib.test.parameter.siamtpn import parameters as getTPNParameters
# from lib.test.utils.controller import arm_and_takeoff, to_quaternion, send_attitude_target


# def doAction(vehicle, dx, dy):
#     if dx < 0:
#         send_attitude_target(vehicle, -5)
#         print('go')

#     elif dx > 0:
#         send_attitude_target(vehicle, 5)
#         print('stay')

#     else:
#         send_attitude_target(vehicle)

# class Tracker:
    # def __init__(self, params_dict=None):
        # self.vidVar     = "::".join(str(datetime.now()).split(" "))
        # self.name       = 'siamtpn'
        # self.params     = getTPNParameters('shufflenet_l345_192', 100)
        # self.params.cpu = params_dict['cpu']
        # self.save_video = params_dict['save']
        # self.connectArg = params_dict['connect']
        # self.wait       = params_dict['wait']
        # self.baud       = params_dict['baud']

        # ! self.connectVehicle()
        # ! arm_and_takeoff(self.vehicle, 10)
        # _update_config(self.params.cfg, params_dict)

    # def connectVehicle(self):
    #     connection_string = self.connectArg
    #     #Start SITL if no connection string specified
    #     if not connection_string:
    #         import dronekit_sitl
    #         sitl = dronekit_sitl.start_default()
    #         connection_string = sitl.connection_string()


    #     # Connect to the Vehicle
    #     print('Connecting to vehicle on: %s' % connection_string)
    #     print('wait=', self.wait)
    #     print('baud=', self.baud)
    #     if (self.wait == True):
    #         print('Not'*(not self.wait) + "Waiting")
    #         vehicle = connect(connection_string, baud=self.baud, wait_ready=self.wait)
    #     else:
    #         print("No time to wait")
    #         vehicle = connect(connection_string,baud=self.baud)
    #     self.vehicle = vehicle

def run_video():
    params     = getTPNParameters('shufflenet_l345_192', 100)
    params.cpu = True
    tracker = SiamTPN(params)

    while True:
        bbox, frame, initState = yield state
        if initState:
            tracker.initialize(frame, { 'init_bbox': bbox })

        out = tracker.track(frame)
        state = [int(s) for s in out['target_bbox']]
            # state = [round(s) for s in [state[0], state[1], state[2] + state[0], state[3] + state[1]]]


    # def run_video_old(self):

    #     class UIControl:
    #         def __init__(self):
    #             self.mode = 'init'
    #             self.target_tl = (-1, -1)
    #             self.target_br = (-1, -1)
    #             self.mode_switch = False

    #         get_tl = lambda self: self.target_tl if self.target_tl[0] < self.target_br[0] else self.target_br
    #         get_br = lambda self: self.target_br if self.target_tl[0] < self.target_br[0] else self.target_tl

    #         def mouse_callback(self, event, x, y, flags, param):
    #             if event == cv2.EVENT_LBUTTONDOWN:
    #                 self.target_tl = (x, y)
    #                 self.target_br = (x, y)
    #                 self.mode = 'select'
    #                 self.mode_switch = True
    #             elif event == cv2.EVENT_MOUSEMOVE and self.mode == 'select':
    #                 self.target_br = (x, y)
    #             elif event == cv2.EVENT_LBUTTONUP and self.mode == 'select':
    #                 self.target_br = (x, y)
    #                 self.mode = 'track'
    #                 self.mode_switch = True

    #         def get_bb(self):
    #             tl = self.get_tl()
    #             br = self.get_br()
    #             bb = [min(tl[0], br[0]), min(tl[1], br[1]), abs(br[0] - tl[0]), abs(br[1] - tl[1])]
    #             return bb   # [lx, ly, w, h]

    #     ui_control = UIControl()
    #     tracker = SiamTPN(self.params)
    #     #print('1')
    #     vidDict = {
    #         "g": '/home/user/Documents/Inet Videos/fllow me crazy guy.mp4',
    #         "d": '/home/user/Documents/Drone flights/IMG_3933.mp4', 
    #         "l": '/home/user/Videos/2LiliasVideo30.MP4',
    #         "w": '/home/user/Documents/Drone flights/NewDroneForTracker.MP4',
    #         "ll": '/home/user/Downloads/Telegram Desktop/RC_0024_220517165932.MP4',
    #         "c": 10,
    #         # "v": glob.glob("/home/user/Videos/v*")[0]
    #     }

    #     vidName =  vidDict['l']

    #     cap           = cv2.VideoCapture(vidName)
    #     width, height = int(cap.get(3)), int(cap.get(4))
    #     display_name  = 'Tracking' # 'SiamTPN'
    #     vid_X_c       = np.round(cap.get(cv2.CAP_PROP_FRAME_WIDTH) / 2, 5)
    #     vid_Y_c       = np.round(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) / 2, 5)
    #     prev_frame_time = 0
    #     cv2.namedWindow(display_name, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
    #     cv2.resizeWindow(display_name, 960, 720)
    #     cv2.setMouseCallback(display_name, ui_control.mouse_callback)
    #     onefps = 1/60

    #     if self.save_video:
    #         fourcc = cv2.VideoWriter_fourcc('M','J','P','G')
    #         out_video = cv2.VideoWriter(f'./results/saved_videos/result{self.vidVar}.mp4', fourcc, 30.0, (width, height))
    #     # ui_control.mode = 'track'
    #     # ui_control.mode_switch = True
    #     # ui_control.target_tl = (1794, 681)
    #     # ui_control.target_br = (1794 + 62, 681 + 127)
    #     output_boxes = []
    #     output_boxes.append((1794, 681, 62, 127))

    #     while True:
    #         ret, frame = cap.read()
    #         if not ret: break
    #         frame_disp = frame.copy() 
    #         new_frame_time = time.time()    
    #         fps = str(int(1 / (new_frame_time - prev_frame_time)))
    #         prev_frame_time = new_frame_time

    #         if ui_control.mode == 'select':
    #             cv2.rectangle(frame_disp, ui_control.get_tl(), ui_control.get_br(), (255, 0, 0), 2)

    #         elif ui_control.mode == 'track':
    #             bb = ui_control.get_bb()
    #             if bb[2] > 1 and bb[3] > 1:
    #                 if ui_control.mode_switch: 
    #                     ui_control.mode_switch = False
    #                     tracker.initialize(frame, { 'init_bbox': bb })

    #                 tic     = cv2.getTickCount()
    #                 out     = tracker.track(frame)
    #                 state   = [int(s) for s in out['target_bbox']]
    #                 fps     = cv2.getTickFrequency() / (cv2.getTickCount() - tic)
    #                 minWH   = min(vid_X_c, vid_Y_c)
    #                 alpha   = 15 * 2 * minWH / min(state[2], state[3]) 
    #                 alpha   = min(alpha, minWH) if alpha >= minWH * 0.5 else minWH * 0.5
    #                 alpha //= 2
    #                 output_boxes.append(state)
    #                 cv2.rectangle(frame_disp, (state[2] // 2 + state[0], state[3] // 2 + state[1]), (state[2] // 2 + state[0], state[3] // 2 + state[1]), (0, 255, 0), 3) # obj rect
    #                 cv2.rectangle(frame_disp, (state[0], state[1]), (state[2] + state[0], state[3] + state[1]), (0, 255, 0), 3) # obj center
    #                 font_color = (0, 0, 0)
    #                 cv2.putText(frame_disp, f'{ui_control.mode.capitalize()}', (20, 30), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, font_color, 1)
    #                 cv2.putText(frame_disp, 'Press q to quit', (20, 55), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, font_color, 1)

    #                 cv2.putText(frame_disp, f'X: {np.round(state[0] + state[2] / 2, 5) - vid_X_c}px, Y: {np.round(state[1] + state[3] / 2, 5) - vid_Y_c}px', (20, 80), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, font_color, 1)
    #                 cv2.rectangle(frame_disp, (int(vid_X_c - alpha), int(vid_Y_c - alpha)), (int(vid_X_c + alpha), int(vid_Y_c + alpha)), (255, 0, 0), 3) # center rect

    #                 RX = np.round(state[0] + state[2] / 2, 5) - vid_X_c
    #                 RX = 0 if abs(RX) - alpha <= 0 else RX - np.sign(RX) * alpha 
    #                 RY = np.round(state[1] + state[3] / 2, 5) - vid_Y_c
    #                 RY = 0 if abs(RY) - alpha <= 0 else RY - np.sign(RY) * alpha 
    #                 cv2.putText(frame_disp, f'RX: {np.round(RX / (2 * vid_X_c), 5)}, RY: {np.round(RY / (2 * vid_Y_c), 5)}', (20, 105), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, font_color, 1)
                    
    #                 # agent action
    #                 # doAction(self.vehicle, RX / (2 * vid_X_c), RY / (2 * vid_Y_c))


    #         cv2.rectangle(frame_disp, (int(vid_X_c), int(vid_Y_c)), (int(vid_X_c), int(vid_Y_c)), (255, 0, 0), 3) # center
    #         cv2.putText(frame_disp, f'FPS: {int(fps)}', (2 * int(vid_X_c) - 220, 35), cv2.FONT_HERSHEY_COMPLEX_SMALL, 2, (0, 0, 0), 3)
    #         cv2.imshow(display_name, frame_disp)
    #         if self.save_video: out_video.write(frame_disp)
    #         if cv2.waitKey(1) == ord('q'): break
    #         time.sleep(max(0, onefps - (time.time() - prev_frame_time)))

    #     if True:
    #         tracked_bb = np.array(output_boxes).astype(int)
    #         np.savetxt('res.txt', tracked_bb, delimiter='\t', fmt='%d')

    #     cap.release()
    #     cv2.destroyAllWindows()
