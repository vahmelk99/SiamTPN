import importlib
import os
from collections import OrderedDict
from lib.test.evaluation.environment import env_settings
import time
import cv2 as cv
from lib.test.evaluation.multi_object_wrapper import MultiObjectWrapper
from lib.config.default import _update_config
from pathlib import Path
import numpy as np
import cv2


def trackerlist(name: str, parameter_name: str, dataset_name: str, run_ids = None, display_name: str = None,
                result_only=False, params_dict=None):
    """Generate list of trackers.
    args:
        name: Name of tracking method.
        parameter_name: Name of parameter file.
        run_ids: A single or list of run_ids.
        display_name: Name to be displayed in the result plots.
    """
    if run_ids is None or isinstance(run_ids, int):
        run_ids = [run_ids]
    return [Tracker(name, parameter_name, dataset_name, run_id, display_name, result_only, params_dict) for run_id in run_ids]


class Tracker:
    """Wraps the tracker for evaluation and running purposes.
    args:
        name: Name of tracking method.
        parameter_name: Name of parameter file.
        run_id: The run id.
        display_name: Name to be displayed in the result plots.
    """

    def __init__(self, name: str, parameter_name: str, dataset_name: str, run_id: int = None, display_name: str = None,
                 result_only=False, params_dict=None):
        assert run_id is None or isinstance(run_id, int)

        self.name = name
        self.parameter_name = parameter_name
        self.dataset_name = dataset_name
        self.run_id = run_id
        self.display_name = display_name
        self.epoch = params_dict['checkpoint']
        self.params = self.get_parameters()
        self._update_params(params_dict)
        _update_config(self.params.cfg, params_dict)

        env = env_settings()
        #print(env)
        if self.run_id is None:
            self.results_dir = '{}/{}/{}'.format(env.results_path, self.name, self.parameter_name)
        else:
            self.results_dir = '{}/{}/{}_{:03d}'.format(env.results_path, self.name, self.parameter_name, self.run_id)
        if result_only:
            self.results_dir = '{}/{}/{}'.format(env.results_path, "LaSOT", self.name)
        #print("save path", self.results_dir)
        tracker_module_abspath = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                                              '..', 'tracker', '%s.py' % self.name))
        if os.path.isfile(tracker_module_abspath):
            tracker_module = importlib.import_module('lib.test.tracker.{}'.format(self.name))
            self.tracker_class = tracker_module.get_tracker_class()
        else:
            self.tracker_class = None

    def create_tracker(self, params):
        #print(params)
        tracker = self.tracker_class(params)
        return tracker

    def _update_params(self, params_dict):
        self.params.cfg.TEST.HANNING_FACTOR = params_dict['windows_factor']
        self.params.debug = params_dict['debug']
        self.params.cpu = params_dict['cpu']


    def run_sequence(self, seq, debug=None,multiobj_mode=None, start=1):
        """Run tracker on sequence.
        args:
            seq: Sequence to run the tracker on.
            visualization: Set visualization flag (None means default value specified in the parameters).
            debug: Set debug level (None means default value specified in the parameters).
            visdom_info: Visdom info.
            multiobj_mode: Which mode to use for multiple objects.
        """
        params = self.params

        debug_ = debug
        if debug is None:
            debug_ = getattr(params, 'debug', 0)

        params.debug = debug_

        # Get init information
        init_info = seq.init_info()
        is_single_object = not seq.multiobj_mode

        if multiobj_mode is None:
            multiobj_mode = getattr(params, 'multiobj_mode', getattr(self.tracker_class, 'multiobj_mode', 'default'))

        if multiobj_mode == 'default' or is_single_object:
            tracker = self.create_tracker(params)
        elif multiobj_mode == 'parallel':
            tracker = MultiObjectWrapper(self.tracker_class, params)
        else:
            raise ValueError('Unknown multi object mode {}'.format(multiobj_mode))

        output = self._track_sequence(tracker, seq, init_info, start=start)
        return output

    def _track_sequence(self, tracker, seq, init_info, start=1):

        output = {'target_bbox': [],
                  'fps': []}
        if tracker.params.save_all_boxes:
            output['all_boxes'] = []
            output['all_scores'] = []

        def _store_outputs(tracker_out: dict, defaults=None):
            defaults = {} if defaults is None else defaults
            for key in output.keys():
                val = tracker_out.get(key, defaults.get(key, None))
                if key in tracker_out or val is not None:
                    output[key].append(val)

        # Initialize
        image = self._read_image(seq.frames[0])

        out = tracker.initialize(image, init_info)
        if out is None:
            out = {}

        prev_output = OrderedDict(out)
        init_default = {'target_bbox': init_info.get('init_bbox'),
                        'fps': 0}
        if tracker.params.save_all_boxes:
            init_default['all_boxes'] = out['all_boxes']
            init_default['all_scores'] = out['all_scores']

        _store_outputs(out, init_default)

        for frame_num, frame_path in enumerate(seq.frames[start:], start=start):
            image = self._read_image(frame_path)

            tic = cv2.getTickCount()

            info = seq.frame_info(frame_num)
            info['previous_output'] = prev_output

            out = tracker.track(image, info)
            prev_output = OrderedDict(out)
            _store_outputs(out, {'fps': cv2.getTickFrequency()/(cv2.getTickCount() - tic)})

        for key in ['target_bbox', 'all_boxes', 'all_scores']:
            if key in output and len(output[key]) <= 1:
                output.pop(key)

        return output

    def run_video(self, videofilepath, optional_box=None, debug=None, visdom_info=None, save_results=False, save_video=True):
        """Run the tracker with the vieofile.
        args:
            debug: Debug level.
        """

        class UIControl:
            def __init__(self):
                self.mode = 'init'  # init, select, track
                self.target_tl = (-1, -1)
                self.target_br = (-1, -1)
                self.mode_switch = False

            def mouse_callback(self, event, x, y, flags, param):
                if event == cv2.EVENT_LBUTTONDOWN:# and self.mode == 'init':
                    self.target_tl = (x, y)
                    self.target_br = (x, y)
                    self.mode = 'select'
                    self.mode_switch = True
                elif event == cv2.EVENT_MOUSEMOVE and self.mode == 'select':
                    self.target_br = (x, y)
                elif event == cv2.EVENT_LBUTTONUP and self.mode == 'select':
                    self.target_br = (x, y)
                    self.mode = 'track'
                    self.mode_switch = True

            def get_tl(self):
                return self.target_tl if self.target_tl[0] < self.target_br[0] else self.target_br

            def get_br(self):
                return self.target_br if self.target_tl[0] < self.target_br[0] else self.target_tl

            def get_bb(self):
                tl = self.get_tl()
                br = self.get_br()

                bb = [min(tl[0], br[0]), min(tl[1], br[1]), abs(br[0] - tl[0]), abs(br[1] - tl[1])]
                return bb   # [lx, ly, w, h]

        ui_control = UIControl()

        #params = self.get_parameters()
        params = self.params
        debug_ = debug
        if debug is None:
            debug_ = getattr(params, 'debug', 0)
        params.debug = debug_

        params.tracker_name = self.name
        params.param_name = self.parameter_name
        # self._init_visdom(visdom_info, debug_)

        multiobj_mode = getattr(params, 'multiobj_mode', getattr(self.tracker_class, 'multiobj_mode', 'default'))

        if multiobj_mode == 'default':
            tracker = self.create_tracker(params)

        elif multiobj_mode == 'parallel':
            tracker = MultiObjectWrapper(self.tracker_class, params, self.visdom, fast_load=True)
        else:
            raise ValueError('Unknown multi object mode {}'.format(multiobj_mode))

        #assert os.path.isfile(videofilepath), "Invalid param {}".format(videofilepath)
        #", videofilepath must be a valid videofile"

        output_boxes = []

        cap = cv.VideoCapture(videofilepath)
        # cap2 = cv2.VideoCapture(0)
        width  = int(cap.get(3))
        height = int(cap.get(4))
        display_name = 'SiamTPN'
        cv.namedWindow(display_name, cv.WINDOW_NORMAL | cv.WINDOW_KEEPRATIO)
        cv.resizeWindow(display_name, 960, 720)
        cv2.setMouseCallback(display_name, ui_control.mouse_callback)

        if save_video:
            fourcc = cv.VideoWriter_fourcc('M','J','P','G')
            out_video = cv.VideoWriter('./results/result_plots/result.avi', fourcc, 15.0, (width,height))

        def _build_init_info(box):
            return {'init_bbox': box}

        # if success is not True:
        #     print("Read frame from {} failed.".format(videofilepath))
        #     exit(-1)
        # if optional_box is not None:
        #     assert isinstance(optional_box, (list, tuple))
        #     assert len(optional_box) == 4, "valid box's foramt is [x,y,w,h]"
        #     tracker.initialize(frame, _build_init_info(optional_box))
        #     output_boxes.append(optional_box)
        # else:
        prev_frame_time = 0
        while True:
            ret, frame = cap.read()
            frame_disp = frame.copy()  
            new_frame_time = time.time()    
            fps = str(int(1/(new_frame_time-prev_frame_time)))
            prev_frame_time = new_frame_time          
            if ui_control.mode == 'select':
                cv2.rectangle(frame_disp, ui_control.get_tl(), ui_control.get_br(), (255, 0, 0), 2)

            elif ui_control.mode == 'track':
                if ui_control.mode_switch: 
                    ui_control.mode_switch = False
                    tracker.initialize(frame, _build_init_info(ui_control.get_bb()))

                tic = cv2.getTickCount()
                out = tracker.track(frame)
                state = [int(s) for s in out['target_bbox']]
                # output_boxes.append(state)
                fps = cv2.getTickFrequency()/(cv2.getTickCount() - tic)
                cv.rectangle(frame_disp, (state[0], state[1]), (state[2] + state[0], state[3] + state[1]), (0, 255, 0), 5)
                font_color = (0, 0, 0)
                cv.putText(frame_disp, f'{ui_control.mode.capitalize()}', (20, 30), cv.FONT_HERSHEY_COMPLEX_SMALL, 1, font_color, 1)
                cv.putText(frame_disp, 'Press q to quit', (20, 55), cv.FONT_HERSHEY_COMPLEX_SMALL, 1, font_color, 1)
            cv.putText(frame_disp, f'FPS: {fps}', (20, 80), cv.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 0, 0), 1)                   
            cv.imshow(display_name, frame_disp)
            # out_video.write(frame_disp)
            key = cv.waitKey(1)
            if key == ord('q'): break

        # When everything done, release the capture
        cap.release()
        cv.destroyAllWindows()

        # if save_results:
        #     if not os.path.exists(self.results_dir):
        #         os.makedirs(self.results_dir)
        #     video_name = Path(videofilepath).stem
        #     base_results_path = os.path.join(self.results_dir, 'video_{}'.format(video_name))

        #     tracked_bb = np.array(output_boxes).astype(int)
        #     bbox_file = '{}.txt'.format(base_results_path)
        #     np.savetxt(bbox_file, tracked_bb, delimiter='\t', fmt='%d')


    def get_parameters(self):
        """Get parameters."""
        param_module = importlib.import_module('lib.test.parameter.{}'.format(self.name))
        params = param_module.parameters(self.parameter_name, self.epoch)
        return params


    def _read_image(self, image_file: str):
        if isinstance(image_file, str):
            im = cv.imread(image_file)
            return cv.cvtColor(im, cv.COLOR_BGR2RGB)
        else:
            raise ValueError("type of image_file should be str or list")



