import os 
import sys
env_path = os.path.dirname(__file__)
if env_path not in sys.path:
    sys.path.append(env_path)
from lib.test.tracker.siamtpn import SiamTPN
from lib.test.parameter.siamtpn import parameters as getTPNParameters

def run_video():
    params     = getTPNParameters('shufflenet_l345_192', 100)
    params.cpu = True
    tracker = SiamTPN(params)
    while True:
        state = [-1] * 4
        bbox, frame, initState = yield state
        tracker.initialize(frame, { 'init_bbox': [bbox[0], bbox[1], bbox[2] - bbox[0], bbox[3] - bbox[1]] })
        while True:
            bbox, frame, initState = yield state
            if initState: break
            out = tracker.track(frame)
            state = out['target_bbox']
            state = list(map(round, [state[0], state[1], state[2] + state[0], state[3] + state[1]]))

def run():
    return run_video()
