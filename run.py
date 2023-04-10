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
        tracker.initialize(frame, { 'init_bbox': bbox })
        print('initinginstidedfsd')
        while True:
            bbox, frame, initState = yield state
            if initState: break
            out = tracker.track(frame)
            state = [int(s) for s in out['target_bbox']]
            # state = [round(s) for s in [state[0], state[1], state[2] + state[0], state[3] + state[1]]]


def run():
    return run_video()
