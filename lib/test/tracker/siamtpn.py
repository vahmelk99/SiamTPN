import os
import torch
import numpy as np
from torch.nn.functional import softmax

from lib.test.tracker.basetracker import BaseTracker
from lib.train.data.processing_utils import sample_target
from lib.models.siamtpn.track  import build_network
from lib.test.tracker.utils import Preprocessor
from lib.utils.box_ops import clip_box

class SiamTPN(BaseTracker):
    def __init__(self, params):
        super(SiamTPN, self).__init__(params)
        network = build_network(params.cfg)
        network.load_state_dict(torch.load(self.params.checkpoint, map_location='cpu')['net'], strict=False)
        self.cfg = params.cfg
        self.network = network if params.cpu else network.cuda()
        self.network.eval()
        self.preprocessor = Preprocessor(cpu=params.cpu)
        # for debug
        self.frame_id = 0
        self.grids = self._generate_anchors(self.cfg.MODEL.ANCHOR.NUM, self.cfg.MODEL.ANCHOR.FACTOR, self.cfg.MODEL.ANCHOR.BIAS)
        self.window = self._hanning_window(self.cfg.MODEL.ANCHOR.NUM)
        self.hanning_factor = self.cfg.TEST.HANNING_FACTOR
        self.feat_sz_tar = self.cfg.MODEL.ANCHOR.NUM

    def initialize(self, image, info: dict):
        gt_box = torch.tensor(info['init_bbox'])
        z_patch_arr, _, z_amask_arr = sample_target(image,gt_box , self.params.template_factor,
                                                    output_sz=self.params.template_size)
        template = self.preprocessor.process(z_patch_arr, z_amask_arr)
        with torch.no_grad():
            tem_feat = self.network.backbone(template)
            self.tem_feat = self.network.fpn(tem_feat)

        # save states
        self.state = info['init_bbox']
        self.frame_id = 0


    def track(self, image, info: dict = None):
        H, W, _ = image.shape
        self.frame_id += 1
        
        x_patch_arr, resize_factor, x_amask_arr = sample_target(image, self.state, self.params.search_factor, output_sz=self.params.search_size)  # (x1, y1, w, h)

        search = self.preprocessor.process(x_patch_arr, x_amask_arr)

        with torch.no_grad():
            tar_feat = self.network.backbone(search)
            tar_feat = self.network.fpn(tar_feat)
            raw_scores, boxes = self.network.head(tar_feat, self.tem_feat)
            raw_scores = raw_scores.cpu()  # B,L,2
            boxes = boxes.cpu()
            pred_boxes = boxes.reshape(-1, 4) 
            lt = self.grids[:,:2] - pred_boxes[:,:2]
            rb = self.grids[:,:2] + pred_boxes[:,2:]
            pred_boxes = torch.cat([lt, rb], -1).view(-1, 4)
            raw_scores = softmax(raw_scores, -1)[:,1].view(self.feat_sz_tar, self.feat_sz_tar)
            raw_scores = raw_scores * (1-self.hanning_factor) + self.hanning_factor * self.window
            max_v, ind = raw_scores.view(-1).topk(1)
            pred_box = pred_boxes[ind, :]  
        pred_box = (pred_box.mean(dim=0) * self.params.search_size / resize_factor).tolist()  # (cx, cy, w, h) [0,1]

        # get the final box result
        H, W, _ = image.shape
        self.state = clip_box(self.map_box_back(pred_box, resize_factor), H, W, margin=10)
        
        return {"target_bbox": self.state}

    def map_box_back(self, pred_box: list, resize_factor: float):
        #print(self.state)
        cx_prev, cy_prev = self.state[0] + 0.5 * self.state[2], self.state[1] + 0.5 * self.state[3]
        x1,y1,x2,y2 = pred_box
        cx, cy, w, h = (x1+x2)/2, (y1+y2)/2, x2-x1, y2-y1
        half_side = 0.5 * self.params.search_size / resize_factor
        cx_real = cx + (cx_prev - half_side)
        cy_real = cy + (cy_prev - half_side)
        #print(cx_real, cy_real, cx_prev, cy_prev, cx, cy)
        return [cx_real - 0.5 * w, cy_real - 0.5 * h, w, h]

    def map_box_back_batch(self, pred_box: torch.Tensor, resize_factor: float):
        cx_prev, cy_prev = self.state[0] + 0.5 * self.state[2], self.state[1] + 0.5 * self.state[3]
        cx, cy, w, h = pred_box.unbind(-1) # (N,4) --> (N,)
        half_side = 0.5 * self.params.search_size / resize_factor
        cx_real = cx + (cx_prev - half_side)
        cy_real = cy + (cy_prev - half_side)
        return torch.stack([cx_real - 0.5 * w, cy_real - 0.5 * h, w, h], dim=-1)

    def _hanning_window(self, num):
        hanning = np.hanning(num)
        window = np.outer(hanning, hanning)
        window = torch.from_numpy(window)
        return window

    def _generate_anchors(self, num=20, factor=1, bias=0.5):
        """
        generate anchors for each sampled point
        """
        x = np.arange(num)
        y = np.arange(num)
        xx, yy = np.meshgrid(x, y) 
        xx = (factor * xx + bias) / num 
        yy = (factor * yy + bias) / num
        xx = torch.from_numpy(xx).view(-1).float()
        yy = torch.from_numpy(yy).view(-1).float()
        grids = torch.stack([xx, yy],-1) # N 2
        return grids
