# !git clone https://github.com/datvuthanh/HybridNets
# %cd HybridNets
# !pip install -r requirements.txt
# !curl --create-dirs -L -o weights/hybridnets.pth https://github.com/datvuthanh/HybridNets/releases/download/v1.0/hybridnets.pth
# on Windows
# curl.exe --create-dirs -L -o weights/hybridnets.pth https://github.com/datvuthanh/HybridNets/releases/download/v1.0/hybridnets.pth

import cv2
import numpy as np
import torch
import os
from torchvision import transforms
from HybridNets.backbone import HybridNetsBackbone
from HybridNets.utils.utils import letterbox, Params
import matplotlib.pyplot as plt

use_cuda = torch.cuda.is_available()
#-----------------change on each computer-------------------------------------------------------------------------------------
params = Params(f'/home/alemomo/Scrivania/H2politO/catkin_ws/src/juno_aug/src/HybridNets/projects/bdd100k.yml')
#-----------------------------------------------------------------------------------------------------------------------------
color_list_seg = {}

for seg_class in params.seg_list:
    color_list_seg[seg_class] = list(np.random.choice(range(256), size=3))

def preprocessing_image(image):
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    resized_shape = params.model['image_size']

    normalize = transforms.Normalize(
        mean=params.mean, std=params.std
    )
    transform = transforms.Compose([
        transforms.ToTensor(),
        normalize,
    ])

    if isinstance(resized_shape, list):
        resized_shape = max(resized_shape)
    shapes = []

    h0, w0 = image.shape[:2]  
    r = resized_shape / max(h0, w0) 
    input_img = cv2.resize(image, (int(w0 * r), int(h0 * r)), interpolation=cv2.INTER_AREA)
    h, w = input_img.shape[:2]

    (input_img, _), ratio, pad = letterbox((input_img, None), resized_shape, auto=True,
                                            scaleup=False)

    if use_cuda:
        input_tensor = transform(input_img).unsqueeze(0).cuda()
    else:
        input_tensor = transform(input_img).unsqueeze(0).cpu()   
    shapes.append(((h0, w0), ((h / h0, w / w0), pad)))  
    return input_tensor, shapes

def initialize_model():
    MULTICLASS_MODE: str = "multiclass"
    print(os.system("pwd"))

    anchors_ratios = params.anchors_ratios
    anchors_scales = params.anchors_scales
    obj_list = params.obj_list
    seg_list = params.seg_list

    #-----------------change on each computer-------------------------------------------------------------------------------------
    weights_path = '/home/alemomo/Scrivania/H2politO/catkin_ws/src/juno_aug/src/HybridNets/weights/hybridnets.pth'
    #-----------------------------------------------------------------------------------------------------------------------------
    state_dict = torch.load(weights_path, map_location='cuda' if use_cuda else 'cpu')

    weight_last_layer_seg = state_dict['segmentation_head.0.weight']

    seg_mode = MULTICLASS_MODE

    model = HybridNetsBackbone(compound_coef=3, num_classes=len(obj_list), ratios=eval(anchors_ratios),
                            scales=eval(anchors_scales), seg_classes=len(seg_list), seg_mode=seg_mode)           # lasciare None sulla backbone è ok

    model.load_state_dict(state_dict)
    model.requires_grad_(False)
    model.eval()

    if use_cuda:
        model = model.cuda()
    else:
        model = model.cpu()
    return model

def preprocessing_mask(seg, shapes, show=False):
    _, seg_mask = torch.max(seg, 1)
    seg_mask_ = seg_mask[0].squeeze().cpu().numpy()
    pad_h = int(shapes[0][1][1][1])
    pad_w = int(shapes[0][1][1][0])
    seg_mask_ = seg_mask_[pad_h:seg_mask_.shape[0]-pad_h, pad_w:seg_mask_.shape[1]-pad_w]
    seg_mask_ = cv2.resize(seg_mask_, dsize=shapes[0][0][::-1], interpolation=cv2.INTER_NEAREST)
    color_seg = np.zeros((seg_mask_.shape[0], seg_mask_.shape[1], 3), dtype=np.uint8)
    for index, seg_class in enumerate(params.seg_list):
        color_seg[seg_mask_ == index+1] = color_list_seg[seg_class]
    color_seg = color_seg[..., ::-1]  

    color_mask = np.mean(color_seg, 2)
    _, color_mask_bn = cv2.threshold(color_mask,0,255, cv2.THRESH_BINARY)
    if show:
        plt.imshow(color_mask_bn)
        plt.show()
    return color_mask_bn.astype('uint8')