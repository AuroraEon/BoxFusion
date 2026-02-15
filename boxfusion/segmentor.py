import torch
import torch.nn.functional as F
import numpy as np
import cv2
from transformers import SegformerImageProcessor, SegformerForSemanticSegmentation

class SceneSegmenter:
    def __init__(self, device='cuda', local_path='./models/segformer-b0-finetuned-ade-512-512'):
        print(f"Loading local SegFormer model from: {local_path} ...")
        self.processor = SegformerImageProcessor.from_pretrained(local_path)
        self.model = SegformerForSemanticSegmentation.from_pretrained(local_path).to(device).eval()
        self.device = device
        
        # ADE20K 类别索引
        self.labels = {
            'wall': 0, 
            'building': 1, 
            'floor': 3, 
            'ceiling': 4,
            'door': 14   # [新增] 门的语义ID
        } 

    @torch.no_grad()
    def get_masks(self, image_np):
        inputs = self.processor(images=image_np, return_tensors="pt").to(self.device)
        outputs = self.model(**inputs)
        
        target_size = image_np.shape[:2]
        upsampled_logits = F.interpolate(
            outputs.logits, size=target_size, mode="bilinear", align_corners=False,
        )
        
        preds = upsampled_logits.argmax(dim=1)[0].cpu().numpy().astype(np.uint8)
        
        # 形态学去噪
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        
        def clean_mask(mask_in):
            mask_uint8 = mask_in.astype(np.uint8) * 255
            cleaned = cv2.morphologyEx(mask_uint8, cv2.MORPH_OPEN, kernel)
            return cleaned > 0

        wall_mask = clean_mask((preds == self.labels['wall']) | (preds == self.labels['building']))
        floor_mask = clean_mask(preds == self.labels['floor'])
        ceil_mask = clean_mask(preds == self.labels['ceiling'])
        door_mask = clean_mask(preds == self.labels['door']) # [新增] 门的 Mask
        
        return wall_mask, floor_mask, ceil_mask, door_mask