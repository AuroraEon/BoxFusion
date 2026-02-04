import torch
import open_clip
import numpy as np
import os

# 1. 设置你想用的轻量模型
MODEL_NAME = 'ViT-B-32' 
local_model_path = '/home/aurora/workspace1/BoxFusion/models/ViT-B-32/open_clip_pytorch_model.bin'

# 2. 加载类别列表 (从你的 demo.py 参数来看是这个文件)
CLASS_TXT_PATH = './data/panoptic_categories_nomerge.txt'
OUTPUT_PATH = './data/class_features_small.pt' # 保存为新名字

def generate_features():
    print(f"Loading model: {MODEL_NAME}...")
    print(f"Loading local model from: {local_model_path}")

    # 3. 加载模型
    model, _, preprocess = open_clip.create_model_and_transforms(
        model_name=MODEL_NAME, 
        pretrained=local_model_path # 关键修改在这里
    )
    tokenizer = open_clip.get_tokenizer(MODEL_NAME)
    
    model.eval()
    if torch.cuda.is_available():
        model.cuda()

    # 读取类别文本
    print(f"Reading classes from {CLASS_TXT_PATH}...")
    classes = np.genfromtxt(CLASS_TXT_PATH, delimiter='\n', dtype=str)
    
    # 构造 Prompt (例如 "a photo of a chair")
    # BoxFusion 原始逻辑可能包含特定的 prompt 模板，这里假设直接用类别名，
    # 如果原始代码有模板 (如 "a {class}"), 需要加上。
    text_inputs = tokenizer(classes.tolist()).cuda()

    print("Encoding text features...")
    with torch.no_grad():
        text_features = model.encode_text(text_inputs)
        text_features /= text_features.norm(dim=-1, keepdim=True) # 归一化

    print(f"Saving to {OUTPUT_PATH}...")
    torch.save(text_features, OUTPUT_PATH)
    print("Done!")

if __name__ == "__main__":
    generate_features()