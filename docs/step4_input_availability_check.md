# Step 4 Input Availability Check

All required regeneration inputs were available through the existing `boxfusion` conda environment and local HM3D sequence folders.

Important interpreter note: the default `python` at `/home/ws/miniconda3/bin/python` did not have `torch`, so regeneration used `/home/ws/miniconda3/envs/boxfusion/bin/python`. That environment had `torch 2.5.1+cu124`, `open_clip 2.32.0`, CUDA visible, and the required Python dependencies for Stage-A.

| input | status |
|---|---|
| `models/cutr_rgbd.pth` | available |
| `config/hm3d.yaml` | available |
| `/home/ws/data/00824-Dd4bFSTQ8gi` | available, 2253 RGB/depth/pose frames |
| `/home/ws/data/00862-LT9Jq6dN3Ea` | available, 7498 RGB/depth/pose frames |
| `/home/ws/data/00829-QaLdnwvtxbs` | available, 1804 RGB/depth/pose frames |
| `data/class_features_small.pt` | available |
| `data/panoptic_categories_nomerge.txt` | available |
| `data/pst_1024_0.tiff` | available |
| `models/ViT-B-32/open_clip_pytorch_model.bin` | available |
| CUDA device 0 | available |
| `runtime_stage1_frozen_evidence/` write permission | available |

No scene was blocked by missing inputs.
