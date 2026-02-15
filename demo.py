import os
os.environ["PYTORCH_ENABLE_MPS_FALLBACK"] = "1"

import argparse
import glob
import itertools
import json
import numpy as np
import rerun
import rerun.blueprint as rrb
import yaml
import torch
import torchvision
import sys
import uuid
import open3d as o3d
from pathlib import Path
from PIL import Image
from scipy.spatial.transform import Rotation
from tools.utils import * 
import open_clip 
import torch.nn.functional as F
import time
import cv2

from boxfusion.cubify_transformer import make_cubify_transformer

from boxfusion.instances import Instances3D
from boxfusion.preprocessor import Augmentor, Preprocessor

from boxfusion.box_manager import BoxManager
from boxfusion.box_fusion import BoxFusion

from boxfusion.room_segmenter import GridRoomSegmenter
from boxfusion.segmentor import SceneSegmenter
import yaml

def save_room_mapping_to_yaml(all_pred_box, room_segmenter, output_path="room_objects.yaml"):
    """
    将物体按房间归类并保存为 YAML，包含针对靠墙物体的邻域修正逻辑。
    """
    # 1. 安全检查
    if all_pred_box is None:
        print("跳过 YAML 导出：未检测到任何物体。")
        return
    
    if room_segmenter.last_room_markers is None:
        print("跳过 YAML 导出：房间分割图尚未生成。")
        return

    # 提取物体的 3D 信息
    # pred_boxes_3d.tensor: [N, 7] -> (x, y, z, w, h, l, yaw)
    box_tensors = all_pred_box.pred_boxes_3d.tensor.cpu().numpy()
    categories = all_pred_box.categories
    instance_ids = all_pred_box.init_id.cpu().numpy()
    
    # 2. 转换世界坐标到栅格坐标
    u_coords, v_coords = room_segmenter._world_to_grid(box_tensors[:, :2])
    markers = room_segmenter.last_room_markers
    wall_label = np.max(markers) # 假设墙壁是最大的标签值
    
    room_data = {}
    search_radius = 10 # 搜索范围：21x21 像素 (约 1.05m x 1.05m)

    for i in range(len(box_tensors)):
        u, v = u_coords[i], v_coords[i]
        label = markers[v, u]
        
        # 3. 邻域修正逻辑：如果点落在墙壁(wall_label)或无效区(<=1)
        if label <= 1 or label == wall_label:
            # 截取邻域切片
            v_start, v_end = max(0, v - search_radius), min(markers.shape[0], v + search_radius)
            u_start, u_end = max(0, u - search_radius), min(markers.shape[1], u + search_radius)
            patch = markers[v_start:v_end, u_start:u_end]
            
            # 过滤掉背景、噪声和墙壁，只留房间标签
            valid_labels = patch[(patch > 1) & (patch != wall_label)]
            
            if valid_labels.size > 0:
                # 取邻域内出现次数最多的有效房间标签
                label = np.bincount(valid_labels.flatten()).argmax()
            else:
                label = wall_label # 依然找不到则标记为墙

        # 4. 组织数据
        if label > 1 and label != wall_label:
            room_key = f"room_{int(label)}"
        else:
            room_key = "unassigned_or_wall"

        if room_key not in room_data:
            room_data[room_key] = []

        room_data[room_key].append({
            "instance_id": int(instance_ids[i]),
            "category": str(categories[i]),
            "position_world": [round(float(x), 3) for x in box_tensors[i, :3].tolist()]
        })

    # 5. 写入 YAML
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, 'w', encoding='utf-8') as f:
        yaml.dump(room_data, f, allow_unicode=True, sort_keys=False)
    
    print(f"成功保存房间-物体映射至: {output_path}")

def extract_architecture_points(xyz, valid, wall_mask, ceil_mask, floor_mask, door_mask, step=4):
    """
    xyz: [H, W, 3] 全量点云
    """
    # 1. 墙壁提取 (利用语义优势，放宽高度以获取高密度)
    wall_mask_torch = torch.from_numpy(wall_mask).to(xyz.device)
    # 只要是墙，且在合理高度范围内 (1.0 - 3.2m)，都算墙
    final_wall_mask = valid & wall_mask_torch & (xyz[..., 2] > 2.8) & (xyz[..., 2] < 3.2)
    wall_pts = xyz[final_wall_mask] 

    # 2. [新增] 门提取 (作为障碍物处理)
    # 门的高度一般是落地到顶框 (0.1m - 2.5m)
    door_mask_torch = torch.from_numpy(door_mask).to(xyz.device)
    # final_door_mask = valid & door_mask_torch & (xyz[..., 2] > 1.2) & (xyz[..., 2] < 3.5)
    final_door_mask = valid & door_mask_torch & (xyz[..., 2] > 2.8) & (xyz[..., 2] < 3.2)
    door_pts = xyz[final_door_mask]

    # 3. 天花板/地面提取 (步长=4)
    xyz_s = xyz[::step, ::step]
    valid_s = valid[::step, ::step]
    
    ceil_mask_s = torch.from_numpy(ceil_mask[::step, ::step]).to(xyz.device)
    ceil_h_mask = (xyz_s[..., 2] > 3.7) & (xyz_s[..., 2] < 5.5)
    
    final_ceil_pts = xyz_s[valid_s & (ceil_mask_s | ceil_h_mask)]

    return wall_pts, door_pts, final_ceil_pts
def run(cfg, model, dataset, clip_model, preprocess, tokenized_text, text_features, augmentor, preprocessor, score_thresh=0.0, viz_on_gt_points=False, gap=25, re_vis=True):
    is_depth_model = "wide/depth" in augmentor.measurement_keys
    blueprint = rrb.Blueprint(
        rrb.Vertical(
            contents=[
                rrb.Horizontal(
                    contents=([
                    rrb.Spatial3DView(
                        name="World",
                        contents=[
                            "+ $origin/**",
                            "+ /device/wide/pred_instances/**",
                            # "+ /world/image/**"
                        ],
                        origin="/world"),
                    ])),
                rrb.Horizontal(
                    contents=([
                        rrb.Spatial2DView(
                            name="Image",
                            origin="/device/wide/image",
                            contents=[
                                "+ $origin/**",
                                "+ /device/wide/pred_instances/**"
                            ])
                    ] + ([
                        # Only show this for RGB-D.
                        rrb.Spatial2DView(
                            name="Depth",
                            origin="/device/wide/depth")
                    ] if is_depth_model else [])),
                    name="Wide")
            ]))

    recording = None
    video_id = None

    device = model.pixel_mean

    count=0
    all_pred_box = None
    all_poses = None

    all_kf_pose = {}
    per_frame_ins = None #save every predicted boxes
    traj_xyz = []

    box_manager = BoxManager(cfg)
    Box_Fuser = BoxFusion(cfg)

    box_count = 0
    start_time = time.time()
    
    # segmentor = SceneSegmenter(device=args.device, model_name="nvidia/segformer-b0-finetuned-ade20k-512-512")
    segmentor = SceneSegmenter(device=args.device, local_path="./models/segformer-b0-finetuned-ade-512-512")
    room_segmenter = GridRoomSegmenter(resolution=0.05)
    accumulated_walls = []
    accumulated_doors = []
    
    for sample in dataset:
        sample_video_id = sample["meta"]["video_id"] #(['sensor_info', 'wide', 'gt', 'meta'])
        pose = sample['sensor_info'].gt.RT
        
        video_id = sample_video_id
        if ((recording is None) or (video_id != sample_video_id)) and re_vis:
            new_recording = rerun.new_recording(
                application_id=str(sample_video_id), recording_id=uuid.uuid4(), make_default=True)
            new_recording.send_blueprint(blueprint, make_active=True)
            rerun.spawn()
            recording = new_recording
        
        pose_np = pose.squeeze().cpu().numpy()

        if re_vis:
            rerun.set_time_seconds("pts", sample["meta"]["timestamp"], recording=recording)

        # -> channels last.
        image = np.moveaxis(sample["wide"]["image"][-1].numpy(), 0, -1)  #[H,W,3]

        if re_vis:
            color_camera = rerun.Pinhole(
                image_from_camera=sample["sensor_info"].wide.image.K[-1].numpy(), resolution=sample["sensor_info"].wide.image.size)

        if is_depth_model and re_vis:
            # Show the depth being sent to the model.            
            depth_camera = rerun.Pinhole(
                image_from_camera=sample["sensor_info"].wide.depth.K[-1].numpy(), resolution=sample["sensor_info"].wide.depth.size)

        if Box_Fuser.update_K_flag == False:
            Box_Fuser.update_intrinsics(sample["sensor_info"].wide.image.size,sample["sensor_info"].wide.image.K[-1].numpy()) #size:[W,H]

        xyzrgb = None
        if viz_on_gt_points and sample["sensor_info"].has("gt"):
            # Backproject GT depth to world so we can compare our predictions.
            depth_gt = sample["wide"]["depth"][-1]
            matched_image = torch.tensor(np.array(Image.fromarray(image).resize((depth_gt.shape[1], depth_gt.shape[0]))))
            # Feel free to change max_depth, but know CA is only trained up to 5m.
            xyz, valid = unproject(depth_gt, sample["sensor_info"].gt.depth.K[-1], pose.squeeze(), max_depth=10.0)
            xyzrgb = torch.cat((xyz, matched_image / 255.0), dim=-1)[valid]            
                    
        packaged = augmentor.package(sample)
        packaged = move_input_to_current_device(packaged, device)
        packaged = preprocessor.preprocess([packaged])

        # Every gap nth frame is selected as keyframe
        if count % gap == 0:
            with torch.no_grad():
                pred_instances = model(packaged)[0] 

            pred_instances = pred_instances[pred_instances.scores >= float(score_thresh)]
 
            if cfg["detection"]["uv_bound"]:
                uv_mask = box_manager.check_uv_bounds(pred_instances.pred_proj_xy,image.shape[1],image.shape[0],ratio=cfg["detection"]["uv_bound_value"]) #[N]
                pred_instances = pred_instances[uv_mask]
            if cfg["detection"]["floor_mask"]:
                floor_mask = box_manager.check_floor_mask(pred_instances.pred_boxes_3d.tensor, ratio=cfg["detection"]["floor_ratio"])
                pred_instances = pred_instances[~floor_mask]

           # avoid first frame empty predictions
            if len(pred_instances) == 0 and count ==0:
                with torch.no_grad():
                    pred_instances = model(packaged)[0]
                pred_instances = pred_instances[pred_instances.scores >= float(cfg['detection']['score_thresh']/4)]
                print("again",count,"pred_instances",len(pred_instances))
                if cfg["detection"]["uv_bound"]:
                    uv_mask = box_manager.check_uv_bounds(pred_instances.pred_proj_xy,image.shape[1],image.shape[0],ratio=cfg["detection"]["uv_bound_value"]) #[N]
                    pred_instances = pred_instances[uv_mask]
                print("again",count,"pred_instances",len(pred_instances))
            
            image_rgb = np.moveaxis(sample["wide"]["image"][-1].numpy(), 0, -1)
            depth_map = sample["wide"]["depth"][-1]
            K = sample["sensor_info"].wide.depth.K[-1]
            pose = sample['sensor_info'].gt.RT.squeeze()

            # 1. 2D 语义分割获取 3 种 Mask (确保顺序一致)
            # 假设你的 segmentor.get_masks 现在返回 wall, floor, ceil, door
            wall_mask_raw, floor_mask_raw, ceil_mask_raw, door_mask_raw = segmentor.get_masks(image_rgb)

            # 2. 反投影
            xyz, valid = unproject(depth_map, K, pose, max_depth=8.0)
            target_h, target_w = valid.shape

            # 3. 强制对齐 Mask (保持 bool 类型)
            wall_mask = cv2.resize(wall_mask_raw.astype(np.uint8), (target_w, target_h), interpolation=cv2.INTER_NEAREST).astype(bool)
            floor_mask = cv2.resize(floor_mask_raw.astype(np.uint8), (target_w, target_h), interpolation=cv2.INTER_NEAREST).astype(bool)
            ceil_mask = cv2.resize(ceil_mask_raw.astype(np.uint8), (target_w, target_h), interpolation=cv2.INTER_NEAREST).astype(bool)
            door_mask = cv2.resize(door_mask_raw.astype(np.uint8), (target_w, target_h), interpolation=cv2.INTER_NEAREST).astype(bool)

            # 4. 提取点云 (传入 door_mask)
            wall_pts_torch, door_pts_torch, ceil_pts_torch = extract_architecture_points(
                xyz, valid, wall_mask, ceil_mask, floor_mask, door_mask, step=4
            )

            # 5. 更新房间分割器
            if wall_pts_torch.shape[0] > 0:
                room_segmenter.update_wall_map(wall_pts_torch.cpu().numpy())
                accumulated_walls.append(wall_pts_torch.cpu().numpy())

            if door_pts_torch.shape[0] > 0:
                room_segmenter.update_door_map(door_pts_torch.cpu().numpy()) # 门也是墙
                accumulated_doors.append(door_pts_torch.cpu().numpy())

            if ceil_pts_torch.shape[0] > 0:
                room_segmenter.update_ceil_map(ceil_pts_torch.cpu().numpy())

            # 提取地面点
            floor_valid = valid & torch.from_numpy(floor_mask).to(valid.device)
            floor_pts_all = xyz[floor_valid].cpu().numpy()
            if len(floor_pts_all) > 0:
                # 【修复 2】高度阈值改为 0.2m，只取真正的地面
                room_segmenter.update_floor_map(floor_pts_all[floor_pts_all[:, 2] < 0.2])

            # 6. 执行分割
            if count % 100 == 0:
                markers = room_segmenter.perform_segmentation(debug_path="./debug_room/")
                if markers is not None:
                    # 1. 保存房间-物体映射 (现有逻辑)
                    if all_pred_box is not None:
                        yaml_name = f"./debug_room/room_objects_{count}.yaml"
                        save_room_mapping_to_yaml(all_pred_box, room_segmenter, output_path=yaml_name)
                    
                    # 2. [新增] 保存轻量化 Vector Map
                    vector_map = room_segmenter.get_vector_map_data()
                    
                    # 如果有物体，也可以把物体以 Box 的形式加进去
                    if all_pred_box is not None:
                        # 将物体 3D box 简化为 2D 带方向的矩形
                        obj_vectors = []
                        box_tensors = all_pred_box.pred_boxes_3d.tensor.cpu().numpy()
                        cats = all_pred_box.categories
                        ids = all_pred_box.init_id.cpu().numpy()
                        
                        for i in range(len(box_tensors)):
                            # box_tensors[i] = x, y, z, w, h, l, yaw
                            # 这里做个简单的转换，实际可以用 box_corners 的投影
                            # obj_vectors.append({
                            #     "id": int(ids[i]),
                            #     "category": str(cats[i]),
                            #     "pose": [float(x) for x in box_tensors[i, :2]], # x, y
                            #     "size": [float(x) for x in box_tensors[i, 3:6]], # w, h, l
                            #     "yaw": float(box_tensors[i, 6])
                            # })
                            obj_vectors.append({
                                "id": int(ids[i]),
                                "category": str(cats[i]),
                                "pose": [float(x) for x in box_tensors[i, :2]], # x, y
                                "size": [float(x) for x in box_tensors[i, 3:6]] # w, h, l
                            })
                        vector_map["objects"] = obj_vectors

                    # 保存为 JSON
                    with open(f"./debug_room/vector_map_{count}.json", 'w') as f:
                        json.dump(vector_map, f, indent=2)
                        
                    print(f"Vector Map 已生成: ./debug_room/vector_map_{count}.json")
            
            # # 将 Mask 转换为 2D 可视化
            # seg_vis = np.zeros((*wall_mask.shape, 3), dtype=np.uint8)
            # seg_vis[wall_mask] = [255, 0, 0]   # 墙：红色
            # seg_vis[floor_mask] = [0, 255, 0]  # 地：绿色
            
            # # 推送到 Rerun
            # if re_vis:
            #     rerun.log("/device/wide/semantic_mask", rerun.Image(seg_vis).compress())

        # Hold off on logging anything until now, since the delay might confuse the user in the visualizer.
        RT = sample["sensor_info"].gt.RT[-1].numpy()
        if re_vis:
            pose_transform = rerun.Transform3D(
                translation=RT[:3, 3],
                rotation=rerun.Quaternion(xyzw=Rotation.from_matrix(RT[:3, :3]).as_quat()))
            rerun.log("/world/image", pose_transform)
            rerun.log("/world/image", color_camera)

            rerun.log("/device/wide/image", pose_transform)
            rerun.log("/device/wide/image", rerun.Image(image).compress())
            rerun.log("/device/wide/image", color_camera)
        traj_xyz.append(RT[:3, 3])
            

        if is_depth_model and re_vis:
            rerun.log("/device/wide/depth", rerun.DepthImage(sample["wide"]["depth"][-1].numpy()))
            rerun.log("/device/wide/depth", depth_camera)
        
        if xyzrgb is not None and re_vis:
            rerun.log("/world/xyz", rerun.Points3D(positions=xyzrgb[..., :3], colors=xyzrgb[..., 3:], radii=None))        

        # visualize the trajectory
        if cfg["vis"]["trajectory"] and re_vis:
            rerun.log("/world/trajectory", rerun.LineStrips3D([np.array(traj_xyz)[:count]], colors=[84,255,159]))

        # only process keyframes
        if count % gap ==0 or count == len(dataset)-1:
            
            all_kf_pose[count] = pose_np
            pose_np = np.expand_dims(pose_np,axis=0)
            pose_np = np.repeat(pose_np, repeats=len(pred_instances), axis=0) 
            
            if len(pred_instances)==0:
                all_pred_box = all_pred_box
                all_poses = all_poses
                box_count += len(pred_instances)
                box_manager.num_record[count] = box_count
                count+=1
                continue
            
            # add new properties for Instance3D predictions
            pred_instances.categories = np.array(['None'] * len(pred_instances)) # Initialize category labels as 'None' for all predicted instances
            pred_instances.cam_pose = torch.from_numpy(pose_np) # Convert camera pose from numpy to tensor and assign to instances
            pred_instances.frame_id = torch.tensor([count]).repeat(pose_np.shape[0]) # Assign current frame ID to all instances in this frame
            pred_instances.init_id = box_count+torch.arange(len(pred_instances)) # Create unique initial IDs for each instance based on global box count
            pred_instances.valid_num = torch.zeros(len(pred_instances)) # Initialize validation counter to zero for all instances
            pred_instances.pred_boxes_3d.transform2world(pred_instances.cam_pose) # Transform 3D bounding boxes from camera coordinates to world coordinates
            pred_instances.project_3d_boxes(sample["sensor_info"].wide.depth.K[-1].numpy(), H=image.shape[0],W=image.shape[1]) # Project 3D boxes to 2D image coordinates using camera intrinsics

            # record how many boxes each keyframe has, so we know which box belongs to which frame
            box_count += len(pred_instances)
            box_manager.num_record[count] = box_count
 
            # first keyframe, initialize some data structures
            if all_pred_box is None and count<gap:
                
                #predict the semantic classes
                boxes = pred_instances.pred_boxes.cpu().numpy()
                #scale the boxes by
                boxes = scale_boxes(boxes,image.shape[0],image.shape[1],scale=1.5)

                class_results, box_features = text_prompt(boxes, tokenized_text, text_features, image, clip_model, preprocess) #[N_box]
                pred_instances.categories = class_results

                all_pred_box = pred_instances
                all_poses = pose_np
                per_frame_ins = pred_instances
 
                #record the current frame boxes info
                box_manager.init_new_predictions(len(pred_instances),0)

            else:
                
                box_manager.init_new_predictions(len(pred_instances),len(per_frame_ins))

                num_before_cat = len(all_pred_box)
                cur_global_pred_box = all_pred_box

                all_pred_box = Instances3D.cat([all_pred_box,pred_instances])
                per_frame_ins = Instances3D.cat([per_frame_ins,pred_instances])

                all_poses = np.concatenate((all_poses, pose_np), axis=0)  

                print("\ncur frame id:",count)
                '''
                STEP1: spatial association using 3D OBB NMS
                '''
                mask, success_mask = Instances3D.spatial_association(all_pred_box,cfg["box_fusion"]["nms_threshold"],box_manager,per_frame_ins.cam_pose)
                
                cur_keep_idx = [i-num_before_cat for i in mask if i>=num_before_cat]
                cur_success_nms = [i-num_before_cat for i in success_mask if i>=num_before_cat]
                
 
                keep_idx = np.asarray(mask)
                if len(cur_keep_idx)>0:
                    '''
                    STEP2: correspondence association for small objects
                    '''
                    all_pred_box,all_poses,keep_idx = Instances3D.correspondence_association(
                        cfg, 
                        box_manager, 
                        cur_keep_idx, 
                        cur_success_nms,
                        pred_instances, 
                        cur_global_pred_box, 
                        all_pred_box,all_poses, 
                        per_frame_ins.cam_pose, 
                        count,
                        mask,
                        sample["sensor_info"].gt.depth.K[-1],
                        all_kf_pose,
                        threshold=cfg['association']['small_threshold'],
                        H=image.shape[0],
                        W=image.shape[1]
                        )

                    # update the fusion list based on keep_idx
                    box_manager.update(keep_idx)
                
                    print(count," box_manager",box_manager.fusion_list)

                    #filter those evident wrong boxes that valid_num=0
                    if cfg['box_fusion']['check_valid']:
                        all_pred_box = box_manager.check_valid_num(all_pred_box, count, gap)

                    '''
                    multi-view box fusion
                    '''
                    print("frame_id:box_num",box_manager.num_record)
                    if cfg['box_fusion']['use']:
                        Box_Fuser.boxfusion(all_pred_box, per_frame_ins, box_manager)
                
                    #predict the semantic classes of remaining new boxes
                    cur_keep_idx = [i-num_before_cat for i in keep_idx if i>=num_before_cat]
                    cur_keep_idx_in_all = [i for i in range(keep_idx.shape[0]) if keep_idx[i]>=num_before_cat]

                    if len(cur_keep_idx)>0:
                        boxes = pred_instances.pred_boxes.cpu().numpy()
                        boxes = boxes[cur_keep_idx]
                        # scale the boxes
                        boxes = scale_boxes(boxes,image.shape[0],image.shape[1],scale=cfg['detection']['scale_box'])
                        # if len(pred_instances)>0:
                        class_results, box_features = text_prompt(boxes, tokenized_text, text_features, image, clip_model, preprocess) #[N_box]
                        all_pred_box.categories[cur_keep_idx_in_all] = class_results

                else: # no new box
                    all_pred_box = all_pred_box[mask]
                    all_poses = all_poses[mask]
                    box_manager.update(keep_idx)
                    print(count, "new boxes have all been nms"," box_manager",box_manager.fusion_list)

            if re_vis:
                visualize_online_boxes(all_pred_box, prefix="/device/wide", boxes_3d_name="pred_boxes_3d", log_instances_name="pred_instances",count=count,save=False,show_class=cfg["vis"]["show_class"],show_label=cfg["vis"]["show_label"]) 

        count+=1
        
        # save the results
        if count == len(dataset)-1 or (count+gap)>len(dataset)-1:
            end_time = time.time()
            duration = end_time - start_time  
            fps = count / duration
            print(f"Cost: {duration:.2f} s", f"Average FPS: {fps:.2f}")
            
            # save global boxes for evaluation
            if cfg['data']['output_dir'] is not None and cfg["eval"]:
                class_list = tokenized_text.tolist()
                class_idx = np.array([class_list.index(c) for c in all_pred_box.categories]) #[N]

                boxes_3d = all_pred_box.pred_boxes_3d.corners.cpu().numpy() # [N,8,3]
                if cfg['dataset'] == 'scannet':
                    boxes_3d = post_process(boxes_3d)
                    
                if boxes_3d.shape[0]>0:
                    save_list = [[(int(0), (boxes_3d[n]), 1.0) for n in range(boxes_3d.shape[0])]] # list of tuples class_idx[n]

                    save_box(save_list, os.path.join(cfg['data']['output_dir'], video_id[0]+"_boxes.pkl"))

            print("正在保存点云文件...")
            save_path = "./exported_pc/"
            os.makedirs(save_path, exist_ok=True)
            
            if accumulated_walls:
                all_walls = np.concatenate(accumulated_walls, axis=0)
                pcd_wall = o3d.geometry.PointCloud()
                pcd_wall.points = o3d.utility.Vector3dVector(all_walls)
                pcd_wall.paint_uniform_color([1, 0, 0]) # 红色
                o3d.io.write_point_cloud(f"{save_path}/only_walls.ply", pcd_wall)
                print(f"墙壁点云已保存至: {save_path}/only_walls.ply")

            if accumulated_doors:
                all_doors = np.concatenate(accumulated_doors, axis=0)
                pcd_door = o3d.geometry.PointCloud()
                pcd_door.points = o3d.utility.Vector3dVector(all_doors)
                pcd_door.paint_uniform_color([0, 1, 0]) # 绿色
                o3d.io.write_point_cloud(f"{save_path}/only_doors.ply", pcd_door)
                print(f"门点云已保存至: {save_path}/only_doors.ply")   
            # 制作合并点云 (墙+门)
            if accumulated_walls or accumulated_doors:
                combined_pcd = pcd_wall + pcd_door
                o3d.io.write_point_cloud(f"{save_path}/walls_and_doors_combined.ply", combined_pcd)
                print(f"合并点云已保存至: {save_path}/walls_and_doors_combined.ply") 

            exit(0)
            break

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("dataset_path", help="Path to the directory containing the .tar files, the full path to a single tar file (recommended), or a path to a txt file containing HTTP links. Using the value \"stream\" will attempt to stream from your device using the NeRFCapture app")
    parser.add_argument("--model-path", help="Path to the model to load")
    parser.add_argument("--config", default=None, type=str, help="config_path")
    parser.add_argument("--clip_path", default='./models/open_clip_pytorch_model.bin', type=str, help="Path to the CLIP model")
    parser.add_argument("--seq", default='None', type=str, help="config_path")
    parser.add_argument("--class_txt", default='./data/panoptic_categories_nomerge.txt', type=str, help="config_path")
    parser.add_argument("--every-nth-frame", default=None, type=int, help="Load every `n` frames")
    parser.add_argument("--viz-on-gt-points", default=True, action="store_true", help="Backproject the GT depth to form a point cloud in order to visualize the predictions")
    parser.add_argument("--device", default="cpu", help="Which device to push the model to (cpu, mps, cuda)")
    parser.add_argument("--video-ids", nargs="+", help="Subset of videos to execute on. By default, all. Ignored if a tar file is explicitly given or in stream mode.")

    args = parser.parse_args()
    print("Command Line Args:", args)

    dataset_path = args.dataset_path
    use_cache = False
    
    if dataset_path.lower() in ["scannet", "ca1m", 'online']:
        if not os.path.exists(args.config):
            raise ValueError("Missing config path")
        else:
            with open(args.config, 'r') as  f:
                cfg = yaml.full_load(f)
        # load the customized sequence if given by the user
        if args.seq is not None:
            if dataset_path.lower()=='ca1m':
                if 'example' in cfg['data']['datadir']:
                    current_file_path = os.path.abspath(__file__)
                    current_dir = os.path.dirname(current_file_path)
                    cfg['data']['datadir'] = os.path.join(current_dir, cfg['data']['datadir'])

                else:
                    new_datadir = os.path.join(os.path.dirname(os.path.dirname(cfg['data']['datadir'])),  args.seq+'/')
                    cfg['data']['datadir'] = new_datadir

                
            else:
                new_datadir = os.path.join(os.path.dirname(os.path.dirname(cfg['data']['datadir'])),  args.seq+'/frames/')
                cfg['data']['datadir'] = new_datadir
                
            # eval only
            if os.path.exists(os.path.join(cfg['data']['output_dir'],args.seq+"_boxes.pkl")) and cfg["eval"]:
                print("Results for boxes already exist, skip evaluation")
                sys.exit(0)
        
        dataset = get_dataset(cfg)

    assert args.model_path is not None
    checkpoint = torch.load(args.model_path, map_location=args.device or "cpu")["model"]
    backbone_embedding_dimension = checkpoint["backbone.0.patch_embed.proj.weight"].shape[0]
        
    is_depth_model = True 
    model = make_cubify_transformer(dimension=backbone_embedding_dimension, depth_model=is_depth_model).eval()
    model.load_state_dict(checkpoint)

    dataset.load_arkit_depth = True
    if args.every_nth_frame is not None:
        dataset = itertools.islice(dataset, 0, None, args.every_nth_frame)

    augmentor = Augmentor(("wide/image", "wide/depth"))
    preprocessor = Preprocessor()
    
    # if args.device is not None:
    #     model = model.to(args.device)
    #     clip_model, preprocess = load_clip(args.clip_path)
    #     text_class = np.genfromtxt(args.class_txt, delimiter='\n', dtype=str) 
    #     text_features = torch.load('./data/class_features.pt').cuda()
    # --- 修改后 ---
    if args.device is not None:
        model = model.to(args.device)
        
        # ------------------- 修改开始 -------------------
        print("正在加载本地 CLIP 模型...")
        # 1. 设置为你下载权重的绝对路径
        local_model_path = '/home/aurora/workspace1/BoxFusion/models/ViT-B-32/open_clip_pytorch_model.bin'
        
        # 2. 直接使用 open_clip 加载本地权重 (注意：必须与 gen_features.py 中的 model_name 一致)
        clip_model, _, preprocess = open_clip.create_model_and_transforms(
            model_name='ViT-B-32', 
            pretrained=local_model_path
        )
        clip_model = clip_model.to(args.device).eval()
        
        text_class = np.genfromtxt(args.class_txt, delimiter='\n', dtype=str) 
        
        # 3. 加载你刚刚生成的新特征文件 (class_features_small.pt)
        print("正在加载新生成的文本特征...")
        text_features = torch.load('./data/class_features_small.pt').to(args.device)
        # ------------------- 修改结束 -------------------

    run(cfg, model, dataset, clip_model, preprocess, text_class, text_features, augmentor, preprocessor, score_thresh=cfg['detection']['score_thresh'], viz_on_gt_points=args.viz_on_gt_points, gap=cfg["data"]["gap"], re_vis=cfg['vis']['rerun'])