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

from boxfusion.dynamic_room_segmenter import DynamicRoomSegmenter

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
    
    room_segmenter = DynamicRoomSegmenter(resolution=0.05)
    accumulated_all_pts = []
    
    # 在循环外初始化起点
    t_loop_start = time.time()
    
    for sample in dataset:
        # ---------------------------------------------------------
        # 阶段 1: 数据加载与预处理 (Data Loading & Preprocessing)
        # ---------------------------------------------------------
        t_data_end = time.time()
        
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

        # ---------------------------------------------------------
        # 阶段 2: 主模型推理 (Network Inference)
        # ---------------------------------------------------------
        t_infer_start = time.time()
        
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
                if cfg["detection"]["uv_bound"]:
                    uv_mask = box_manager.check_uv_bounds(pred_instances.pred_proj_xy,image.shape[1],image.shape[0],ratio=cfg["detection"]["uv_bound_value"]) #[N]
                    pred_instances = pred_instances[uv_mask]
            
            # ================= 点云反投影增量收集 =================
            image_rgb = np.moveaxis(sample["wide"]["image"][-1].numpy(), 0, -1)
            depth_map = sample["wide"]["depth"][-1]
            K = sample["sensor_info"].wide.depth.K[-1]
            pose = sample['sensor_info'].gt.RT.squeeze()

            # 反投影并将当前帧有效点云收集起来
            xyz, valid = unproject(depth_map, K, pose, max_depth=8.0)
            matched_image = torch.tensor(np.array(Image.fromarray(image_rgb).resize((depth_map.shape[1], depth_map.shape[0]))))
            xyzrgb = torch.cat((xyz, matched_image / 255.0), dim=-1)[valid]
            
            # --- 极速降采样单帧点云 ---
            xyzrgb_np = xyzrgb.cpu().numpy()
            if xyzrgb_np.shape[0] > 0:
                pcd_frame = o3d.geometry.PointCloud()
                # 【关键修复】：强制转换为 float64 和连续内存，防止 Open3D 报错或静默失败！
                pcd_frame.points = o3d.utility.Vector3dVector(np.ascontiguousarray(xyzrgb_np[:, :3], dtype=np.float64))
                pcd_frame.colors = o3d.utility.Vector3dVector(np.ascontiguousarray(xyzrgb_np[:, 3:6], dtype=np.float64))
                
                pcd_frame = pcd_frame.voxel_down_sample(voxel_size=0.05)
                
                xyzrgb_down = np.concatenate([np.asarray(pcd_frame.points), np.asarray(pcd_frame.colors)], axis=1)
                accumulated_all_pts.append(xyzrgb_down)
                
        t_infer_end = time.time()

        # ---------------------------------------------------------
        # 阶段 3: 房间拓扑分割 (Room Segmentation)
        # ---------------------------------------------------------
        t_seg_start = time.time()
        
        # 将分割触发逻辑提出来，只要是 100 的整数倍帧就会检查，不再受 gap 限制
        if count % 100 == 0:
            print(f"\n[调试信息] 当前帧: {count}, 缓存的点云片段数: {len(accumulated_all_pts)}")
            
            if len(accumulated_all_pts) > 0:
                print(f"[{count}] 正在执行动态 2D 栅格生成与房间拓扑分割...")
                
                # --- 全局二次降采样与缓存清理 ---
                all_pts_merged = np.concatenate(accumulated_all_pts, axis=0)
                
                pcd_global = o3d.geometry.PointCloud()
                # 【关键修复】：同样做类型保护
                pcd_global.points = o3d.utility.Vector3dVector(np.ascontiguousarray(all_pts_merged[:, :3], dtype=np.float64))
                pcd_global.colors = o3d.utility.Vector3dVector(np.ascontiguousarray(all_pts_merged[:, 3:6], dtype=np.float64))
                pcd_global = pcd_global.voxel_down_sample(voxel_size=0.05)
                
                downsampled_merged_pts = np.concatenate([np.asarray(pcd_global.points), np.asarray(pcd_global.colors)], axis=1)
                
                # 用精简后的全局点云覆盖列表
                accumulated_all_pts = [downsampled_merged_pts]
                
                # 提取 xyz 传给分割器
                xyz_only = np.ascontiguousarray(downsampled_merged_pts[:, :3], dtype=np.float64)
    
                markers = room_segmenter.perform_segmentation(
                    xyz_only, 
                    all_pred_box, 
                    debug_path="./debug_room/", 
                    count=count
                )

                if markers is not None:
                    # yaml_name = f"./debug_room/room_objects_{count}.yaml"
                    # room_segmenter.save_room_mapping_to_yaml(all_pred_box, output_path=yaml_name)
                    vector_map = room_segmenter.get_vector_map_data(all_pred_box)
                    with open(f"./debug_room/vector_map_{count}.json", 'w') as f:
                        json.dump(vector_map, f, indent=2)
                    print(f"[{count}] Vector Map 及拓扑数据已更新！")
            else:
                print(f"[{count}] 警告: 没有收集到有效点云，无法执行房间分割！")
                
        t_seg_end = time.time()

        # ---------------------------------------------------------
        # 阶段 4: Rerun 可视化发送 (Visualization)
        # ---------------------------------------------------------
        t_rerun_start = time.time()
        
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
            
        t_rerun_end = time.time()

        # ---------------------------------------------------------
        # 阶段 5: BoxFusion 关联与更新 (BoxFusion & Feature Extraction)
        # ---------------------------------------------------------
        t_fusion_start = time.time()
        
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
            pred_instances.embeddings = torch.zeros((len(pred_instances), 512))
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

                # --- [修改点 1：将特征挂载到实例上] ---
                # box_features 是 torch.Tensor，将其保留在 pred_instances 中
                pred_instances.embeddings = box_features.cpu() 
                # ------------------------------------

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
                        # --- [修改点 2：同步更新增量特征] ---
                        if not hasattr(all_pred_box, 'embeddings'):
                            dim = box_features.shape[-1]
                            all_pred_box.embeddings = torch.zeros((len(all_pred_box), dim))
                        all_pred_box.embeddings[cur_keep_idx_in_all] = box_features.cpu()

                else: # no new box
                    all_pred_box = all_pred_box[mask]
                    all_poses = all_poses[mask]
                    box_manager.update(keep_idx)
                    print(count, "new boxes have all been nms"," box_manager",box_manager.fusion_list)

            if re_vis:
                visualize_online_boxes(all_pred_box, prefix="/device/wide", boxes_3d_name="pred_boxes_3d", log_instances_name="pred_instances",count=count,save=False,show_class=cfg["vis"]["show_class"],show_label=cfg["vis"]["show_label"]) 
                
        t_fusion_end = time.time()
        
        # --- 打印本帧耗时统计（仅在关键帧打印） ---
        if count % gap == 0:
            print(f"\n=== 关键帧 [{count}] 耗时分析 (单位: 秒) ===")
            print(f"数据加载与预处理: {t_data_end - t_loop_start:.4f}")
            print(f"主模型与边界框推理: {t_infer_end - t_infer_start:.4f}")
            print(f"拓扑生成与房间分割: {t_seg_end - t_seg_start:.4f}")
            print(f"Rerun 可视化发送: {t_rerun_end - t_rerun_start:.4f}")
            print(f"特征提取与 BoxFusion: {t_fusion_end - t_fusion_start:.4f}")
            print("=========================================\n")

        count+=1
        
        # 为下一次循环重置起点
        t_loop_start = time.time()
        
        # ... (保留你原来的 save global boxes for evaluation 等代码直至结束)
        if count == len(dataset)-1 or (count+gap)>len(dataset)-1:
            end_time = time.time()
            duration = end_time - start_time  
            fps = count / duration
            print(f"count: {count:.2f} frames")
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
            # --- 新增开始：将收集到的点云列表合并并保存为 PLY ---
            if len(accumulated_all_pts) > 0:
                all_pts_merged = np.concatenate(accumulated_all_pts, axis=0)
                
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(all_pts_merged[:, :3])
                
                # 如果包含了颜色信息 (xyzrgb 维度为 6)
                if all_pts_merged.shape[1] == 6:
                    pcd.colors = o3d.utility.Vector3dVector(all_pts_merged[:, 3:6])
                
                # 进行体素降采样以减小文件体积，0.02 表示 2cm 的体素大小
                pcd = pcd.voxel_down_sample(voxel_size=0.02)
                
                # 兼容 video_id 是列表或字符串的情况
                vid_str = video_id[0] if isinstance(video_id, list) else video_id
                pc_save_name = os.path.join(save_path, f"{vid_str}_global_map.ply")
                
                o3d.io.write_point_cloud(pc_save_name, pcd)
                print(f"全局点云已成功保存至: {pc_save_name}，共 {len(pcd.points)} 个点。")
            # --- 新增结束 ---
            
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
    
    if dataset_path.lower() in ["scannet", "ca1m", 'online', 'hm3d']:
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
            
            # 修改 2：专门为 hm3d 增加路径拼接逻辑
            elif dataset_path.lower() == 'hm3d':
                # 因为你的 HM3D 结构是 .../val/00824-Dd4bFSTQ8gi，没有 frames 子目录
                # 所以我们只需拿到上级目录 (.../val)，然后拼上 seq 名称
                new_datadir = os.path.join(os.path.dirname(cfg['data']['datadir']), args.seq)
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