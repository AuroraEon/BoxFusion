import numpy as np
import cv2
import os
import yaml
from boxfusion.scene_graph_builder import SemanticSceneGraph, RoomNode, ObjectNode

class DynamicRoomSegmenter:
    """
    结合 HOV-SG 密度直方图与分水岭算法的动态房间分割器。
    """
    def __init__(self, resolution=0.05):
        self.resolution = resolution
        
        # --- [修改点 1: 锁死全局原点] ---
        # 设定一个足够大的物理安全边界（假设室内场景不会超出当前 SLAM 坐标系系原点的 -50米）
        # 这个 origin_x 和 origin_y 在整个系统运行期间绝对不允许改变！
        self.origin_x = -50.0
        self.origin_y = -50.0
        
        # 宽高可以随着点云的输入动态向正方向扩展
        self.grid_width = 0
        self.grid_height = 0
        # -----------------------------

        self.last_room_markers = None
        self.last_gateways = []
        
        self.height_estimated = False
        self.slice_z_min = 2.8 
        self.slice_z_max = 3.7 
        self.full_z_max = 3.8
        # --- [新增：Room Tracking 状态] ---
        # 记录所有被确认的房间。格式: {global_id: {'mask': 2D_array, 'color': (B,G,R)}}
        self.tracked_rooms = {} 
        self.next_global_id = 1  # 递增的全局唯一 ID (等价于 UUID)
        self.tracking_iou_threshold = 0.3 # 认定为同一个房间的最小重叠率
        # --------------------------------

    def _world_to_grid(self, points):
        pts_2d = np.atleast_2d(points)[:, :2]
        
        u = np.floor((pts_2d[:, 0] - self.origin_x) / self.resolution).astype(int)
        v = np.floor((pts_2d[:, 1] - self.origin_y) / self.resolution).astype(int)
        
        # 严禁 clip！生成合法性掩码 (valid mask)
        valid_mask = (u >= 0) & (u < self.grid_width) & (v >= 0) & (v < self.grid_height)
        
        # 对于越界的点，不强制拉回，而是给一个无效值 -1（下游需要配合过滤）
        u[~valid_mask] = -1
        v[~valid_mask] = -1
        
        return u, v, valid_mask

    def _grid_to_world(self, u, v):
        # 加上 0.5 确保反投时落在网格中心点，这是一个良好的工程契约
        x = (u + 0.5) * self.resolution + self.origin_x
        y = (v + 0.5) * self.resolution + self.origin_y
        return np.array([x, y])

    def perform_segmentation(self, all_pts_merged, all_pred_box=None, debug_path=None, count=0):
        all_pts_np = np.asarray(all_pts_merged)
        if len(all_pts_np) == 0: return None

        # 1. 动态高度切片
        if not self.height_estimated and len(all_pts_np) > 1000:
            z_values = all_pts_np[:, 2]
            floor_z = np.percentile(z_values, 2)
            ceiling_z = np.percentile(z_values, 98)
            self.slice_z_min = floor_z + 1.5   # 借鉴HOV-SG: 地板上1.5m
            self.slice_z_max = ceiling_z - 0.3 # 借鉴HOV-SG: 天花板下0.3m
            
            # HOV-SG 提取外部轮廓用的全尺寸切片高度 (去除天花板下0.2m)
            self.full_z_max = ceiling_z - 0.2
            self.height_estimated = True
            print(f"\n[RoomSegmenter] 动态高度 -> 地板: {floor_z:.2f}m | 天花板: {ceiling_z:.2f}m")

        max_x = np.max(all_pts_np[:, 0])
        max_y = np.max(all_pts_np[:, 1])
        
        # 计算当前点云需要的最大网格尺寸，并增加 20 个 pixel 的 padding 缓冲
        needed_width = int(np.ceil((max_x - self.origin_x) / self.resolution)) + 20
        needed_height = int(np.ceil((max_y - self.origin_y) / self.resolution)) + 20
        
        # 只增不减，确保画布稳定
        self.grid_width = max(self.grid_width, needed_width)
        self.grid_height = max(self.grid_height, needed_height)

        # 2. 准备 HOV-SG 所需的两组点云切片
        z_mask_walls = (all_pts_np[:, 2] >= self.slice_z_min) & (all_pts_np[:, 2] <= self.slice_z_max)
        z_mask_full = (all_pts_np[:, 2] < self.full_z_max)
        
        pts_walls = all_pts_np[z_mask_walls][:, [0, 1]]
        pts_full = all_pts_np[z_mask_full][:, [0, 1]]

        # 计算网格 Bin 数量时，直接使用当前的 grid_width 和 grid_height
        num_bins = (self.grid_width, self.grid_height)
        hist_range = [[self.origin_x, self.origin_x + self.grid_width * self.resolution], 
                      [self.origin_y, self.origin_y + self.grid_height * self.resolution]]

        # ---------------------------------------------------------
        # 步骤 A: 提取强化的墙壁骨架 (Walls Skeleton) 
        # ---------------------------------------------------------
        if len(pts_walls) == 0: return None
        
        hist, _, _ = np.histogram2d(pts_walls[:, 0], pts_walls[:, 1], bins=num_bins, range=hist_range)
        hist = hist.T  # <--- 【千万别漏】：必须转置，把 (W, H) 变成图像需要的 (H, W)！
        
        # 【核心修复 1】：过滤点云密度异常值！截断前 2% 的极高密度点
        hist_nonzero = hist[hist > 0]
        if len(hist_nonzero) > 0:
            p98 = np.percentile(hist_nonzero, 98)
            hist = np.clip(hist, 0, p98) 

        hist = cv2.normalize(hist, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        hist = cv2.GaussianBlur(hist, (5, 5), 1)
        
        hist_threshold = 0.15 * np.max(hist)
        _, walls_skeleton = cv2.threshold(hist, hist_threshold, 255, cv2.THRESH_BINARY)

        walls_skeleton = cv2.copyMakeBorder(walls_skeleton, 10, 10, 10, 10, cv2.BORDER_CONSTANT, value=0)
        kernel_cross = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
        walls_skeleton = cv2.morphologyEx(walls_skeleton, cv2.MORPH_CLOSE, kernel_cross, iterations=1)

        # ---------------------------------------------------------
        # 步骤 B: 提取外部物理轮廓 (Outside Boundary)
        # ---------------------------------------------------------
        if len(pts_full) == 0: return None
        
        # <--- 【极度关键】：步骤 B 必须使用完全一致的 range，保证画布绝对重合
        hist_full, _, _ = np.histogram2d(pts_full[:, 0], pts_full[:, 1], bins=num_bins, range=hist_range)
        hist_full = hist_full.T  # <--- 这里也要转置！
        hist_full = cv2.normalize(hist_full, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        hist_full = cv2.GaussianBlur(hist_full, (21, 21), 2)
        _, outside_boundary = cv2.threshold(hist_full, 0, 255, cv2.THRESH_BINARY)

        outside_boundary = cv2.copyMakeBorder(outside_boundary, 10, 10, 10, 10, cv2.BORDER_CONSTANT, value=0)
        kernel_rect5 = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        outside_boundary = cv2.morphologyEx(outside_boundary, cv2.MORPH_CLOSE, kernel_rect5, iterations=3)

        contours, _ = cv2.findContours(outside_boundary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        outside_boundary = np.zeros_like(outside_boundary)
        cv2.drawContours(outside_boundary, contours, -1, (255, 255, 255), -1)

        # ---------------------------------------------------------
        # 步骤 C: 合并生成 full_map 并处理语义挖空 (门)
        # ---------------------------------------------------------
        full_map_padded = cv2.bitwise_or(walls_skeleton, cv2.bitwise_not(outside_boundary))
        kernel_rect3 = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        full_map_padded = cv2.morphologyEx(full_map_padded, cv2.MORPH_CLOSE, kernel_rect3, iterations=2)
        
        # 去除 Padding 恢复原图尺寸
        h_p, w_p = full_map_padded.shape
        full_map = full_map_padded[10:h_p-10, 10:w_p-10]

        if all_pred_box is not None:
            corners_3d = all_pred_box.pred_boxes_3d.corners.cpu().numpy()
            categories = all_pred_box.categories
            for i in range(len(corners_3d)):
                if 'door' in str(categories[i]).strip().lower():
                    pts_2d = corners_3d[i, :, :2]
                    
                    # --- [新增：接收并使用 valid_mask 过滤越界坐标，防止后续崩溃] ---
                    box_u, box_v, valid_mask = self._world_to_grid(pts_2d)
                    box_u = box_u[valid_mask]
                    box_v = box_v[valid_mask]
                    
                    # 只有合法点 >= 3 个才能构成闭合多边形
                    if len(box_u) >= 3: 
                        hull = cv2.convexHull(np.column_stack((box_u, box_v)))
                        cv2.fillPoly(full_map, [hull], 0)

        # ---------------------------------------------------------
        # 步骤 D: 距离变换与分水岭 
        # ---------------------------------------------------------
        free_space = cv2.bitwise_not(full_map)
        
        # 获取最原始的 float32 距离图
        dist = cv2.distanceTransform(free_space, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
        
        # 【新增】：算一下 dist_norm 专门留给后面的 _save_debug 画彩色热力图用
        dist_norm = cv2.normalize(dist, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        cv2.normalize(dist, dist_norm, 0, 255, cv2.NORM_MINMAX)
        
        max_dist = np.max(dist)
        print(f"[RoomSegmenter] 距离变换最大值: {max_dist:.2f} 像素 (约 {max_dist*self.resolution:.2f} 米)")
        
        # 【核心修复 2】：摒弃 Otsu，使用基于物理距离的自适应阈值
        # 设定：离墙壁至少 0.4 米的区域才算种子点。如果房间实在太窄，则退化为取最大距离的 60%
        safe_dist_pixels = 0.4 / self.resolution
        if max_dist < safe_dist_pixels:
            safe_dist_pixels = max_dist * 0.6
            
        _, dist_thresh = cv2.threshold(dist, safe_dist_pixels, 255, cv2.THRESH_BINARY)
        dist_thresh = dist_thresh.astype(np.uint8)
        
        # 【核心修复 3】：修复原代码的 (11,1) 核，使用标准形态学去除噪点毛刺
        dist_thresh = cv2.morphologyEx(dist_thresh, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        
        contours, _ = cv2.findContours(dist_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # 略微放宽过滤条件 (从 0.5 放宽到 0.25 平方米，防止狭长走廊的种子点丢失)
        min_area_m = 0.25
        min_area_pixels = (min_area_m / self.resolution) ** 2
        valid_contours = [c for c in contours if cv2.contourArea(c) > min_area_pixels]
        
        print(f"[RoomSegmenter] 提取到的原始轮廓: {len(contours)}, 有效种子点(> {min_area_m}㎡): {len(valid_contours)}")
        
        if len(valid_contours) == 0:
            print("[RoomSegmenter] 警告: 未能提取到有效种子点，分割终止。")
            return None

        markers = np.zeros(dist.shape, dtype=np.int32)
        for i in range(len(valid_contours)):
            cv2.drawContours(markers, valid_contours, i, (i + 1), -1)
            
        bg_label = len(valid_contours) + 1
        cv2.circle(markers, (3, 3), 1, bg_label, -1)

        full_map_rgb = cv2.cvtColor(full_map, cv2.COLOR_GRAY2BGR)
        cv2.watershed(full_map_rgb, markers)

        wall_label = bg_label + 1
        markers[full_map > 0] = wall_label 
        markers[markers == bg_label] = 0   
        markers[markers == -1] = wall_label

        # =========================================================
        # --- [新增：Room Tracking (IoU 掩码匹配)] ---
        # =========================================================
        current_labels = np.unique(markers)
        current_labels = current_labels[(current_labels > 0) & (current_labels != wall_label)]
        
        new_tracked_rooms = {}
        # 用于记录当前帧的 marker label 到 global_id 的映射
        self.label_to_global = {} 
        
        current_h, current_w = markers.shape

        for lbl in current_labels:
            # 提取当前帧该房间的 mask
            curr_mask = (markers == lbl).astype(np.uint8)
            
            best_iou = 0.0
            best_global_id = None
            
            # 与历史房间进行对比
            for gid, hist_data in self.tracked_rooms.items():
                hist_mask = hist_data['mask']
                hist_h, hist_w = hist_mask.shape
                
                # 因为网格可能会动态扩展，需要将历史 mask pad 到当前尺寸 (原点固定，只在右下角 pad)
                pad_h = max(0, current_h - hist_h)
                pad_w = max(0, current_w - hist_w)
                padded_hist_mask = np.pad(hist_mask, ((0, pad_h), (0, pad_w)), mode='constant', constant_values=0)
                
                # 裁剪以防万一（理论上 pad 后尺寸应该一致）
                padded_hist_mask = padded_hist_mask[:current_h, :current_w]
                
                # 计算交并比 (IoU)
                intersection = np.logical_and(curr_mask, padded_hist_mask).sum()
                union = np.logical_or(curr_mask, padded_hist_mask).sum()
                
                iou = intersection / union if union > 0 else 0
                
                if iou > best_iou:
                    best_iou = iou
                    best_global_id = gid
            
            # 判定匹配结果
            if best_iou >= self.tracking_iou_threshold and best_global_id is not None:
                # 匹配成功：继承老 ID
                assigned_id = best_global_id
                room_color = self.tracked_rooms[best_global_id]['color']
                # 从备选池中移除，防止一个老房间被多个新房间重复匹配 (简单的贪心分配)
                del self.tracked_rooms[best_global_id] 
            else:
                # 匹配失败：发现新房间，分配新 ID
                assigned_id = self.next_global_id
                self.next_global_id += 1
                # 为新房间随机分配一个固定的可视化颜色
                room_color = tuple(np.random.randint(50, 255, size=3).tolist())
            
            # 更新当前帧的追踪字典
            new_tracked_rooms[assigned_id] = {
                'mask': curr_mask,
                'color': room_color
            }
            self.label_to_global[lbl] = assigned_id

        # 更新全局追踪器
        self.tracked_rooms = new_tracked_rooms
        # =========================================================
        
        self.last_room_markers = markers
        self._extract_gateways(markers, wall_label, all_pred_box)

        if debug_path:
            self._save_debug(debug_path, count, hist, walls_skeleton, outside_boundary, dist_norm, markers, wall_label, full_map)
            
        return markers

    def _extract_gateways(self, markers, wall_label, all_pred_box):
        self.last_gateways = []
        unique_labels = np.unique(markers)
        unique_labels = unique_labels[(unique_labels > 0) & (unique_labels != wall_label)] 
        
        # --- [核心修改 1：将 19x19 (0.95米) 缩小为 5x5 (0.25米)] ---
        # 0.25 米足以跨越 Watershed 算法留下的 1 像素分界线或薄门，但绝对无法穿透标准的承重墙
        kernel_gw = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        # --------------------------------------------------------
        
        door_boxes_2d = []
        if all_pred_box is not None:
            corners_3d = all_pred_box.pred_boxes_3d.corners.cpu().numpy()
            categories = all_pred_box.categories
            for i in range(len(corners_3d)):
                if 'door' in str(categories[i]).lower():
                    pts_2d = corners_3d[i, :, :2]
                    door_boxes_2d.append(pts_2d)
        
        for i in range(len(unique_labels)):
            for j in range(i + 1, len(unique_labels)):
                id1 = unique_labels[i]
                id2 = unique_labels[j]
                
                mask1 = (markers == id1).astype(np.uint8)
                mask2 = (markers == id2).astype(np.uint8)
                
                # 轻微膨胀寻找真实相邻边界
                dilated_1 = cv2.dilate(mask1, kernel_gw)
                intersection = cv2.bitwise_and(dilated_1, mask2)
                
                if cv2.countNonZero(intersection) > 0:
                    # --- [核心修改 2：提取交集的连通块 (处理两个房间有多个门的情况)] ---
                    num_labels, labels_im, stats, centroids = cv2.connectedComponentsWithStats(intersection, connectivity=8)
                    
                    # 遍历每一个独立的相交区域 (跳过背景 0)
                    for k in range(1, num_labels):
                        # 过滤掉太小的噪点交集 (面积小于 3 个像素的偶然触碰)
                        if stats[k, cv2.CC_STAT_AREA] < 3:
                            continue
                            
                        # 提取该通道的所有像素点
                        y_coords, x_coords = np.where(labels_im == k)
                        points_grid = np.column_stack((x_coords, y_coords))
                        
                        # 1. 计算中心点
                        center_grid = centroids[k]
                        pos_world = self._grid_to_world(center_grid[0], center_grid[1])
                        
                        # 2. 估算物理宽度 (Width) 和 法向朝向 (Yaw)
                        # 利用最小外接矩形 (minAreaRect) 来拟合这个交集线段
                        if len(points_grid) >= 5:
                            rect = cv2.minAreaRect(points_grid.astype(np.float32))
                            (cx, cy), (w, h), angle = rect
                            # Gateway 通常是一条狭长地带，较长的边代表通行宽度
                            length_pixels = max(w, h)
                            width_m = length_pixels * self.resolution
                            # 法向垂直于门洞走向
                            yaw = np.deg2rad(angle) if w > h else np.deg2rad(angle + 90)
                        else:
                            # 像素太少，给个保守默认值
                            width_m = 3 * self.resolution
                            yaw = 0.0

                        # 3. 门洞类型研判 (视觉 Door Check)
                        is_door_visually = False
                        for door_pts in door_boxes_2d:
                            # 注意：这里调用了我们在 P0 修改过的带有 valid_mask 返回值的 _world_to_grid
                            door_u, door_v, valid_mask = self._world_to_grid(door_pts)
                            if not np.any(valid_mask): continue
                            
                            door_poly = np.column_stack((door_u[valid_mask], door_v[valid_mask])).astype(np.float32)
                            if len(door_poly) < 3: continue
                            
                            # 检查交集中心是否落在检测到的门框内，允许 3 像素的外扩容错率
                            if cv2.pointPolygonTest(door_poly, (float(center_grid[0]), float(center_grid[1])), True) >= -3.0:
                                is_door_visually = True
                                break
                        
                        # 4. 获取我们在 P0 建立的全局持久化 ID
                        global_id1 = self.label_to_global.get(id1, int(id1)) if hasattr(self, 'label_to_global') else int(id1)
                        global_id2 = self.label_to_global.get(id2, int(id2)) if hasattr(self, 'label_to_global') else int(id2)

                        self.last_gateways.append({
                            "type": "door" if is_door_visually else "open_passage",
                            "pos_world": [round(float(pos_world[0]), 3), round(float(pos_world[1]), 3)],
                            "connects": [int(global_id1), int(global_id2)],
                            "grid_pos": [int(center_grid[0]), int(center_grid[1])],
                            "width_m": round(float(width_m), 3),
                            "yaw": round(float(yaw), 3)
                        })

    def get_vector_map_data(self, all_pred_box=None, count=None):
        if self.last_room_markers is None: return {}
        
        vector_data = {
            "map_info": {
                "resolution": self.resolution,
                "origin_x": self.origin_x,
                "origin_y": self.origin_y,
                "grid_width": self.grid_width,
                "grid_height": self.grid_height,
                "contract": "floor_mapping_with_center_offset"
            },
            "rooms": [], 
            "gateways": [], 
            "objects": []
        }
        
        unique_labels = np.unique(self.last_room_markers)
        wall_label = np.max(unique_labels)
        
        for label in unique_labels:
            if label <= 0 or label == wall_label: continue
            if label not in self.label_to_global: continue
            global_id = self.label_to_global[label]
            
            mask = (self.last_room_markers == label).astype(np.uint8) * 255
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                epsilon = 0.02 * cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, epsilon, True)
                world_pts = []
                for pt in approx:
                    world_xy = self._grid_to_world(pt[0][0], pt[0][1])
                    world_pts.append(world_xy.tolist())
                vector_data["rooms"].append({"id": int(global_id), "polygon": world_pts})

        for gate in self.last_gateways:
            vector_data["gateways"].append(gate)
        
        if all_pred_box is not None:
            box_tensors = all_pred_box.pred_boxes_3d.tensor.cpu().numpy()
            categories = all_pred_box.categories
            instance_ids = all_pred_box.init_id.cpu().numpy()
            
            scores = all_pred_box.scores.cpu().numpy() if hasattr(all_pred_box, 'scores') else np.ones(len(box_tensors))
            has_embeddings = hasattr(all_pred_box, 'embeddings')

            for i in range(len(box_tensors)):
                cx, cy = float(box_tensors[i, 0]), float(box_tensors[i, 1])
                dx, dy, dz = float(box_tensors[i, 3]), float(box_tensors[i, 4]), float(box_tensors[i, 5])
                yaw = float(box_tensors[i, 6]) if box_tensors.shape[1] > 6 else 0.0

                u_arr, v_arr, valid_mask = self._world_to_grid(np.array([[cx, cy]]))
                u, v, is_valid = u_arr[0], v_arr[0], valid_mask[0]
                
                room_uuid = -1 
                if is_valid:
                    label = self.last_room_markers[v, u]
                    if label in self.label_to_global:
                        room_uuid = self.label_to_global[label]

                cos_y, sin_y = np.cos(yaw), np.sin(yaw)
                R = np.array([[cos_y, -sin_y], [sin_y, cos_y]])
                corners_local = np.array([
                    [ dx/2,  dy/2],
                    [-dx/2,  dy/2],
                    [-dx/2, -dy/2],
                    [ dx/2, -dy/2]
                ])
                corners_global = (R @ corners_local.T).T + np.array([cx, cy])
                footprint_2d = np.round(corners_global, 3).tolist()

                obj_data = {
                    "id": int(instance_ids[i]),
                    "category": str(categories[i]),
                    "score": round(float(scores[i]), 3),
                    "room_uuid": int(room_uuid),
                    "pose": [round(cx, 3), round(cy, 3)],
                    "size": [round(dx, 3), round(dy, 3), round(dz, 3)],
                    "yaw": round(yaw, 3),
                    "footprint_2d": footprint_2d
                }
                
                if has_embeddings:
                    obj_data["embedding_ref"] = f"embedding_{int(instance_ids[i])}"
                
                vector_data["objects"].append(obj_data)
                
            if has_embeddings:
                emb_dict = {}
                for i, inst_id in enumerate(instance_ids):
                    emb_dict[f"embedding_{int(inst_id)}"] = all_pred_box.embeddings[i].numpy().tolist()
                vector_data["embeddings"] = emb_dict

        # ==========================================================
        # --- [新增：Scene Graph 构建、关系推理与可视化] ---
        # ==========================================================
        if all_pred_box is not None and len(vector_data["objects"]) > 0:
            sg = SemanticSceneGraph()

            # 1. 注入 Room 节点
            for room_info in vector_data["rooms"]:
                r_node = RoomNode(room_id=room_info["id"], polygon_2d=room_info["polygon"])
                sg.add_room(r_node)

            # 2. 注入 Object 节点
            for obj_info in vector_data["objects"]:
                pos_3d = (obj_info["pose"][0], obj_info["pose"][1], obj_info["size"][2] / 2.0)
                bbox_3d = (obj_info["size"][0], obj_info["size"][1], obj_info["size"][2])
                
                o_node = ObjectNode(
                    obj_id=obj_info["id"],
                    pos=pos_3d,
                    bbox=bbox_3d,
                    label=obj_info["category"],
                    clip_feature=None,  
                    room_id=obj_info["room_uuid"]
                )
                sg.add_object(o_node)

            # 3. 执行空间关系推理 (阈值可根据你的实际室内尺度微调)
            sg.compute_spatial_relations(dist_threshold=1.0, z_tolerance=0.2)

            # 4. 将推理出的边导出到 JSON 数据中
            vector_data["relationships"] = []
            for source, target, data in sg.graph.edges(data=True):
                vector_data["relationships"].append({
                    "source": source,
                    "target": target,
                    "relation": data["relation"]
                })
            
            # 5. 生成并保存 2D 拓扑可视化
            vis_path = f"./debug_room/scene_graph_{count if count is not None else 'latest'}.png"
            try:
                sg.visualize_2d_graph(save_path=vis_path)
            except Exception as e:
                print(f"[警告] 场景图可视化失败: {e}")

        return vector_data

    # def save_room_mapping_to_yaml(self, all_pred_box, output_path="room_objects.yaml"):
    #     if all_pred_box is None or self.last_room_markers is None: return

    #     box_tensors = all_pred_box.pred_boxes_3d.tensor.cpu().numpy()
    #     categories = all_pred_box.categories
    #     instance_ids = all_pred_box.init_id.cpu().numpy()

    #     u_coords, v_coords = self._world_to_grid(box_tensors[:, :2])
    #     markers = self.last_room_markers
    #     wall_label = np.max(markers)

    #     room_data = {}
    #     for i in range(len(box_tensors)):
    #         u, v = u_coords[i], v_coords[i]
    #         # 增加安全检查以防坐标越界
    #         if 0 <= v < self.grid_height and 0 <= u < self.grid_width:
    #             label = markers[v, u]
    #         else:
    #             label = wall_label

    #         if label > 0 and label != wall_label:
    #             room_key = f"room_{int(label)}"
    #         else:
    #             room_key = "unassigned_or_wall"

    #         if room_key not in room_data:
    #             room_data[room_key] = []

    #         room_data[room_key].append({
    #             "instance_id": int(instance_ids[i]),
    #             "category": str(categories[i]),
    #             "position_world": [round(float(x), 3) for x in box_tensors[i, :3].tolist()]
    #         })

    #     os.makedirs(os.path.dirname(output_path), exist_ok=True)
    #     with open(output_path, 'w', encoding='utf-8') as f:
    #         yaml.dump(room_data, f, allow_unicode=True, sort_keys=False)

    def _save_debug(self, path, count, hist, walls_skeleton, outside_boundary, dist_norm, markers, wall_label, full_map):
        os.makedirs(path, exist_ok=True)
        cv2.imwrite(f"{path}/run_{count}_01_density_hist.png", hist)
        cv2.imwrite(f"{path}/run_{count}_02_walls_skeleton.png", walls_skeleton)
        cv2.imwrite(f"{path}/run_{count}_03_outside_boundary.png", outside_boundary)
        cv2.imwrite(f"{path}/run_{count}_04_full_map.png", full_map)
        cv2.imwrite(f"{path}/run_{count}_05_distance_map.png", cv2.applyColorMap(dist_norm, cv2.COLORMAP_JET))

        h, w = markers.shape
        vis = np.zeros((h, w, 3), dtype=np.uint8)
        unique_labels = np.unique(markers)
        np.random.seed(42)
        colors = np.random.randint(50, 200, size=(np.max(unique_labels) + 10, 3), dtype=np.uint8)
        
        for label in unique_labels:
            if label > 0 and label != wall_label and label in self.label_to_global:
                gid = self.label_to_global[label]
                vis[markers == label] = self.tracked_rooms[gid]['color']
        
        vis[full_map == 255] = [255, 255, 255] 

        for gate in self.last_gateways:
            u, v = gate['grid_pos']
            color = (0, 0, 255) if gate['type'] == 'door' else (0, 255, 255)
            cv2.circle(vis, (u, v), 6, color, -1)

        cv2.imwrite(f"{path}/run_{count}_06_final_rooms.png", vis)