import numpy as np
import cv2
import os
from scipy.ndimage import maximum_filter

class GridRoomSegmenter:
    def __init__(self, resolution=0.05, grid_size=2000):
        self.resolution = resolution
        self.grid_size = grid_size
        self.half_size = grid_size // 2
        
        # 基础数据层
        self.raw_density_grid = np.zeros((grid_size, grid_size), dtype=np.float32)  # 墙壁密度
        self.raw_door_grid = np.zeros((grid_size, grid_size), dtype=np.float32)     # [新增] 门密度
        self.raw_floor_grid = np.zeros((grid_size, grid_size), dtype=np.float32)   
        self.raw_ceil_grid = np.zeros((grid_size, grid_size), dtype=np.float32)    
        
        # 结果缓存
        self.last_room_markers = None
        self.last_gateways = [] # 存储检测到的门: [{'pos': (x,y), 'width': w, 'from': id1, 'to': id2}, ...]

    def _world_to_grid(self, points):
        u = np.round(points[:, 0] / self.resolution).astype(int) + self.half_size
        v = np.round(points[:, 1] / self.resolution).astype(int) + self.half_size
        u = np.clip(u, 0, self.grid_size - 1)
        v = np.clip(v, 0, self.grid_size - 1)
        return u, v
    
    def _grid_to_world(self, u, v):
        x = (u - self.half_size) * self.resolution
        y = (v - self.half_size) * self.resolution
        return np.array([x, y])

    def update_wall_map(self, wall_points):
        if len(wall_points) == 0: return
        u, v = self._world_to_grid(wall_points)
        np.add.at(self.raw_density_grid, (v, u), 1.0)
    
    def update_door_map(self, door_points):
        # [新增] 专门记录门点，用于后续 Gateway 验证
        if len(door_points) == 0: return
        u, v = self._world_to_grid(door_points)
        np.add.at(self.raw_door_grid, (v, u), 1.0)
        # 注意：门也必须视为物理阻挡供分水岭使用，所以也要加到 density_grid
        np.add.at(self.raw_density_grid, (v, u), 1.0) 

    def update_floor_map(self, floor_points):
        if len(floor_points) == 0: return
        u, v = self._world_to_grid(floor_points)
        np.add.at(self.raw_floor_grid, (v, u), 1.0)

    def update_ceil_map(self, ceil_points):
        if len(ceil_points) == 0: return
        u, v = self._world_to_grid(ceil_points)
        np.add.at(self.raw_ceil_grid, (v, u), 1.0)

    def perform_segmentation(self, debug_path=None):
        if np.all(self.raw_density_grid == 0): return None

        # --- 1. 预处理 ---
        # 截断过高的密度值，防止溢出或权重失衡
        density_clipped = np.clip(self.raw_density_grid, 0, 50)
        _, walls_mask = cv2.threshold(density_clipped, 1.5, 255, cv2.THRESH_BINARY)
        walls_skeleton = walls_mask.astype(np.uint8)
        
        # 物理阻挡层 (包含墙和门)
        kernel_thick = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)) # 稍微软一点，不要太厚
        walls_thick = cv2.dilate(walls_skeleton, kernel_thick)

        # --- 2. 外部边界处理 ---
        combined_footprint = cv2.bitwise_or(
            (self.raw_floor_grid > 0.5).astype(np.uint8) * 255,
            (self.raw_ceil_grid > 0.5).astype(np.uint8) * 255
        )
        
        outside_boundary = np.zeros_like(walls_skeleton)
        if np.any(combined_footprint > 0):
            # 闭运算填充内部孔洞，让房间更完整
            kernel_footprint = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15))
            closed_footprint = cv2.morphologyEx(combined_footprint, cv2.MORPH_CLOSE, kernel_footprint)
            contours, _ = cv2.findContours(closed_footprint, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(outside_boundary, contours, -1, 255, thickness=-1) 
        else:
            return None

        # --- 3. 构建分水岭地形 ---
        # 障碍物 = 墙壁 OR (非地板且非天花板的外部区域)
        full_map = cv2.bitwise_or(walls_thick, cv2.bitwise_not(outside_boundary))

        # --- 4. 距离变换与种子生成 ---
        dist_input = cv2.bitwise_not(full_map)
        dist_map = cv2.distanceTransform(dist_input, cv2.DIST_L2, 5)
        dist_smooth = cv2.GaussianBlur(dist_map, (7, 7), 0) # 稍微减小模糊核
        
        _, seed_blobs = cv2.threshold(dist_smooth, 1.0 / self.resolution, 255, cv2.THRESH_BINARY)
        seed_blobs = seed_blobs.astype(np.uint8)
        
        # 腐蚀种子，确保种子完全在房间中心，分开紧挨着的房间
        seed_blobs = cv2.erode(seed_blobs, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))

        contours, _ = cv2.findContours(seed_blobs, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        min_area = (0.5 / self.resolution) ** 2 # 最小房间面积降低一点
        
        markers = np.zeros(dist_map.shape, dtype=np.int32)
        room_count = 0
        
        for cnt in contours:
            if cv2.contourArea(cnt) > min_area:
                room_count += 1
                cv2.drawContours(markers, [cnt], -1, room_count + 1, -1) # Room ID 从 2 开始

        if room_count == 0: return None

        # --- 5. 分水岭算法 ---
        wall_label = room_count + 2
        markers[full_map > 0] = wall_label # 标记已知障碍物区域
        
        # 未知区域
        unknown_mask = (dist_input == 255) & (markers == 0)
        markers[unknown_mask] = 0 
        
        full_map_bgr = cv2.cvtColor(full_map, cv2.COLOR_GRAY2BGR)
        markers = cv2.watershed(full_map_bgr, markers.astype(np.int32))
        
        self.last_room_markers = markers

        # --- 6. 核心步骤：Gateway 检测 ---
        self._extract_gateways(markers, wall_label)

        if debug_path:
            self._save_debug(debug_path, walls_skeleton, walls_thick, full_map, dist_map, markers, outside_boundary)
            
        return markers

    def _extract_gateways(self, markers, wall_label):
        """
        [新增逻辑] 检测不同房间之间的连接通道
        """
        self.last_gateways = []
        unique_labels = np.unique(markers)
        unique_labels = unique_labels[(unique_labels > 1) & (unique_labels != wall_label)] # 只看房间ID
        
        # 使用膨胀法寻找相邻房间
        # 逻辑：将房间A膨胀一圈，看是否覆盖到了房间B。如果覆盖到了，重叠区域就是“边界”。
        
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        
        # 为了效率，我们只处理 ROI，但简单起见这里遍历处理
        # 实际部署可以优化为只检查 mask 的边缘
        
        for i in range(len(unique_labels)):
            for j in range(i + 1, len(unique_labels)):
                id1 = unique_labels[i]
                id2 = unique_labels[j]
                
                mask1 = (markers == id1).astype(np.uint8)
                mask2 = (markers == id2).astype(np.uint8)
                
                # 膨胀 mask1
                dilated_1 = cv2.dilate(mask1, kernel)
                
                # 寻找交集 (Boundary Region)
                # 分水岭的边界通常是 -1 或 wall_label。
                # 如果两个房间相邻，它们中间通常隔着一条线（分水岭线）。
                # 我们寻找 mask1 膨胀后与 mask2 的重叠，或者它们共同的边界线。
                
                # 更稳健的方法：找这两个房间的边界线上的点
                intersection = cv2.bitwise_and(dilated_1, mask2)
                
                if cv2.countNonZero(intersection) > 0:
                    # 找到了相邻关系，接下来计算中心点和宽度
                    points = cv2.findNonZero(intersection)
                    center_grid = np.mean(points, axis=0)[0] # (u, v)
                    
                    # 转换回世界坐标
                    pos_world = self._grid_to_world(center_grid[0], center_grid[1])
                    
                    # 验证：这里真的是门吗？
                    # 检查 raw_door_grid 在该位置是否有响应
                    u, v = int(center_grid[0]), int(center_grid[1])
                    is_door_visually = self.raw_door_grid[v, u] > 0
                    
                    # 或者：检查这里是否是“空”的（没有强墙壁阻挡）
                    # 因为分水岭即使在空地也会划线，所以我们需要区分“虚拟边界”和“物理门洞”
                    # 如果 density_grid 在这里很高，说明是实墙（只是碰巧相邻）；如果低，说明是通路。
                    is_passable = self.raw_density_grid[v, u] < 5.0 # 阈值可调
                    
                    # 结合两者：如果是 SegFormer 说是门，或者 密度极低
                    if is_door_visually or is_passable:
                        self.last_gateways.append({
                            "type": "door" if is_door_visually else "open_passage",
                            "pos_world": pos_world.tolist(),
                            "connects": [int(id1), int(id2)],
                            "grid_pos": [int(center_grid[0]), int(center_grid[1])]
                        })

    def get_vector_map_data(self):
        """
        [新增逻辑] 生成轻量化 BEV 向量数据
        返回: dict 包含 房间多边形, 门线段, 墙线段
        """
        if self.last_room_markers is None: return {}
        
        vector_data = {
            "rooms": [],
            "gateways": [],
            "walls": [] # 可选，如果需要画墙的轮廓
        }
        
        # 1. 提取房间多边形
        unique_labels = np.unique(self.last_room_markers)
        wall_label = np.max(unique_labels)
        
        for label in unique_labels:
            if label <= 1 or label == wall_label: continue
            
            mask = (self.last_room_markers == label).astype(np.uint8) * 255
            # 简化轮廓
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                # 轮廓简化 (Ramer-Douglas-Peucker)
                epsilon = 0.02 * cv2.arcLength(cnt, True) # 精度控制，越大约简略
                approx = cv2.approxPolyDP(cnt, epsilon, True)
                
                # 转换为世界坐标
                world_pts = []
                for pt in approx:
                    u, v = pt[0]
                    world_xy = self._grid_to_world(u, v)
                    world_pts.append(world_xy.tolist())
                
                vector_data["rooms"].append({
                    "id": int(label),
                    "polygon": world_pts
                })

        # 2. 提取门 (直接转换已计算的 gateways)
        for gate in self.last_gateways:
            vector_data["gateways"].append(gate)

        return vector_data
    
    def _save_debug(self, path, walls_raw, walls_thick, full_map, dist, markers, boundary):
        os.makedirs(path, exist_ok=True)
        
        # 基础图保存
        cv2.imwrite(f"{path}/01_walls_raw.png", walls_raw)
        cv2.imwrite(f"{path}/01_walls_thick.png", walls_thick)
        cv2.imwrite(f"{path}/01_boundary_solid.png", boundary)
        
        # --- 证据 1: 算法真正看到的阻挡层 ---
        # 任何白色的地方，水都流不过去
        cv2.imwrite(f"{path}/04a_barrier_mask.png", full_map)

        dist_vis = cv2.normalize(dist, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        cv2.imwrite(f"{path}/02_distance_map.png", cv2.applyColorMap(dist_vis, cv2.COLORMAP_JET))
        # cv2.imwrite(f"{path}/02b_seeds.png", seeds)

        # 准备着色
        h, w = markers.shape
        color_vis = np.zeros((h, w, 3), dtype=np.uint8)
        unique_labels = np.unique(markers)
        np.random.seed(42)
        colors = np.random.randint(70, 255, size=(int(np.max(unique_labels)) + 10, 3), dtype=np.uint8)

        # 基础房间填色 (不含任何墙壁线条)
        for label in unique_labels:
            mask = (markers == label)
            if label == -1: pass # 忽略分水岭边界线
            # elif label == wall_label: pass # 忽略墙壁/外部 (保持黑色)
            elif label > 1: color_vis[mask] = colors[label]

        # --- 证据 2: 纯房间形状 ---
        # 这里的黑色缝隙就是墙壁的厚度
        cv2.imwrite(f"{path}/04b_room_only_no_walls.png", color_vis)

        # --- 证据 3: 缝隙分析 (关键) ---
        # 叠加原始细墙 (白色)。
        # 如果算法正确，你应该在“彩色房间”和“白色细线”之间看到一圈“黑色空隙”。
        # 那个黑色空隙就是 (walls_thick - walls_raw) 的部分。
        gap_analysis = color_vis.copy()
        gap_analysis[walls_raw > 0] = [255, 255, 255]
        cv2.imwrite(f"{path}/04c_gap_proof.png", gap_analysis)

        # 最终图 (为了美观，通常我们希望填满那个缝隙，所以这里做一个视觉处理)
        # 如果你觉得之前的图“重合”，是因为我为了好看把缝隙填上了。
        # 这里生成一张“未填充缝隙”的真实结果图
        cv2.imwrite(f"{path}/03_final_rooms_raw_output.png", color_vis)
        
        # 再生成一张“美化版” (填补缝隙，让人看着舒服)
        # 用房间颜色对周围黑色区域进行轻微膨胀
        kernel_fill = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        filled_vis = cv2.dilate(color_vis, kernel_fill)
        # 再把原始白线画上去
        filled_vis[walls_raw > 0] = [255, 255, 255]
        cv2.imwrite(f"{path}/03_final_rooms_pretty.png", filled_vis)
        # (保持你原有的 debug 代码，或者加上画 Gateways 的逻辑)
        # 这里简单画一下 Gateway
        vis = np.zeros((*markers.shape, 3), dtype=np.uint8)
        
        # 画房间
        unique_labels = np.unique(markers)
        np.random.seed(42)
        colors = np.random.randint(50, 200, size=(np.max(unique_labels) + 10, 3), dtype=np.uint8)
        for label in unique_labels:
            if label > 1: vis[markers == label] = colors[label]
            
        # 画门
        for gate in self.last_gateways:
            u, v = gate['grid_pos']
            color = (0, 0, 255) if gate['type'] == 'door' else (0, 255, 255)
            cv2.circle(vis, (u, v), 5, color, -1)
            cv2.putText(vis, "GW", (u+5, v), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
            
        cv2.imwrite(f"{path}/06_vector_debug.png", vis)