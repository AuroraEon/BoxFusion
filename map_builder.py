import json
import cv2
import numpy as np
import networkx as nx
import os

class NavigationMapBuilder:
    def __init__(self, json_path, resolution=0.05, grid_size=2000):
        self.resolution = resolution
        self.grid_size = grid_size
        self.half_size = grid_size // 2
        
        # 加载数据
        with open(json_path, 'r') as f:
            self.data = json.load(f)
            
        self.occupancy_grid = None
        self.topo_graph = nx.Graph()
        self.waypoints_data = [] # 初始化为空列表，防止未生成时报错

    def _world_to_grid(self, x, y):
        u = int(x / self.resolution + self.half_size)
        v = int(y / self.resolution + self.half_size)
        return u, v

    def _grid_to_world(self, u, v):
        x = (u - self.half_size) * self.resolution
        y = (v - self.half_size) * self.resolution
        return x, y

    def build_occupancy_grid(self, save_path=None):
        # 初始化：255=Free (白色), 0=Occupied (黑色)
        self.occupancy_grid = np.full((self.grid_size, self.grid_size), 255, dtype=np.uint8)

        # 绘制墙壁
        if 'rooms' in self.data:
            for room in self.data['rooms']:
                poly_pts = np.array([self._world_to_grid(*pt) for pt in room['polygon']])
                cv2.polylines(self.occupancy_grid, [poly_pts], True, 0, thickness=2)

        # 绘制物体
        if 'objects' in self.data:
            for obj in self.data['objects']:
                cx, cy = obj['pose']
                w, l, _ = obj['size']
                yaw = obj.get('yaw', 0)
                
                rect = ((cx / self.resolution + self.half_size, 
                         cy / self.resolution + self.half_size), 
                        (w / self.resolution, l / self.resolution), 
                        np.degrees(yaw))
                box = cv2.boxPoints(rect)
                
                # [修复] 使用 np.int32
                box = np.int32(box)
                
                cv2.drawContours(self.occupancy_grid, [box], 0, 0, -1)

        if save_path:
            cv2.imwrite(save_path, self.occupancy_grid)
            print(f"[Map] 占据栅格地图已保存至: {save_path}")
            
            yaml_content = f"""image: {os.path.basename(save_path)}
resolution: {self.resolution}
origin: [{-self.grid_size/2 * self.resolution}, {-self.grid_size/2 * self.resolution}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
            yaml_path = save_path.replace('.png', '.yaml')
            with open(yaml_path, 'w') as f:
                f.write(yaml_content)
            print(f"[Map] ROS YAML 配置已保存至: {yaml_path}")

    def build_topological_graph(self):
        self.topo_graph.clear()
        
        # 1. 添加房间节点
        room_centers = {}
        for room in self.data.get('rooms', []):
            rid = room['id']
            poly = np.array(room['polygon'])
            center = np.mean(poly, axis=0)
            self.topo_graph.add_node(f"Room_{rid}", pos=center.tolist(), type='room', label=f"Room {rid}")
            room_centers[rid] = center
            
        # 2. 添加门节点
        for i, gate in enumerate(self.data.get('gateways', [])):
            gid = f"Gate_{i}"
            pos = np.array(gate['pos_world'])
            self.topo_graph.add_node(gid, pos=pos.tolist(), type='gateway', label="Door")
            
            for rid in gate['connects']:
                r_node = f"Room_{rid}"
                # 只有当房间存在时才连接 (防止孤立门报错)
                if rid in room_centers: 
                    dist = np.linalg.norm(pos - room_centers[rid])
                    self.topo_graph.add_edge(r_node, gid, weight=float(dist))

        print(f"[Graph] 拓扑图构建完成: {self.topo_graph.number_of_nodes()} 节点, {self.topo_graph.number_of_edges()} 边")

    def generate_waypoints(self):
        valid_waypoints = []
        room_centers = {r['id']: np.mean(r['polygon'], axis=0) for r in self.data.get('rooms', [])}

        for obj in self.data.get('objects', []):
            cx, cy = obj['pose']
            w, l, _ = obj['size']
            yaw = obj.get('yaw', 0)
            
            candidates = []
            offsets = [
                (np.cos(yaw) * (w/2 + 0.8), np.sin(yaw) * (w/2 + 0.8)),
                (-np.cos(yaw) * (w/2 + 0.8), -np.sin(yaw) * (w/2 + 0.8)),
                (np.cos(yaw+1.57) * (l/2 + 0.8), np.sin(yaw+1.57) * (l/2 + 0.8)),
                (np.cos(yaw-1.57) * (l/2 + 0.8), np.sin(yaw-1.57) * (l/2 + 0.8))
            ]
            
            best_wp = None
            min_dist_to_center = float('inf')
            
            closest_room_dist = float('inf')
            closest_room_center = np.array([0,0])
            for r_center in room_centers.values():
                d = np.linalg.norm(np.array([cx, cy]) - r_center)
                if d < closest_room_dist:
                    closest_room_dist = d
                    closest_room_center = r_center

            for off_x, off_y in offsets:
                wx, wy = cx + off_x, cy + off_y
                u, v = self._world_to_grid(wx, wy)
                
                if not (0 <= u < self.grid_size and 0 <= v < self.grid_size):
                    continue
                    
                if self.occupancy_grid[v, u] == 255: 
                    dist = np.linalg.norm(np.array([wx, wy]) - closest_room_center)
                    if dist < min_dist_to_center:
                        min_dist_to_center = dist
                        best_wp = (wx, wy)
            
            if best_wp:
                wp_data = {
                    "target_obj_id": obj['id'],
                    "target_obj_cat": obj['category'],
                    "waypoint_pos": [round(best_wp[0], 3), round(best_wp[1], 3)],
                    "waypoint_yaw": 0.0 
                }
                valid_waypoints.append(wp_data)
                
                # 将 Waypoint 加入拓扑图
                node_name = f"WP_{obj['id']}"
                self.topo_graph.add_node(node_name, pos=best_wp, type='waypoint', label=obj['category'])
        
        self.waypoints_data = valid_waypoints
        print(f"[Waypoints] 生成了 {len(valid_waypoints)} 个交互导航点")
        return valid_waypoints

    def save_all(self, output_dir):
        os.makedirs(output_dir, exist_ok=True)
        
        # 1. 栅格图
        self.build_occupancy_grid(os.path.join(output_dir, "map.png"))
        
        # 2. 拓扑图 (先构建!)
        self.build_topological_graph()
        # [修复] 显式指定 edges="links"
        graph_data = nx.node_link_data(self.topo_graph, edges="links")
        with open(os.path.join(output_dir, "topo_graph.json"), 'w') as f:
            json.dump(graph_data, f, indent=2)
            
        # 3. 导航点 (先生成!)
        self.generate_waypoints()
        with open(os.path.join(output_dir, "waypoints.json"), 'w') as f:
            json.dump(self.waypoints_data, f, indent=2)

if __name__ == "__main__":
    # 请确保路径正确指向你的 json 文件
    builder = NavigationMapBuilder("./debug_room/vector_map_2200.json") 
    builder.save_all("./output_maps")