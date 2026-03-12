import networkx as nx
import numpy as np
from typing import List, Dict, Optional, Tuple

# ==========================================
# Step 3: 数据结构与类定义 (Data Structures)
# ==========================================

class RoomNode:
    """房间节点"""
    def __init__(self, room_id: int, polygon_2d: Optional[List[Tuple[float, float]]] = None):
        self.node_id = f"room_{room_id}"
        self.room_id = room_id
        self.polygon_2d = polygon_2d or []
        self.type = "Room"

    def get_attributes(self) -> Dict:
        return {
            "type": self.type, 
            "room_id": self.room_id,
            "polygon_2d": self.polygon_2d  # <--- 就是漏了这一行！
        }


class ObjectNode:
    """物体节点"""
    def __init__(self, obj_id: int, pos: Tuple[float, float, float], 
                 bbox: Tuple[float, float, float], label: str, 
                 clip_feature: np.ndarray, room_id: int):
        self.node_id = f"obj_{obj_id}"
        self.obj_id = obj_id
        self.pos = np.array(pos)        # (x, y, z) 中心坐标
        self.bbox = np.array(bbox)      # (dx, dy, dz) 尺寸
        self.label = label              # 语义标签
        self.clip_feature = clip_feature # CLIP 向量
        self.room_id = room_id          # 所属房间 ID
        self.type = "Object"
        
        # 预计算 AABB (Axis-Aligned Bounding Box) 的极值，方便后续物理关系推理
        half_size = self.bbox / 2.0
        self.min_pt = self.pos - half_size
        self.max_pt = self.pos + half_size

    def get_attributes(self) -> Dict:
        return {
            "type": self.type,
            "label": self.label,
            "pos": self.pos.tolist(),
            "bbox": self.bbox.tolist(),
            # "clip_feature": self.clip_feature.tolist() # 为了打印清晰，实际图中可保留但打印时可忽略
        }


class SemanticSceneGraph:
    """场景图管理器"""
    def __init__(self):
        # 使用有向图，因为空间关系(ON/UNDER)和从属关系(INSIDE)是有向的
        self.graph = nx.DiGraph()
        self.objects: List[ObjectNode] = []
        self.rooms: Dict[int, RoomNode] = {}

    def add_room(self, room: RoomNode):
        self.rooms[room.room_id] = room
        self.graph.add_node(room.node_id, **room.get_attributes())

    def add_object(self, obj: ObjectNode):
        self.objects.append(obj)
        self.graph.add_node(obj.node_id, **obj.get_attributes())
        
        # Step 1: 建立 object INSIDE room 的关系
        room_node_id = f"room_{obj.room_id}"
        if room_node_id in self.graph.nodes:
            self.graph.add_edge(obj.node_id, room_node_id, relation="INSIDE")
        else:
            print(f"[Warning] Room {obj.room_id} not found for Object {obj.node_id}")

    # ==========================================
    # Step 2: 空间关系推理引擎 (Spatial Reasoning)
    # ==========================================
    def compute_spatial_relations(self, dist_threshold: float = 1.0, z_tolerance: float = 0.15):
        """
        根据物体的 3D 包围盒自动计算 next_to, on, under 等关系
        :param dist_threshold: 判断 next_to 的 XY 平面距离阈值 (米)
        :param z_tolerance: 判断 on/under 时的 Z 轴高度误差容忍度 (米)
        """
        num_objs = len(self.objects)
        for i in range(num_objs):
            for j in range(i + 1, num_objs):
                obj_a = self.objects[i]
                obj_b = self.objects[j]
                
                # 只在同一个房间内计算物理关系 (优化计算量)
                if obj_a.room_id != obj_b.room_id:
                    continue

                # --- 1. 检查 XY 平面重叠 (用于 ON / UNDER) ---
                overlap_x = max(obj_a.min_pt[0], obj_b.min_pt[0]) < min(obj_a.max_pt[0], obj_b.max_pt[0])
                overlap_y = max(obj_a.min_pt[1], obj_b.min_pt[1]) < min(obj_a.max_pt[1], obj_b.max_pt[1])
                xy_overlap = overlap_x and overlap_y

                # --- 2. 判断 ON 和 UNDER ---
                relation_found = False
                if xy_overlap:
                    # A 在 B 上: A 的底部 约等于 B 的顶部
                    if abs(obj_a.min_pt[2] - obj_b.max_pt[2]) < z_tolerance:
                        self.graph.add_edge(obj_a.node_id, obj_b.node_id, relation="ON")
                        self.graph.add_edge(obj_b.node_id, obj_a.node_id, relation="UNDER")
                        relation_found = True
                    # B 在 A 上: B 的底部 约等于 A 的顶部
                    elif abs(obj_b.min_pt[2] - obj_a.max_pt[2]) < z_tolerance:
                        self.graph.add_edge(obj_b.node_id, obj_a.node_id, relation="ON")
                        self.graph.add_edge(obj_a.node_id, obj_b.node_id, relation="UNDER")
                        relation_found = True

                # --- 3. 判断 NEXT_TO (如果不是 ON/UNDER 的关系) ---
                if not relation_found:
                    # 计算 3D 中心距离，或者 2D 平面距离
                    dist_xy = np.linalg.norm(obj_a.pos[:2] - obj_b.pos[:2])
                    # 距离小于阈值，且高度差异不大，认为是相邻
                    z_diff = abs(obj_a.pos[2] - obj_b.pos[2])
                    if dist_xy < dist_threshold and z_diff < 0.5:
                        self.graph.add_edge(obj_a.node_id, obj_b.node_id, relation="NEXT_TO")
                        self.graph.add_edge(obj_b.node_id, obj_a.node_id, relation="NEXT_TO")

    def print_graph(self):
        """格式化打印图的节点和边"""
        print("=== Scene Graph Nodes ===")
        for node, data in self.graph.nodes(data=True):
            if data['type'] == 'Room':
                print(f"📍 {node} (Room)")
            else:
                print(f"📦 {node} | Label: {data['label']:<10} | Pos: {[round(p, 2) for p in data['pos']]}")
                
        print("\n=== Scene Graph Edges (Relationships) ===")
        for source, target, data in self.graph.edges(data=True):
            rel = data['relation']
            # 根据关系类型加点简单的 emoji 方便终端查看
            icon = "➡️"
            if rel == "INSIDE": icon = "🏠"
            elif rel == "ON": icon = "⬆️"
            elif rel == "UNDER": icon = "⬇️"
            elif rel == "NEXT_TO": icon = "↔️"
            print(f"{icon} {source:<12} --[{rel:<8}]--> {target}")

    def visualize_2d_graph(self, save_path="scene_graph_2d.png"):
        """
        使用 matplotlib 和 networkx 进行 2D 物理-拓扑可视化 (优化排版版)
        """
        import matplotlib.pyplot as plt
        import networkx as nx
        import numpy as np
        
        # 尝试导入高级文字排版库
        try:
            from adjustText import adjust_text
            use_adjust_text = True
        except ImportError:
            use_adjust_text = False
            print("\n[提示] 未检测到 adjustText 库。为了获得无重叠的完美文字排版，强烈建议运行: pip install adjustText\n")

        # 稍微放大画布尺寸，给密集的节点更多空间
        plt.figure(figsize=(16, 14))
        
        # 1. 提取所有节点的 2D 物理坐标
        pos_dict = {}
        room_nodes = []
        object_nodes = []
        labels = {}

        for node_id, data in self.graph.nodes(data=True):
            if data['type'] == 'Room':
                room_nodes.append(node_id)
                labels[node_id] = f"Room {data['room_id']}"
                
                # 计算房间的多边形质心
                poly = np.array(data.get('polygon_2d', [[0,0]]))
                if len(poly) > 0:
                    cx, cy = np.mean(poly[:, 0]), np.mean(poly[:, 1])
                else:
                    cx, cy = 0, 0
                pos_dict[node_id] = (cx, cy)
                
            elif data['type'] == 'Object':
                object_nodes.append(node_id)
                # 简化标签，只保留类别和ID
                labels[node_id] = f"{data['label']}({node_id.split('_')[1]})"
                pos_dict[node_id] = (data['pos'][0], data['pos'][1])

        # 2. 区分边 (Edges)
        edge_inside = [(u, v) for u, v, d in self.graph.edges(data=True) if d['relation'] == 'INSIDE']
        edge_on = [(u, v) for u, v, d in self.graph.edges(data=True) if d['relation'] == 'ON']
        edge_next_to = [(u, v) for u, v, d in self.graph.edges(data=True) if d['relation'] == 'NEXT_TO']

        # 3. 绘制节点 (Nodes)
        # Room 节点画大一点，透明度调低一点作为背景底座
        nx.draw_networkx_nodes(self.graph, pos_dict, nodelist=room_nodes, 
                               node_color='#a1c9f4', node_shape='s', node_size=3500, alpha=0.8)
        # Object 节点画小一点，避免互相遮挡
        nx.draw_networkx_nodes(self.graph, pos_dict, nodelist=object_nodes, 
                               node_color='#8de5a1', node_shape='o', node_size=150, alpha=0.9)

        # 4. 绘制边 (Edges)
        nx.draw_networkx_edges(self.graph, pos_dict, edgelist=edge_inside, 
                               width=1.0, alpha=0.3, edge_color='gray', style='dashed')
        nx.draw_networkx_edges(self.graph, pos_dict, edgelist=edge_on, 
                               width=2.0, alpha=0.8, edge_color='#ff9f9b', arrowsize=12)
        nx.draw_networkx_edges(self.graph, pos_dict, edgelist=edge_next_to, 
                               width=1.0, alpha=0.5, edge_color='#d0bbff', style='dotted')

        # 5. 绘制文字标签 (核心优化点)
        texts = []
        for node_id, (x, y) in pos_dict.items():
            if node_id in room_nodes:
                # 房间名称直接居中写在蓝色大方块里面，加粗
                plt.text(x, y, labels[node_id], fontsize=12, fontweight='bold', 
                         ha='center', va='center', color='#333333', zorder=10)
            else:
                # 收集所有物体节点的文字对象
                texts.append(plt.text(x, y, labels[node_id], fontsize=8, color='black'))

        # 启用自动避让重叠排版
        if use_adjust_text and texts:
            adjust_text(texts, 
                        # 引线样式：浅灰色细线
                        arrowprops=dict(arrowstyle='-', color='gray', lw=0.5, alpha=0.7),
                        # 稍微增加文字相互推开的力度
                        expand_points=(1.5, 1.5),
                        expand_text=(1.2, 1.2))

        # 6. 图例与排版
        plt.title("2D Topological Semantic Scene Graph (Optimized Layout)", fontsize=18, pad=20)
        
        from matplotlib.lines import Line2D
        legend_elements = [
            Line2D([0], [0], marker='s', color='w', label='Room Node', markerfacecolor='#a1c9f4', markersize=15),
            Line2D([0], [0], marker='o', color='w', label='Object Node', markerfacecolor='#8de5a1', markersize=8),
            Line2D([0], [0], color='gray', lw=1.0, ls='--', alpha=0.5, label='Relation: INSIDE'),
            Line2D([0], [0], color='#ff9f9b', lw=2.0, label='Relation: ON'),
            Line2D([0], [0], color='#d0bbff', lw=1.0, ls=':', label='Relation: NEXT_TO')
        ]
        plt.legend(handles=legend_elements, loc='upper right', framealpha=0.9, fontsize=10)
        
        plt.axis('equal') 
        plt.grid(True, linestyle=':', alpha=0.4)
        # 去除边框，让画面更干净
        plt.gca().spines['top'].set_visible(False)
        plt.gca().spines['right'].set_visible(False)
        plt.gca().spines['bottom'].set_visible(False)
        plt.gca().spines['left'].set_visible(False)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=400, bbox_inches='tight') # 提高输出分辨率到 400 dpi
            print(f"✅ 高清 Scene Graph 拓扑图已保存至 {save_path}")
        else:
            plt.show()


# ==========================================
# Step 4: 示例运行 (Mock Data Demo)
# ==========================================
if __name__ == "__main__":
    # 1. 实例化 Scene Graph
    sg = SemanticSceneGraph()

    # 2. 创建 Room 节点
    room_1 = RoomNode(room_id=1, polygon_2d=[(0,0), (5,0), (5,5), (0,5)])
    sg.add_room(room_1)

    # 3. 创建 Object 节点 (伪造一些感知输出的数据)
    # (x, y, z), (dx, dy, dz)
    # 假设：桌子在房间中间
    table = ObjectNode(obj_id=101, pos=(2.5, 2.5, 0.4), bbox=(1.2, 0.8, 0.8), 
                       label="table", clip_feature=np.random.rand(512), room_id=1)
    
    # 假设：苹果在桌子上 (Z轴坐标: 桌子中心0.4 + 半高0.4 + 苹果半高0.05 = 0.85)
    apple = ObjectNode(obj_id=102, pos=(2.5, 2.5, 0.85), bbox=(0.1, 0.1, 0.1), 
                       label="apple", clip_feature=np.random.rand(512), room_id=1)
    
    # 假设：椅子在桌子旁边
    chair = ObjectNode(obj_id=103, pos=(2.5, 1.5, 0.25), bbox=(0.5, 0.5, 0.5), 
                       label="chair", clip_feature=np.random.rand(512), room_id=1)
    
    chair2 = ObjectNode(obj_id=104, pos=(3.5, 1.5, 0.25), bbox=(0.5, 0.5, 0.5), 
                       label="chair2", clip_feature=np.random.rand(512), room_id=1)

    # 将物体加入图中 (会自动建立 INSIDE 关系)
    sg.add_object(table)
    sg.add_object(apple)
    sg.add_object(chair)
    sg.add_object(chair2)

    # 4. 自动推理并构建空间关系边
    sg.compute_spatial_relations(dist_threshold=1.5, z_tolerance=0.1)

    # 5. 打印结果
    sg.print_graph()
    sg.visualize_2d_graph(save_path="demo_scene_graph.png")