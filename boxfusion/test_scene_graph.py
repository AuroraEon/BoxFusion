import numpy as np
import os
from scene_graph_builder import SemanticSceneGraph, RoomNode, ObjectNode

def run_mock_test():
    print("🚀 开始运行 Scene Graph 独立测试...\n")
    sg = SemanticSceneGraph()

    # ==========================================
    # 1. 模拟构建房间 (Rooms)
    # ==========================================
    # 房间 1：一个 5x5 的方形房间，左下角在 (0,0)
    room1 = RoomNode(room_id=1, polygon_2d=[(0, 0), (5, 0), (5, 5), (0, 5)])
    
    # 房间 2：紧挨着房间 1 的另一个 5x5 的房间，左下角在 (5,0)
    room2 = RoomNode(room_id=2, polygon_2d=[(5, 0), (10, 0), (10, 5), (5, 5)])
    
    sg.add_room(room1)
    sg.add_room(room2)
    print("✅ 房间模拟加载完成 (Room 1, Room 2)")

    # ==========================================
    # 2. 模拟构建物体 (Objects)
    # 参数解释：pos=(x, y, z), bbox=(dx, dy, dz)
    # ==========================================
    
    # --- 场景 A：房间 1 中的物品 ---
    # 1. 桌子 (放在房间 1 中央)
    # 桌子中心点在 z=0.4，高度 dz=0.8，所以它的顶部在 z = 0.4 + 0.4 = 0.8，底部在 z=0
    table = ObjectNode(
        obj_id=101, pos=(2.5, 2.5, 0.4), bbox=(1.2, 0.8, 0.8),
        label="table", clip_feature=None, room_id=1
    )
    
    # 2. 苹果 (放在桌子上)
    # 桌子顶部在 0.8。苹果中心点设在 0.85，高度 0.1，所以苹果底部在 0.85 - 0.05 = 0.8。正好贴合桌子！
    apple = ObjectNode(
        obj_id=102, pos=(2.5, 2.5, 0.85), bbox=(0.1, 0.1, 0.1),
        label="apple", clip_feature=None, room_id=1
    )
    
    # 3. 椅子 (在桌子旁边)
    # x 坐标离桌子 1.0 米，y 坐标一样，高度比较矮
    chair = ObjectNode(
        obj_id=103, pos=(1.5, 2.5, 0.25), bbox=(0.5, 0.5, 0.5),
        label="chair", clip_feature=None, room_id=1
    )

    # 4. 垃圾桶 (在桌子下方)
    # 垃圾桶放在桌子坐标系内，高度比较矮，顶部 z=0.3（在桌板 0.8 之下）
    trash_can = ObjectNode(
        obj_id=104, pos=(2.5, 2.5, 0.15), bbox=(0.3, 0.3, 0.3),
        label="trash_can", clip_feature=None, room_id=1
    )

    # --- 场景 B：房间 2 中的物品 ---
    # 5. 沙发
    sofa = ObjectNode(
        obj_id=201, pos=(7.5, 2.5, 0.3), bbox=(2.0, 1.0, 0.6),
        label="sofa", clip_feature=None, room_id=2
    )

    # 依次加入图中
    for obj in [table, apple, chair, trash_can, sofa]:
        sg.add_object(obj)
    print("✅ 物体模拟加载完成 (table, apple, chair, trash_can, sofa)\n")

    # ==========================================
    # 3. 执行空间推理
    # ==========================================
    print("⚙️  正在执行空间关系推导...")
    # dist_threshold=1.5米用于判定NEXT_TO，z_tolerance=0.15用于判定贴合误差
    sg.compute_spatial_relations(dist_threshold=1.5, z_tolerance=0.15)
    
    # ==========================================
    # 4. 打印并在本地生成图
    # ==========================================
    sg.print_graph()
    
    # 检查能否顺利生成二维拓扑图
    os.makedirs("./debug_mock", exist_ok=True)
    vis_path = "./debug_mock/mock_scene_graph.png"
    sg.visualize_2d_graph(save_path=vis_path)
    
    print("\n🎉 测试完成！")
    print(f"👉 请打开 {vis_path} 查看生成的拓扑可视化图片。")

if __name__ == "__main__":
    run_mock_test()