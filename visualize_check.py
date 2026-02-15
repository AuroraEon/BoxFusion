import json
import cv2
import numpy as np
import matplotlib.pyplot as plt
import networkx as nx

def visualize_output(output_dir="./output_maps"):
    # 1. 读取地图
    map_img = cv2.imread(f"{output_dir}/map.png", cv2.IMREAD_GRAYSCALE)
    if map_img is None:
        print("错误: 找不到 map.png")
        return
    
    # 读取 YAML 获取分辨率
    resolution = 0.05
    origin = [0, 0] # 简化，通常需要读取 yaml
    try:
        with open(f"{output_dir}/map.yaml", 'r') as f:
            for line in f:
                if "resolution" in line: resolution = float(line.split(':')[1])
                if "origin" in line: origin = eval(line.split(':')[1])
    except:
        pass

    h, w = map_img.shape
    debug_img = cv2.cvtColor(map_img, cv2.COLOR_GRAY2BGR)

    # 坐标转换函数 (World -> Grid)
    def to_grid(x, y):
        # 假设 map_builder 用的中心原点逻辑:
        # u = x / res + w/2
        # v = y / res + h/2
        # 如果你的 map_builder 逻辑不同，这里需要对应修改
        u = int(x / resolution + w/2)
        v = int(y / resolution + h/2)
        return u, v

    # 2. 绘制 Waypoints (蓝色小点)
    with open(f"{output_dir}/waypoints.json", 'r') as f:
        wps = json.load(f)
        for wp in wps:
            x, y = wp['waypoint_pos']
            u, v = to_grid(x, y)
            if 0 <= u < w and 0 <= v < h:
                cv2.circle(debug_img, (u, v), 2, (255, 0, 0), -1) # 蓝点

    # 3. 绘制拓扑图 (节点和边)
    with open(f"{output_dir}/topo_graph.json", 'r') as f:
        graph_data = json.load(f)
        
    # 绘制边 (黄色线)
    nodes_map = {n['id']: n for n in graph_data['nodes']}
    for link in graph_data['links']:
        n1 = nodes_map[link['source']]
        n2 = nodes_map[link['target']]
        u1, v1 = to_grid(n1['pos'][0], n1['pos'][1])
        u2, v2 = to_grid(n2['pos'][0], n2['pos'][1])
        cv2.line(debug_img, (u1, v1), (u2, v2), (0, 255, 255), 1)

    # 绘制节点 (房间=红, 门=绿)
    for node in graph_data['nodes']:
        u, v = to_grid(node['pos'][0], node['pos'][1])
        color = (0, 0, 255) if node['type'] == 'room' else (0, 255, 0)
        cv2.circle(debug_img, (u, v), 4, color, -1)
        # cv2.putText(debug_img, str(node['id']), (u, v), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0), 1)

    # 显示结果
    plt.figure(figsize=(12, 12))
    plt.imshow(cv2.cvtColor(debug_img, cv2.COLOR_BGR2RGB))
    plt.title(f"Verification: {len(graph_data['nodes'])} Nodes, {len(wps)} Waypoints")
    plt.axis('off')
    plt.show()
    # 保存
    cv2.imwrite(f"{output_dir}/verification_result.png", debug_img)
    print(f"验证图已保存至: {output_dir}/verification_result.png")

if __name__ == "__main__":
    visualize_output()