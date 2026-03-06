import numpy as np
import rerun
import rerun.blueprint as rrb
import matplotlib.pyplot as plt
import torch
import open3d as o3d
from pathlib import Path
from PIL import Image
from scipy.spatial.transform import Rotation


from boxfusion.batching import Sensors
from boxfusion.color import random_color_v2
from boxfusion.capture_stream import ScannetDataset, CA1MDataset, ROSDataset, HM3DDataset
import pickle
import open_clip

def move_device_like(src: torch.Tensor, dst: torch.Tensor) -> torch.Tensor:
    try:
        return src.to(dst)
    except:
        return src.to(dst.device)

def move_to_current_device(x, t):
    if isinstance(x, (list, tuple)):
        return [move_device_like(x_, t) for x_ in x]
    
    return move_device_like(x, t)

def move_input_to_current_device(batched_input: Sensors, t: torch.Tensor):
    # Assume only two levels of nesting for now.
    return { name: { name_: move_to_current_device(m, t) for name_, m in s.items() } for name, s in batched_input.items() }


def visualize_online_boxes(instances, prefix, boxes_3d_name="gt_boxes_3d", log_instances_name="instances", count=0,save=False, show_class=False, show_label=True,**kwargs):

    all_centers=[]
    all_sizes=[]
    all_colors=[]
    all_quaternions = []

    colors = [] 
    cur_instance =  instances 
    boxes_3d = cur_instance.get(boxes_3d_name)

    ids = None
   
    colors = [random_color_v2(ind/len(boxes_3d)) for ind in range(len(boxes_3d))]
    
    quaternions = [
        rerun.Quaternion(
            xyzw=Rotation.from_matrix(boxes_3d.R.cpu().numpy()[r]).as_quat()
        )
        for r in range(boxes_3d.R.cpu().numpy().shape[0])
    ]

    all_quaternions = quaternions

    # Hard-code these suffixes.
    centers_box=boxes_3d.gravity_center.cpu().numpy()
    sizes_size=boxes_3d.dims.cpu().numpy()
    
    all_centers=centers_box
    all_sizes=sizes_size 
    all_colors=colors
    if not show_class:
        ids = np.arange(all_sizes.shape[0])
        ids = ids.astype(str)
    else:
        ids = cur_instance.categories


    exclude_mask = np.arange(ids.shape[0],dtype=int)

    rerun.log(
        f"{prefix}/{log_instances_name}",
        rerun.Boxes3D(
            centers=all_centers[exclude_mask],
            sizes=all_sizes[exclude_mask],
            quaternions=all_quaternions,
            colors=all_colors,
            labels=ids[exclude_mask],
            show_labels=show_label),
        **kwargs)
        
    # save bbox.ply
    if save:
        boxes3d_to_ply(sizes_size,centers_box,colors,quaternions,f'./result/box_{count}.ply')


def boxes3d_to_ply(sizes, centers, colors, quaternions, output_path):
    # Extract centers, sizes and quaternions

    vertices = []
    faces = []
    vertex_colors = []
    face_template = np.array([[0, 1, 2], [0, 2, 3],  # bottom face
                              [4, 5, 6], [4, 6, 7],  # top face
                              [0, 1, 5], [0, 5, 4],  # front face
                              [1, 2, 6], [1, 6, 5],  # right face
                              [2, 3, 7], [2, 7, 6],  # back face
                              [3, 0, 4], [3, 4, 7]], dtype=np.int32)  # left face
    
    for i in range(len(centers)):
        # Calculate 8 vertices of the cube
        half_size = sizes[i] / 2
        corners = np.array([[-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
                            [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]]) * half_size
        
        # Apply rotation (quaternion)
        from scipy.spatial.transform import Rotation
        rot = Rotation.from_quat(quaternions[i]).as_matrix()
        corners = np.dot(corners, rot.T) + centers[i]
        
        # Add to vertex list
        vertices.append(corners)
        vertex_colors.extend([colors[i]] * 8)  # Same color for each vertex

        # Add face indices (offset by current box's vertex indices)
        faces.append(face_template + 8 * i)
    
    # Merge all vertices and faces
    vertices = np.vstack(vertices)
    faces = np.vstack(faces)
    # print("vertices", vertices.shape)

    # Create mesh and save
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(vertices)
    mesh.triangles = o3d.utility.Vector3iVector(faces)
    mesh.vertex_colors = o3d.utility.Vector3dVector(vertex_colors)  # Key step
    o3d.io.write_triangle_mesh(output_path, mesh)



def flip_axis_to_camera(pc):
    ''' Flip X-right,Y-forward,Z-up to X-right,Y-down,Z-forward
    Input and output are both (N,3) array
    '''
    pc2 = np.copy(pc)
    pc2[...,[0,1,2]] = pc2[...,[0,2,1]] # cam X,Y,Z = depth X,-Z,Y
    pc2[...,1] *= -1
    return pc2


def flip_axis_to_camera_AABB(pc):
    ''' Flip X-right,Y-forward,Z-up to X-right,Y-down,Z-forward
    Input and output are both (N,3) array
    '''
    pc2 = np.copy(pc)
    pc2[...,[0,1,2]] = pc2[...,[0,2,1]] # cam X,Y,Z = depth X,-Z,Y
    pc2[...,1] *= -1

    min_vals = np.min(pc2, axis=0)  #  [min_x, min_y, min_z]
    max_vals = np.max(pc2, axis=0)  #  [max_x, max_y, max_z]
    
    x = [min_vals[0], max_vals[0]]
    y = [min_vals[1], max_vals[1]]
    z = [min_vals[2], max_vals[2]]
    
    # [xmax,ymax,zmax] 111
    # [xmax,ymax,zmin] 110
    # [xmin,ymax,zmin] 010
    # [xmin,ymax,zmax] 011
    # [xmax,ymin,zmax] 101
    # [xmax,ymin,zmin] 100
    # [xmin,ymin,zmin] 000
    # [xmin,ymin,zmax] 001

    # aabb_corners = np.array([
    #     [x[0], y[0], z[0]],  # v0: min_x, min_y, min_z
    #     [x[1], y[0], z[0]],  # v1: max_x, min_y, min_z
    #     [x[1], y[1], z[0]],  # v2: max_x, max_y, min_z
    #     [x[0], y[1], z[0]],  # v3: min_x, max_y, min_z
    #     [x[0], y[0], z[1]],  # v4: min_x, min_y, max_z
    #     [x[1], y[0], z[1]],  # v5: max_x, min_y, max_z
    #     [x[1], y[1], z[1]],  # v6: max_x, max_y, max_z
    #     [x[0], y[1], z[1]]   # v7: min_x, max_y, max_z
    # ])

    aabb_corners = np.array([
        [x[1], y[1], z[1]],  # v0: min_x, min_y, min_z
        [x[1], y[1], z[0]],  # v1: max_x, min_y, min_z
        [x[0], y[1], z[0]],  # v2: max_x, max_y, min_z
        [x[0], y[1], z[1]],  # v3: min_x, max_y, min_z
        [x[1], y[0], z[1]],  # v4: min_x, min_y, max_z
        [x[1], y[0], z[0]],  # v5: max_x, min_y, max_z
        [x[0], y[0], z[0]],  # v6: max_x, max_y, max_z
        [x[0], y[0], z[1]]   # v7: min_x, max_y, max_z
    ])

    return aabb_corners



def generate_jet_colors(N):
    """
    Generate N RGB values using jet colormap, range [0,1]
    :param N: number of points
    :return: [N,3] RGB array
    """
    cmap = plt.get_cmap('jet')  # Get jet colormap
    colors = cmap(np.linspace(0, 1, N))[:, :3]  # Take first 3 columns (RGB), ignore Alpha channel
    return colors

def read_ply_with_rgb_open3d(file_path):
    """
    Read PLY file and return [N,6] array (xyz+rgb)
    :param file_path: PLY file path
    :return: np.ndarray [N,6]
    """
    pcd = o3d.io.read_point_cloud(file_path)  # Read point cloud
    points = np.asarray(pcd.points)  # [N,3] xyz
    colors = np.asarray(pcd.colors)  # [N,3] rgb (range [0,1])
    
    # Convert rgb from [0,1] to [0,255] (optional)
    # colors = (colors * 255).astype(np.uint8)
    colors = colors 
    
    # Merge xyz and rgb
    point_cloud = np.hstack((points, colors))  # [N,6]
    return point_cloud

def get_camera_coords(depth):
    height, width = depth.shape
    device = depth.device

    # camera xy.
    camera_coords = torch.stack(
        torch.meshgrid(
            torch.arange(0, width, device=device),
            torch.arange(0, height, device=device), indexing="xy"),
        dim=-1)

    return camera_coords

def unproject(depth, K, RT, max_depth=10.0):
    """
    Unproject depth image to 3D world coordinates.
    
    This function converts a depth image into 3D world coordinates by:
    1. Converting depth values to camera coordinates
    2. Applying inverse camera intrinsics to get 3D camera space points
    3. Transforming to world coordinates using the given transformation matrix
    
    Args:
        depth (torch.Tensor): Depth image tensor of shape (H, W) containing depth values
        K (torch.Tensor): Camera intrinsic matrix of shape (3, 3)
        RT (torch.Tensor): Camera extrinsic transformation matrix of shape (4, 4) 
                          for camera-to-world transformation
        max_depth (float, optional): Maximum depth threshold for filtering valid points.
                                   Points beyond this depth are marked invalid. Defaults to 10.0.
    
    Returns:
        tuple: A tuple containing:
            - world_xyz (torch.Tensor): 3D world coordinates of shape (H, W, 3)
            - valid (torch.Tensor): Boolean mask of shape (H, W) indicating valid depth points
                                  (depth > 0 and optionally depth < max_depth)
    
    Note:
        This function assumes the existence of a helper function `get_camera_coords(depth)`
        that generates normalized camera coordinates for the depth image.
    """
    camera_coords = get_camera_coords(depth) * depth[..., None]

    intrinsics_4x4 = torch.eye(4, device=depth.device)
    intrinsics_4x4[:3, :3] = K

    valid = depth > 0
    if max_depth is not None:
        valid &= (depth < max_depth)

    depth = depth[..., None]
    uvd = torch.cat((camera_coords, depth, torch.ones_like(depth)), dim=-1)

    camera_xyz =  torch.linalg.inv(intrinsics_4x4) @ uvd.view(-1, 4).T
    world_xyz = RT @ camera_xyz

    return world_xyz.T[..., :-1].reshape(uvd.shape[0], uvd.shape[1], 3), valid


def get_dataset(config):
    '''
    Get the dataset class from the config file.
    '''
    if config['dataset'] == 'scannet':
        dataset = ScannetDataset

    elif config['dataset'] == 'CA1M':
        dataset = CA1MDataset
        
    elif config['dataset'] == 'online':
        dataset = ROSDataset

    elif config['dataset'] == 'hm3d':
        dataset = HM3DDataset
        
    return dataset(config,)

def post_process(boxes, threshold=0.3):

    min_vals = np.min(boxes, axis=1)  # [N, 3] 
    max_vals = np.max(boxes, axis=1)  # [N, 3] 
    ranges = max_vals - min_vals     # [N, 3]
    

    valid_x = ranges[:, 0] >= threshold
    valid_y = ranges[:, 1] >= threshold
    valid_z = ranges[:, 2] >= threshold
    
    valid_mask = valid_x & valid_y & valid_z
    
    boxes = boxes[valid_mask]
    
    return boxes




def save_box(data, filename):
    """Save list data to pickle file
    
    Args:
        data: Data to be saved (containing tuples, numpy arrays)
        filename: Filename (default: object_data.pkl)
    """
    with open(filename, 'wb') as file:
        
        pickle.dump(data, file, protocol=pickle.HIGHEST_PROTOCOL)
    print(f"Results successfully saved to {filename}")


def load_data(filename):

    with open(filename, 'rb') as file:
        data = pickle.load(file)
    print(f"load {filename} data")
    return data

def load_clip(pretrained_path=None):
    print(f'[INFO] loading CLIP model...')

    if pretrained_path is None:
        model, _, preprocess = open_clip.create_model_and_transforms("ViT-H-14", pretrained="laion2b_s32b_b79k")
    else:
        model, _, preprocess = open_clip.create_model_and_transforms("ViT-H-14", pretrained=pretrained_path)  # load from local (OK!)

    model.cuda()
    model.eval()
    print(f'[INFO]', ' finish loading CLIP model...')
    return model, preprocess

def scale_boxes(boxes, H, W, scale=1.2):
    """
    Scale 2D bounding boxes with fixed center points
    
    Parameters:
    boxes : np.ndarray [N,4] in format [x_min,y_min,x_max,y_max]
    H : int image height
    W : int image width
    scale : float scaling factor
    
    Returns:
    scaled_boxes : np.ndarray [N,4] scaled bounding boxes
    """
    centers_x = (boxes[:, 0] + boxes[:, 2]) / 2
    centers_y = (boxes[:, 1] + boxes[:, 3]) / 2
    widths = boxes[:, 2] - boxes[:, 0]
    heights = boxes[:, 3] - boxes[:, 1]
    
    new_widths = widths * scale
    new_heights = heights * scale

    x_min = np.clip(centers_x - new_widths/2, 0, W)
    x_max = np.clip(centers_x + new_widths/2, 0, W)
    y_min = np.clip(centers_y - new_heights/2, 0, H)
    y_max = np.clip(centers_y + new_heights/2, 0, H)
    
    return np.stack([x_min, y_min, x_max, y_max], axis=1)

@torch.no_grad()
def retriev(model, preprocess, elements, text_features, device) -> int:
    """
    批处理优化：先在 CPU 上完成所有预处理并堆叠，然后一次性整体发送给 GPU
    """
    # 1. 列表推导式仅做 CPU 上的图像尺寸变换
    preprocessed_list = [preprocess(image) for image in elements]
    
    # 2. 堆叠成一个大的 Tensor [N, 3, 224, 224] 后，一次性转移到 GPU
    stacked_images = torch.stack(preprocessed_list).to(device)
    
    # 3. 批量推理
    image_features = model.encode_image(stacked_images)
    image_features /= image_features.norm(dim=-1, keepdim=True)
    text_features /= text_features.norm(dim=-1, keepdim=True) 

    probs = 100.0 * image_features @ text_features.T

    return probs, image_features

def segment_image(image_array, bbox):
    """
    极速裁剪函数：抛弃所有无用的蒙版生成，直接在内存中进行 Numpy 切片
    """
    x1, y1, x2, y2 = map(int, bbox)
    
    # 增加边界保护，防止预测框超出图像边缘导致程序崩溃
    h, w = image_array.shape[:2]
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(w, x2), min(h, y2)

    # 纯 Numpy 切片，耗时几乎为 0
    crop_array = image_array[y1:y2, x1:x2]
    
    # 防止因越界产生空数组
    if crop_array.size == 0:
        return Image.new("RGB", (224, 224), (0, 0, 0))
        
    return Image.fromarray(crop_array)

def crop_image(boxes, rgb):
    """
    直接传入 numpy 数组，避免重复转换
    """
    cropped_boxes = []
    cropped_images = []

    for cur_box in boxes:
        cropped_images.append(segment_image(rgb, cur_box))  
        cropped_boxes.append(cur_box)  

    return cropped_boxes, cropped_images

def text_prompt(boxes, class_prompt, text_features, img_path, clip_model, preprocess):
    cropped_boxes, cropped_images= crop_image(
        boxes, img_path
    )


    scores, img_features = retriev(
        clip_model, preprocess, cropped_images, text_features, device="cuda:0"
    )

    max_values, max_id = torch.max(scores, dim=-1) #
    max_id = max_id.cpu().numpy()
    categories = class_prompt[max_id]

    return  categories, img_features