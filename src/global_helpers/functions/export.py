import csv

def export_centerline(path, world_map, save_path):
    header = ['x_m', 'y_m', 'w_tr_right_m', 'w_tr_left_m']

    with open(save_path, 'w', encoding='UTF8', newline='') as f:
        path_world_coordinates = list(map(lambda point: world_map.grid_point_to_world_point(
            point), path))
        
        header_str = "# "
        for word in header[:-1]:
            header_str += word +","
        header_str += header[-1]
        f.write(f"{header_str}\n")

        writer = csv.writer(f)
        path_world_coords_reformatted = list(map(lambda point: [point.x, point.y, point.w_tr_right, point.w_tr_left], path_world_coordinates))
        writer.writerows(path_world_coords_reformatted)

def export_trajcetory(trajectory, save_path):
    with open(save_path, 'w') as f:
        header = "#x_m; y_m; psi_rad"
        f.write(header + "\n")
        for pose in trajectory:
            f.write(f"{pose.coordinate.x},{pose.coordinate.y},{pose.yaw}\n")