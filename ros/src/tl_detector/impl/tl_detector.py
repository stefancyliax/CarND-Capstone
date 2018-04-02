import scipy.spatial

class TLDetector(object):
    def __init__(self, waypoints_2d = None):
        self.waypoints_2d = waypoints_2d
        if self.waypoints_2d: 
            self.wp_tree = scipy.spatial.KDTree(waypoints_2d)
        else:
            self.wp_tree = None
        
    def is_ready(self):
        return self.wp_tree is not None

    def get_closest_waypoint(self, pos_2d):
        if self.wp_tree is None:
            raise ValueError("TLDetector not initialized!")
                
        dst, idx = self.wp_tree.query(pos_2d)
        return idx

    def get_closest_traffic_light(self, pos_2d, lights, pos_lights_stop_lines_2d):
        if len(lights) != len(pos_lights_stop_lines_2d):
            raise ValueError("Incompatible sizes!")
        if self.wp_tree is None:
            raise ValueError("TLDetector not initialized!")

        line_wp_idx, closest_light = -1, None
        
        car_wp_idx = self.get_closest_waypoint(pos_2d)
        diff_idx = len(self.waypoints_2d)
        for idx, light in enumerate(lights):
            # Get stop line waypoint index
            pos_stop_line = pos_lights_stop_lines_2d[idx]
            temp_wp_idx = self.get_closest_waypoint([pos_stop_line[0], pos_stop_line[1]])
            # Find closest stop line waypoint index
            d_idx = temp_wp_idx - car_wp_idx
            if d_idx >= 0 and d_idx < diff_idx:
                diff_idx = d_idx
                closest_light = light
                line_wp_idx = temp_wp_idx

        return line_wp_idx, closest_light
