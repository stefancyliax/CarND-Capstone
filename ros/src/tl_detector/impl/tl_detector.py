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

    def get_closest_waypoint(self, wp_2d):
        if self.wp_tree is None:
            raise ValueError("TLDetector not initialized!")
                
        dst, idx = self.wp_tree.query(wp_2d)
        return idx

