
import numpy as np

class Axis:
    def __init__(self, x_dir, y_dir, xo=0, yo=0,):
        # origin
        self.xo= xo
        self.yo= yo
        # direction
        x_dir= x_dir - xo
        y_dir= y_dir - yo
        # normalize
        self.x_dir= x_dir/np.linalg.norm([x_dir, y_dir])
        self.y_dir= y_dir/np.linalg.norm([x_dir, y_dir])
        
        self.dir_vec = np.array([self.x_dir, self.y_dir])
        print(self.dir_vec)
    
    def project_of(self, x, y):
        return np.dot([x-self.xo, y-self.yo], self.dir_vec)
    
def get_max_min_interval(axis, points):
    '''
    get the max and min value of the projection of points on the axis
    '''
    min_val= np.inf
    max_val= -np.inf
    for point in points:
        print(axis.project_of(*point))
        min_val= min(min_val, axis.project_of(*point))
        max_val= max(max_val, axis.project_of(*point))
    return (min_val, max_val)

def check_interval_overlap(interval1, interval2):
    '''
    check if two intervals overlap
    '''
    return interval1[1] >= interval2[0] and interval2[1] >= interval1[0]



x_axis= Axis(1, 1)
test_point = np.array([1, 1])
print(x_axis.project_of(*test_point))
print(np.sqrt(2))



test_points_1 = [np.array([1.5, 1.5]), np.array([1, 1]), np.array([2, 2])]
test_points_2 = [np.array([1, 1]),]
print(get_max_min_interval(x_axis, test_points_1))

interval_1  = get_max_min_interval(x_axis, test_points_1)
interval_2  = get_max_min_interval(x_axis, test_points_2)
print(interval_1)
print(interval_2)

print(check_interval_overlap(interval_1, interval_2))