import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

"""
@Description: 用于插值生成中间路径
@Author: 张瑞明
@Date: 2025-04-30 02:40:00
"""

# Border points data
left_border_points = np.array([
    [0,	1007.501251],
    [3154.181328, 5276.648195],
    [8655.157244, 14927.41465],
    [10371.64335, 17827.03779],
    [12538.18496, 20006.94569],
    [14977.43922, 20867.02611],
    [16644.09058, 20880.77321],
    [23965.10102, 23606.25271],
    [26930.13775, 24113.12386],
    [29637.03472, 22936.96605],
    [30763.44882, 21532.31974],
    [30861.94671, 20802.05995]
])

right_border_points = np.array([
    [1606.136861, 0],
    [1503.001748, 64.74255572],
    [1999.498333, 1751.820309],
    [4023.590081, 3259.750547],
    [5827.658652, 6113.762703],
    [6110.092198, 8314.920638],
    [9696.803875, 14165.89827],
    [11829.95659, 16642.02121],
    [17062.23702, 19962.60025],
    [24077.67258, 22538.85331],
    [27476.72102, 22234.53169],
    [29899.42541, 20195.85731]
])

def generate_middle_path(left_points, right_points, num_points=1000):
    """
    Generate middle path points between left and right border points.
    
    Parameters:
    -----------
    left_points : np.array
        Left border points with shape (n, 2)
    right_points : np.array
        Right border points with shape (m, 2)
    num_points : int
        Number of points to generate for the middle path
        
    Returns:
    --------
    np.array
        Middle path points with shape (num_points, 2)
    """
    # Parameterize the points by their cumulative distance
    def parameterize_by_distance(points):
        dists = np.sqrt(np.sum(np.diff(points, axis=0)**2, axis=1))
        cum_dists = np.concatenate(([0], np.cumsum(dists)))
        return cum_dists / cum_dists[-1]  # Normalize to [0, 1]
    
    t_left = parameterize_by_distance(left_points)
    t_right = parameterize_by_distance(right_points)
    
    # Create interpolation functions for both borders
    fx_left = interp1d(t_left, left_points[:, 0], kind='linear')
    fy_left = interp1d(t_left, left_points[:, 1], kind='linear')
    
    fx_right = interp1d(t_right, right_points[:, 0], kind='linear')
    fy_right = interp1d(t_right, right_points[:, 1], kind='linear')
    
    # Sample both borders at regular intervals
    t_samples = np.linspace(0, 1, num_points)
    
    left_samples = np.column_stack((fx_left(t_samples), fy_left(t_samples)))
    right_samples = np.column_stack((fx_right(t_samples), fy_right(t_samples)))
    
    # Middle path is the average of corresponding points
    middle_path = (left_samples + right_samples) / 2
    
    return middle_path, left_samples, right_samples

# Generate middle path
num_points = 200
middle_path, left_samples, right_samples = generate_middle_path(left_border_points, right_border_points, num_points=num_points)

# Visualize the result
plt.figure(figsize=(12, 12))
plt.plot(left_border_points[:, 0], left_border_points[:, 1], 'ro-', label='Original Left Border')
plt.plot(right_border_points[:, 0], right_border_points[:, 1], 'bo-', label='Original Right Border')
plt.plot(left_samples[:, 0], left_samples[:, 1], 'r--', alpha=0.5, label='Interpolated Left Border')
plt.plot(right_samples[:, 0], right_samples[:, 1], 'b--', alpha=0.5, label='Interpolated Right Border')
plt.plot(middle_path[:, 0], middle_path[:, 1], 'g-', linewidth=2, label='Middle Path')
plt.axis('equal')
plt.grid(True)
plt.legend()
plt.title('Generated Middle Path')
plt.xlabel('X')
plt.ylabel('Y')
plt.savefig('middle_path_visualization.png', dpi=300, bbox_inches='tight')
plt.show()

# Print some of the middle path points
print(f"\nGenerated {num_points} middle path points. First 5 points:")
for i in range(5):
    print(f"Point {i}: ({middle_path[i, 0]:.2f}, {middle_path[i, 1]:.2f})")

# Function to save the middle path points to a file
def save_middle_path(middle_path, filename='middle_path_points.txt'):
    np.savetxt(filename, middle_path, fmt='%.6f', delimiter=',', 
               header='x,y', comments='')
    print(f"\nMiddle path points saved to {filename}")

# Save the middle path points
save_middle_path(middle_path)