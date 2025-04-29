
"""
@Description: 根据bias的值生成偏右的路径
@Author: 张瑞明
@Date: 2025-04-30 02:40:00
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

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

def generate_middle_path(left_points, right_points, num_points=100, right_bias=0.0):
    """
    Generate middle path points between left and right border points with optional right bias.
    
    Parameters:
    -----------
    left_points : np.array
        Left border points with shape (n, 2)
    right_points : np.array
        Right border points with shape (m, 2)
    num_points : int
        Number of points to generate for the middle path
    right_bias : float
        Bias towards the right border (0.0 to 1.0)
        0.0 means middle path (no bias)
        0.5 means 50% towards the right border
        1.0 means exactly on the right border
        
    Returns:
    --------
    np.array
        Middle path points with shape (num_points, 2)
    """
    # Ensure right_bias is between 0 and 1
    right_bias = max(0.0, min(1.0, right_bias))
    
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
    
    # Calculate weighted average based on right_bias
    # right_bias = 0.0 -> middle path (50% left, 50% right)
    # right_bias = 1.0 -> right border (0% left, 100% right)
    left_weight = 0.5 - right_bias / 2
    right_weight = 0.5 + right_bias / 2
    
    middle_path = left_weight * left_samples + right_weight * right_samples
    
    return middle_path, left_samples, right_samples

# Generate middle path with right bias
num_points = 200
right_bias = 0.3  # Adjust this value between 0.0 and 1.0 to control the bias
middle_path, left_samples, right_samples = generate_middle_path(left_border_points, right_border_points, num_points=num_points, right_bias=right_bias)

# Visualize the result
plt.figure(figsize=(12, 12))
plt.plot(left_border_points[:, 0], left_border_points[:, 1], 'ro-', label='Original Left Border')
plt.plot(right_border_points[:, 0], right_border_points[:, 1], 'bo-', label='Original Right Border')
plt.plot(left_samples[:, 0], left_samples[:, 1], 'r--', alpha=0.5, label='Interpolated Left Border')
plt.plot(right_samples[:, 0], right_samples[:, 1], 'b--', alpha=0.5, label='Interpolated Right Border')
plt.plot(middle_path[:, 0], middle_path[:, 1], 'g-', linewidth=2, label=f'Middle Path (Right Bias: {right_bias:.1f})')

# Generate paths with different bias values for comparison
bias_values = [0.0, 0.5, 0.8]
bias_colors = ['y-', 'c-', 'm-']

for bias, color in zip(bias_values, bias_colors):
    if bias != right_bias:  # Skip if it's the same as the main bias we're already showing
        bias_path, _, _ = generate_middle_path(left_border_points, right_border_points, num_points=num_points, right_bias=bias)
        plt.plot(bias_path[:, 0], bias_path[:, 1], color, linewidth=1, alpha=0.7, label=f'Bias: {bias:.1f}')

plt.axis('equal')
plt.grid(True)
plt.legend()
plt.title('Generated Middle Path with Right Bias')
plt.xlabel('X')
plt.ylabel('Y')
plt.savefig('middle_path_with_bias_visualization.png', dpi=300, bbox_inches='tight')
plt.show()

# Print some of the middle path points
print(f"\nGenerated {num_points} middle path points. First 5 points:")
for i in range(5):
    print(f"Point {i}: ({middle_path[i, 0]:.2f}, {middle_path[i, 1]:.2f})")

# Function to save the middle path points to a file
def save_middle_path(middle_path, filename='middle_path_points.txt', bias=0.0):
    np.savetxt(filename, middle_path, fmt='%.6f', delimiter=',', 
               header='x,y', comments='')
    print(f"\nMiddle path points with right bias {bias:.2f} saved to {filename}")

# Save the middle path points
save_middle_path(middle_path, filename=f'middle_path_points_bias_{right_bias:.1f}.txt', bias=right_bias)

# Function to analyze lane width distribution with bias
def analyze_lane_width(middle_path, left_samples, right_samples):
    """
    Analyze the distance from middle path to both borders
    """
    # Calculate distances to left and right borders for each point
    left_distances = np.sqrt(np.sum((middle_path - left_samples)**2, axis=1))
    right_distances = np.sqrt(np.sum((middle_path - right_samples)**2, axis=1))
    
    # Calculate statistics
    avg_left_dist = np.mean(left_distances)
    avg_right_dist = np.mean(right_distances)
    max_left_dist = np.max(left_distances)
    max_right_dist = np.max(right_distances)
    min_left_dist = np.min(left_distances)
    min_right_dist = np.min(right_distances)
    
    print("\nLane Width Analysis:")
    print(f"  Average distance to left border: {avg_left_dist:.2f}")
    print(f"  Average distance to right border: {avg_right_dist:.2f}")
    print(f"  Maximum distance to left border: {max_left_dist:.2f}")
    print(f"  Maximum distance to right border: {max_right_dist:.2f}")
    print(f"  Minimum distance to left border: {min_left_dist:.2f}")
    print(f"  Minimum distance to right border: {min_right_dist:.2f}")
    
    # Plot distance distribution
    plt.figure(figsize=(10, 6))
    plt.plot(left_distances, 'r-', label='Distance to Left Border')
    plt.plot(right_distances, 'b-', label='Distance to Right Border')
    plt.grid(True)
    plt.legend()
    plt.title(f'Lane Width Analysis (Right Bias: {right_bias:.1f})')
    plt.xlabel('Point Index')
    plt.ylabel('Distance')
    plt.savefig(f'lane_width_analysis_bias_{right_bias:.1f}.png', dpi=300, bbox_inches='tight')
    plt.show()

# Analyze lane width
analyze_lane_width(middle_path, left_samples, right_samples)
