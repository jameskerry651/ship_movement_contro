import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from scipy.spatial.distance import cdist

def generate_ship_path(pier_positions, left_border_points, right_border_points, 
                      num_points=1000, bridge_avoidance_distance=300, smoothing_iterations=3,
                      right_bias=0.3):  # Added right_bias parameter
    """
    Generate a ship path that follows the main channel and avoids bridges.
    The path will have a bias towards the right side of the channel.
    
    Parameters:
    -----------
    pier_positions : numpy.ndarray
        Array of pier positions (potential bridge locations)
    left_border_points : numpy.ndarray
        Points defining the left border of the navigable channel
    right_border_points : numpy.ndarray
        Points defining the right border of the navigable channel
    num_points : int
        Number of points to generate in the path
    bridge_avoidance_distance : float
        Minimum distance to maintain from bridges
    smoothing_iterations : int
        Number of smoothing passes to apply to the final path
    right_bias : float
        Amount of bias towards the right side (0.0 = center, 0.5 = halfway to right border)
        
    Returns:
    --------
    numpy.ndarray
        Array of waypoints for the ship path
    """
    # Step 1: Identify bridge structures from pier positions
    bridges = identify_bridges(pier_positions)
    
    # Step 2: Generate center path between borders with a right bias
    center_path, _i, _j = generate_middle_path(left_border_points, right_border_points, num_points, right_bias=right_bias)

    
    # Step 3: Adjust path to avoid bridges
    safe_path = avoid_bridges(center_path, bridges, left_border_points, right_border_points, 
                             avoidance_distance=bridge_avoidance_distance, right_bias=right_bias)
    
    # Step 4: Smooth the path
    final_path = smooth_path(safe_path, iterations=smoothing_iterations)
    
    return np.array(final_path)

def identify_bridges(pier_positions, proximity_threshold=100):
    """
    Identify bridge structures from pier positions based on proximity.
    
    Parameters:
    -----------
    pier_positions : numpy.ndarray
        Array of pier positions
    proximity_threshold : float
        Maximum distance between piers to be considered part of the same bridge
        
    Returns:
    --------
    list
        List of arrays, each containing positions of piers forming a bridge
    """
    bridges = []
    current_bridge = [pier_positions[0]]
    
    # Group nearby piers into bridge structures
    for i in range(1, len(pier_positions)):
        dist = np.linalg.norm(pier_positions[i] - pier_positions[i-1])
        
        if dist < proximity_threshold:
            current_bridge.append(pier_positions[i])
        else:
            if len(current_bridge) > 1:
                bridges.append(np.array(current_bridge))
            current_bridge = [pier_positions[i]]
    
    # Add the last bridge if it exists
    if len(current_bridge) > 1:
        bridges.append(np.array(current_bridge))
        
    return bridges

def distance(p1, p2):
    """Calculate Euclidean distance between two points."""
    return np.linalg.norm(np.array(p1) - np.array(p2))

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

    

def is_near_bridge(point, bridges, threshold):
    """Check if a point is close to any bridge pier."""
    for bridge in bridges:
        distances = cdist([point], bridge)
        if np.min(distances) < threshold:
            return True, np.argmin(distances)
    return False, None

def avoid_bridges(path, bridges, left_border, right_border, avoidance_distance=200, right_bias=0.3):
    """
    Modify the path to avoid bridges with a preference for the right side.
    
    Parameters:
    -----------
    path : list
        List of points forming the initial path
    bridges : list
        List of bridge structures to avoid
    left_border : numpy.ndarray
        Points defining the left border
    right_border : numpy.ndarray
        Points defining the right border
    avoidance_distance : float
        Minimum distance to maintain from bridges
    right_bias : float
        Amount of bias towards the right side when avoiding bridges
        
    Returns:
    --------
    list
        List of points forming the safe path
    """
    safe_path = []
    path_length = len(path)
    
    for i, point in enumerate(path):
        near_bridge, _ = is_near_bridge(point, bridges, avoidance_distance)
        
        if near_bridge:
            # Find nearest bridge pier
            min_dist = float('inf')
            nearest_pier = None
            nearest_bridge_idx = None
            
            for b_idx, bridge in enumerate(bridges):
                for pier in bridge:
                    dist = distance(point, pier)
                    if dist < min_dist:
                        min_dist = dist
                        nearest_pier = pier
                        nearest_bridge_idx = b_idx
            
            if nearest_pier is not None:
                # Find corresponding points on borders at similar parametric position
                t = i / (path_length - 1)
                left_idx = min(int(t * len(left_border)), len(left_border) - 1)
                right_idx = min(int(t * len(right_border)), len(right_border) - 1)
                
                left_point = left_border[left_idx]
                right_point = right_border[right_idx]
                
                # Calculate distances from nearest pier to border points
                dist_left = distance(left_point, nearest_pier)
                dist_right = distance(right_point, nearest_pier)
                
                # Choose direction with preference for right side
                # If right side is at least 80% as safe as left side, prefer right
                if dist_right >= 0.8 * dist_left:
                    # Move toward right border
                    safe_point = [
                        point[0] + 0.7 * (right_point[0] - point[0]),
                        point[1] + 0.7 * (right_point[1] - point[1])
                    ]
                else:
                    # Move toward left border if right is significantly more dangerous
                    safe_point = [
                        point[0] + 0.7 * (left_point[0] - point[0]),
                        point[1] + 0.7 * (left_point[1] - point[1])
                    ]
                
                safe_path.append(safe_point)
            else:
                safe_path.append(point)
        else:
            # If point is not near a bridge, keep it as is
            safe_path.append(point)
    
    return safe_path

def smooth_path(path, iterations=3):
    """
    Apply smoothing to the path.
    
    Parameters:
    -----------
    path : list
        List of points forming the path to smooth
    iterations : int
        Number of smoothing passes to apply
        
    Returns:
    --------
    list
        List of points forming the smoothed path
    """
    smoothed = path
    
    for _ in range(iterations):
        new_path = [smoothed[0]]  # Keep first point
        
        for i in range(1, len(smoothed) - 1):
            # Average point with its neighbors
            new_point = [
                (smoothed[i-1][0] + smoothed[i][0] + smoothed[i+1][0]) / 3,
                (smoothed[i-1][1] + smoothed[i][1] + smoothed[i+1][1]) / 3
            ]
            new_path.append(new_point)
        
        new_path.append(smoothed[-1])  # Keep last point
        smoothed = new_path
    
    return smoothed

def plot_results(center_ref, pier_positions, left_border, right_border, path):
    """
    Visualize the ship path, channel borders, and bridge positions.
    
    Parameters:
    -----------
    pier_positions : numpy.ndarray
        Array of pier positions
    left_border : numpy.ndarray
        Points defining the left border
    right_border : numpy.ndarray
        Points defining the right border
    path : numpy.ndarray
        Array of waypoints for the ship path
    """
    plt.figure(figsize=(12, 8))
    
    # Plot borders
    plt.plot(left_border[:, 0], left_border[:, 1], 'r-', linewidth=1, label='Left Border')
    plt.plot(right_border[:, 0], right_border[:, 1], 'b-', linewidth=1, label='Right Border')
    
    # Plot piers/bridges
    plt.scatter(pier_positions[:, 0], pier_positions[:, 1], color='black', s=20, label='Piers')
    
    # Calculate and plot center line (reference only)
    
    center_line = np.array(center_ref)
    plt.plot(center_line[:, 0], center_line[:, 1], 'y--', linewidth=1, alpha=0.5, label='Center Reference')
    
    # Plot generated path
    path = np.array(path)
    plt.plot(path[:, 0], path[:, 1], 'g-', linewidth=2, label='Ship Path (Right-Biased)')
    
    # Mark start and end points
    plt.scatter(path[0, 0], path[0, 1], color='green', s=100, marker='^', label='Start')
    plt.scatter(path[-1, 0], path[-1, 1], color='red', s=100, marker='v', label='End')
    
    plt.legend()
    plt.title('Ship Route Planning (Middle-Right Path)')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # Sample data
    pier_positions = np.array([
        [904.6367455, 1983.85201],
        [1529.108235, 1504.268175],
        [1660.172983, 1407.708453],
        [2548.682218, 4460.930873],
        [3869.961073, 3377.928421],
        [4374.625395, 7076.262667],
        [5086.475821, 6594.347355],
        [5798.293632, 6112.432366],
        [5218.089009, 8879.540071],
        [5339.494183, 8801.271574],
        [5460.997113, 8722.892224],
        [5582.405801, 8644.623744],
        [5703.816304, 8566.244411],
        [5825.228506, 8487.975948],
        [5946.73847, 8409.596632],
        [6068.154185, 8331.328186],
        [8698.153323, 14880.07622],
        [8791.218583, 14814.66716],
        [8846.385963, 14776.75208],
        [9367.014633, 14394.27549],
        [9486.000621, 14304.6984],
        [9637.70713, 14194.38999],
        [10754.16604, 17507.08645],
        [11281.68388, 17076.49381],
        [11809.24378, 16645.90142],
        [16659.98653, 20888.97713],
        [16706.61213, 20795.1864],
        [17012.44797, 20113.2639],
        [23047.75274, 23318.67112],
        [23403.45659, 22417.45724]
    ])
    
    left_border_points = np.array([
        [0, 1007.501251],
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
    
    
    # Generate the ship path with a right bias of 0.3
    # This means the path will be 30% closer to the right border than to the left
    right_bias = 0.3  # Adjust this value between 0.0 and 0.5 to control the bias
    ship_path = generate_ship_path(pier_positions, left_border_points, right_border_points, 
                                  num_points=1000, bridge_avoidance_distance=350,
                                  right_bias=right_bias)
    
    # Display the results

    center_ref, _x, _y = generate_middle_path(left_border_points, right_border_points, num_points=1000, right_bias=right_bias)
    plot_results(center_ref, pier_positions, left_border_points, right_border_points, ship_path)
        
    # Save the waypoints to a file
    np.savetxt('ship_waypoints_right_biased.csv', ship_path, delimiter=',', header='x,y', comments='')
    print("Waypoints saved to 'ship_waypoints_right_biased.csv'")