import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from scipy.spatial.distance import cdist

def generate_ship_path(pier_positions, left_border_points, right_border_points, 
                      num_points=100, bridge_avoidance_distance=300, smoothing_iterations=3):
    """
    Generate a ship path that follows the main channel and avoids bridges.
    
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
        
    Returns:
    --------
    numpy.ndarray
        Array of waypoints for the ship path
    """
    # Step 1: Identify bridge structures from pier positions
    bridges = identify_bridges(pier_positions)
    
    # Step 2: Generate center path between borders
    center_path = generate_center_path(left_border_points, right_border_points, num_points)
    
    # Step 3: Adjust path to avoid bridges
    safe_path = avoid_bridges(center_path, bridges, left_border_points, right_border_points, 
                             avoidance_distance=bridge_avoidance_distance)
    
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

def generate_center_path(left_border, right_border, num_points):
    """
    Generate a center path between the left and right borders.
    
    Parameters:
    -----------
    left_border : numpy.ndarray
        Points defining the left border
    right_border : numpy.ndarray
        Points defining the right border
    num_points : int
        Number of points to generate
        
    Returns:
    --------
    list
        List of points forming the center path
    """
    # Parameterize both borders by cumulative distance
    left_distances = [0]
    for i in range(1, len(left_border)):
        left_distances.append(left_distances[-1] + distance(left_border[i-1], left_border[i]))
    
    right_distances = [0]
    for i in range(1, len(right_border)):
        right_distances.append(right_distances[-1] + distance(right_border[i-1], right_border[i]))
    
    # Normalize distances to [0,1]
    left_norm = np.array(left_distances) / left_distances[-1]
    right_norm = np.array(right_distances) / right_distances[-1]
    
    # Create interpolation functions for both borders
    left_interp_x = interp1d(left_norm, left_border[:, 0], bounds_error=False, fill_value="extrapolate")
    left_interp_y = interp1d(left_norm, left_border[:, 1], bounds_error=False, fill_value="extrapolate")
    
    right_interp_x = interp1d(right_norm, right_border[:, 0], bounds_error=False, fill_value="extrapolate")
    right_interp_y = interp1d(right_norm, right_border[:, 1], bounds_error=False, fill_value="extrapolate")
    
    # Generate center path
    center_path = []
    for i in range(num_points):
        t = i / (num_points - 1)
        left_x = left_interp_x(t)
        left_y = left_interp_y(t)
        right_x = right_interp_x(t)
        right_y = right_interp_y(t)
        
        # Center point is the average of corresponding points on left and right borders
        center_path.append([(left_x + right_x) / 2, (left_y + right_y) / 2])
    
    return center_path

def is_near_bridge(point, bridges, threshold):
    """Check if a point is close to any bridge pier."""
    for bridge in bridges:
        distances = cdist([point], bridge)
        if np.min(distances) < threshold:
            return True, np.argmin(distances)
    return False, None

def avoid_bridges(path, bridges, left_border, right_border, avoidance_distance=300):
    """
    Modify the path to avoid bridges.
    
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
                
                # Choose the safer side (farther from pier)
                if dist_left > dist_right:
                    # Move 70% toward left border
                    safe_point = [
                        point[0] + 0.7 * (left_point[0] - point[0]),
                        point[1] + 0.7 * (left_point[1] - point[1])
                    ]
                else:
                    # Move 70% toward right border
                    safe_point = [
                        point[0] + 0.7 * (right_point[0] - point[0]),
                        point[1] + 0.7 * (right_point[1] - point[1])
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

def plot_results(pier_positions, left_border, right_border, path):
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
    
    # Plot generated path
    path = np.array(path)
    plt.plot(path[:, 0], path[:, 1], 'g-', linewidth=2, label='Ship Path')
    
    # Mark start and end points
    plt.scatter(path[0, 0], path[0, 1], color='green', s=100, marker='^', label='Start')
    plt.scatter(path[-1, 0], path[-1, 1], color='red', s=100, marker='v', label='End')
    
    plt.legend()
    plt.title('Ship Route Planning')
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
    
    # Generate the ship path
    ship_path = generate_ship_path(pier_positions, left_border_points, right_border_points, 
                                  num_points=150, bridge_avoidance_distance=350)
    
    # Display the results
    plot_results(pier_positions, left_border_points, right_border_points, ship_path)
    
    # Print the first and last few waypoints
    print("First 5 waypoints:")
    for i in range(min(5, len(ship_path))):
        print(f"  [{ship_path[i][0]:.2f}, {ship_path[i][1]:.2f}]")
    
    print("\nLast 5 waypoints:")
    for i in range(max(0, len(ship_path)-5), len(ship_path)):
        print(f"  [{ship_path[i][0]:.2f}, {ship_path[i][1]:.2f}]")
    
    print(f"\nTotal number of waypoints: {len(ship_path)}")
    
    # Save the waypoints to a file
    np.savetxt('ship_waypoints.csv', ship_path, delimiter=',', header='x,y', comments='')
    print("Waypoints saved to 'ship_waypoints.csv'")