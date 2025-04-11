from typing import List, Tuple
import numpy as np
import math


def calculate_distance(point1: Tuple[float, float], point2: Tuple[float, float]) -> float:
    """Calculate the Euclidean distance between two points."""
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

def closest_points(point_list: List[List[Tuple[float, float]]], dynamic_point: Tuple[float, float]) -> List[Tuple[float, float]]:
    """Find the two closest points from a list to a given point."""
    flattened_points = [point for sublist in point_list for point in sublist]
    distances = [np.linalg.norm(np.array(point) - np.array(dynamic_point)) for point in flattened_points]
    closest_indices = np.argsort(distances)[:2]
    closest_indices.sort()
    return [flattened_points[i] for i in closest_indices]

def remove_duplicates(point_list: List[List[Tuple[float, float]]]) -> List[List[Tuple[float, float]]]:
    """Remove duplicate points while preserving sublist structure."""
    unique_points = set()
    new_list = []
    for sublist in point_list:
        new_sublist = []
        for point in sublist:
            if point not in unique_points:
                unique_points.add(point)
                new_sublist.append(point)
        if new_sublist:
            new_list.append(new_sublist)
    return new_list

def remove_before_point(point_list: List[List[Tuple[float, float]]], search_point: Tuple[float, float]) -> List[List[Tuple[float, float]]]:
    """Remove all points before a given point in the list."""
    found = False
    new_list = []
    for point in point_list:
        if found:
            new_list.append(point)
        else:
            if point == search_point:
                found = True
                new_list.append(point)
    return new_list

def flatten_and_remove_duplicates(point_list: List[List[Tuple[float, float]]]) -> List[Tuple[float, float]]:
    """Flatten a list of lists of points and remove duplicates."""
    unique_points = set()
    flattened_points = []
    for sublist in point_list:
        for point in sublist:
            if point not in unique_points:
                unique_points.add(point)
                flattened_points.append(point)
    return flattened_points

def closest_segment_point_ahead(points: List[Tuple[float, float]], position: Tuple[float, float]) -> Tuple[float, float]:
    """Find the closest point ahead on a path segment."""
    min_distance = float('inf')
    point_ahead = None
    for i in range(len(points) - 1):
        p1, p2 = np.array(points[i]), np.array(points[i + 1])
        p = np.array(position)
        line_vec = p2 - p1
        point_vec = p - p1
        line_len = np.linalg.norm(line_vec)
        line_unitvec = line_vec / line_len
        point_vec_scaled = point_vec / line_len
        t = np.dot(line_unitvec, point_vec_scaled)    
        t = max(0.0, min(1.0, t))
        nearest = line_vec * t
        dist = np.linalg.norm(point_vec - nearest)
        if dist < min_distance:
            min_distance = dist
            point_ahead = (p2[0], p2[1])
    return point_ahead

def find_point_at_distance(point_list: List[Tuple[float, float]], distance: float) -> Tuple[float, float]:
    """Find a point at a specific distance along a path."""
    running_total = 0
    prev_point = point_list[0]
    for point in point_list[1:]:
        segment_length = np.linalg.norm(np.array(point) - np.array(prev_point))
        if running_total + segment_length >= distance:
            excess_distance = distance - running_total
            direction_vector = np.array(point) - np.array(prev_point)
            unit_vector = direction_vector / np.linalg.norm(direction_vector)
            exact_point = np.array(prev_point) + unit_vector * excess_distance
            return tuple(exact_point)
        running_total += segment_length
        prev_point = point
    return point_list[-1]

def find_point_ahead_on_path(path: List[List[Tuple[float, float]]], current_position: Tuple[float, float], distance_ahead: float) -> Tuple[float, float]:
    """Find a point on the path that is a certain distance ahead of the current position."""
    path = flatten_and_remove_duplicates(path)
    segment_point_ahead = closest_segment_point_ahead(path, current_position)
    updated_path = remove_before_point(path, segment_point_ahead)
    updated_path.insert(0, current_position)
    return find_point_at_distance(updated_path, distance_ahead) 

