import time
import numpy as np
from pruning_testcases import test_pruning, time_pruning, profile_pruning

def are_three_points_collinear3(points):
    # Extract coordinates from the array
    x1, y1 = points[0]
    x2, y2 = points[1]
    x3, y3 = points[2]
    
    # Calculate the area using the determinant method
    determinant = x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)

    # Check if the area is zero
    return determinant == 0

def are_three_points_collinear_grid(points, epsilon=1e-6):
    # Extract coordinates
    x1, y1 = points[0]
    x2, y2 = points[1]
    x3, y3 = points[2]
    
    # Calculate differences
    dx1 = x2 - x1
    dy1 = y2 - y1
    dx2 = x3 - x2
    dy2 = y3 - y2
    
    # Check if the differences are consistent
    if dx1 == dy1 == dx2 == dy2:
        return True
    return dx1 * dy2 == dy1 * dx2

def collinearity_check(p1, p2, p3, epsilon=1e-6):   
    m = np.concatenate((p1, p2, p3), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon

def collinearity_check2(p1, p2, p3, epsilon=1e-6):   
    return are_three_points_collinear_grid([p1, p2, p3], epsilon)

def point(p):
    return np.array([p[0], p[1], 1.]).reshape(1, -1)

def prune(path):
    if path is not None:
        i = 0
        pruned_path = []
        pruned_path.append(path[0])
        while(i<len(path)):
            start = path[i]
            mid = start
            # print(f"add {start} start")
            j = i+1
                
            while j<len(path):
                end = path[j]
                if collinearity_check2(start,mid, end):
                    # print(f"Skip {mid} = col({start}, {mid}, {end})")
                    mid = end
                    j+=1
                else:
                    # print(f"add {mid} end")
                    pruned_path.append(mid)
                    i=j-1
                    break
            if j==len(path):
                pruned_path.append(path[-1])
                break
                
                
    else:
        pruned_path = path
        
    return pruned_path

def multipoint_colinearity(points, epsilon=1e-6):   
    # If there are less than 3 points, they are always collinear
    if len(points) < 3:
        return True
    if len(points) == 3:
        return are_three_points_collinear_grid(points, epsilon=epsilon)

    # Compute the direction vectors
    vectors = np.diff(points, axis=0)
    
    # Check the cross product of consecutive vectors
    cross_products = np.cross(vectors[:-1], vectors[1:])

    # If all cross products are zero, points are collinear
    return np.all(cross_products < epsilon)

def get_pruned_part(path, start_idx):
    # end_idx = start_idx + 2
    end_idx = len(path)
    while not multipoint_colinearity(path[start_idx:end_idx]):
        new_end_idx = (start_idx + end_idx) // 2
        if new_end_idx == end_idx:
            # didnt move so it's a bug
            print('Error: get_pruned_part did not move')
            return end_idx
        # if the length of the path is less than 10 then abort this loop
        # if end_idx - start_idx < 10:
        #     end_idx = start_idx + 1
        #     break

        end_idx = new_end_idx

    # now extend this part by one until it is not collinear
    while end_idx < len(path) and are_three_points_collinear_grid(path[end_idx-2:end_idx+1]):
        end_idx += 1

    if start_idx + 1 == end_idx:
        return end_idx
    return end_idx -1


def prune2(path):
    pruned_path = [path[0]]
    # extendedPath = np.array(path)
    start_idx = 0
    while start_idx < len(path) - 1:
        end_idx = get_pruned_part(path, start_idx)
        pruned_path.append(path[end_idx])
        start_idx = end_idx
    return pruned_path
    

def get_pruned_part3(path, start_idx):
    l = 2
    end_idx = start_idx + l
    last_colinearity_check = multipoint_colinearity(path[start_idx:end_idx])
    while last_colinearity_check:
        end_idx += l
        l += l
        if end_idx >= len(path):
            break
        last_colinearity_check = multipoint_colinearity(path[start_idx:end_idx])
    end_idx -= l//2
    if l == 4:
        return end_idx -1
    
    # now extend this part by one until it is not collinear
    while end_idx < len(path) and are_three_points_collinear3(path[end_idx-2:end_idx+1]):
        end_idx += 1

    if start_idx + 1 == end_idx:
        return end_idx
    return end_idx -1
def prune3(path):
    pruned_path = [path[0]]
    # extendedPath = np.array(path)
    start_idx = 0
    while start_idx < len(path) - 1:
        end_idx = get_pruned_part3(path, start_idx)
        pruned_path.append(path[end_idx])
        start_idx = end_idx
    return pruned_path

def prune_rec(path, start_idx, end_idx):
    span = end_idx - start_idx +1
    if span == 3:
        if are_three_points_collinear3(path[start_idx:end_idx+1]):
            return [path[start_idx], path[end_idx]]
        else:
            return path[start_idx:end_idx+1]

    mid = (start_idx + end_idx) // 2
    l_span = mid - start_idx +1
    r_span = end_idx - mid
    left = prune_rec(path, start_idx, mid) if l_span > 2 else path[start_idx:mid+1]
    right = prune_rec(path, mid+1, end_idx) if r_span > 2 else path[mid+1:end_idx+1]
    if len(left) > 1 and are_three_points_collinear3([left[-2], left[-1], right[0]]):
        left = left[:-1]
    if len(right) >1 and are_three_points_collinear3([left[-1], right[0], right[1]]):
        right = right[1:]
    return left + right

def prune4(path):
    return prune_rec(path, 0, len(path)-1)


if __name__ == "__main__":
    print("Test prune")
    test_pruning(prune)
    time_pruning(prune)
    profile_pruning(prune)

    # print("Test prune3")
    # test_pruning(prune3)
    # time_pruning(prune3)
    # profile_pruning(prune3)

    print("Test prune4")
    test_pruning(prune4)
    time_pruning(prune4)
    profile_pruning(prune4)