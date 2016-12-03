import copy

class Waypoint():
    def __init__(self, location, neighbors):
        """
            Initialize a waypoint object.
            
            Args:
                location: An (x,y) floating point tuple specifying the waypoint's 
                    offset from the origin.
                neighbors: An character list specifying the waypoint's neighbors in 
                    the graph as their ids.
        """
        self.location = location
        self.neighbors = set(neighbors)
        self.visited = False

class FloorPlan():
    def __init__(self, point_ids, locations, neighbors, landmarks):
        """
            Instantiate a FloorPlan object.
            
            Args:
                point_ids: A set containing a unique identifier for each waypoint in the graph.
                locations: A dictionary mapping point_ids to tuples representing locations.
                neighbors: A dictionary mapping point_ids to lists containing other point_ids
                    representing the current node's neighbors.
                landmarks: A dictionary mapping room numbers to point_ids.
        """
        
        self.landmarks = set(landmarks)
        self.graph = {}
        for point_id in point_ids:
            self.graph[point_id] = Waypoint(locations[point_id], neighbors[point_id])
        
    
    def dist2(self, point1, point2):
        """Return the distance squared between a tuple representing a location and a waypoint."""
        return (point1[0] - point2[0])**2 + (point1[1] - point2[1])**2
    
    def get_closest(self, point):
        """Return the closest waypoint to the given position."""
        return min(self.graph, key = lambda k: self.dist2(point, self.graph[k].location))
    
    def reset(self):
        for key in self.graph:
            self.graph[key].visited = True
    
    def get_path(self, cur_pos, destination):
        """
            Compute the shortest path from the current position to the 
                destination (Djikstra's Algorithm).
        
        
        """
        
        # get our start and end positions
        start = self.get_closest(cur_pos)
        dest = self.get_closest(destination)
        
        # create vertex set, distance list, and path
        Q = copy.deepcopy(self.graph)
        prev = {}
        dist = {}
        for point in self.graph:
            prev[point] = None
            dist[point] = float('inf')
        
        dist[start] = 0
        while Q:
            # get the closest waypoint off the queue
            cur_id = min(Q, key = lambda k: dist[k])
            cur_waypoint = Q.pop(cur_id)
            
            # if we've made it to our destination, we're done
            if cur_id == dest:
                break
            
            # update minmum path lengths
            for neighbor in cur_waypoint.neighbors:
                test_dist = dist[cur_id] + self.dist2(self.graph[neighbor].location, cur_waypoint.location)
                if test_dist < dist[neighbor]:
                    dist[neighbor] = test_dist
                    prev[neighbor] = cur_id
                
        # back out the actual path
        crawler = dest
        path = []
        while True:
            path.insert(0,crawler)
            if prev[crawler] is not None:
                crawler = prev[crawler]
            else:
                # we've reached the start of our search
                return path
        
if __name__ == "__main__":
    import MD2
    mygraph = FloorPlan(MD2.points, MD2.locations, MD2.neighbors, MD2.rooms)
    print(mygraph.get_path((-66,8), (-48, 8)))
    print(mygraph.get_closest((200,2)))