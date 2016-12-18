import copy

class Waypoint():
    def __init__(self, location, neighbors, to_meters):
        """
            Initialize a waypoint object.
            
            Args:
                location: An (x,y) floating point tuple specifying the waypoint's 
                    offset from the origin.
                neighbors: An character list specifying the waypoint's neighbors in 
                    the graph as their ids.
        """
        self.location = location if not to_meters else tuple(p * .3048 for p in location)
        self.neighbors = set(neighbors)

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
            self.graph[point_id] = Waypoint(locations[point_id], neighbors[point_id], True)
        
    
    def dist2(self, point1, point2):
        """Return the distance squared between a tuple representing a location and a waypoint."""
        return (point1[0] - point2[0])**2 + (point1[1] - point2[1])**2
    
    def get_closest(self, point):
        """Return the closest waypoint to the given position."""
        return min(self.graph, key = lambda k: self.dist2(point, self.graph[k].location))
    
    def get_path(self, cur_pos, destination):
        """
            Compute the shortest path from the current position to the 
                destination (Djikstra's Algorithm). Note: only guaranteed to work for acyclic graphs.
                
            Args:
                cur_pos: An (x,y) float tuple representing the current position relative
                    to the origin.
                destination: An (x,y) float tuple representing the desired position relative
                    to the origin.
            
            Returns:
                A path of tuples representing the desired path to the destination waypoint.
        """
        
        # get our start and end positions
        start = self.get_closest(cur_pos)
        dest = self.get_closest(destination)
        
        # create vertex set, distance list, and prev path
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
            # add the current point to the path at the begining
            path.insert(0,self.graph[crawler].location)
            
            # we've reached our start position
            if prev[crawler] is None:
                return path
            
            # keep moving back through the path
            crawler = prev[crawler]
        
if __name__ == "__main__":
    import MD2
    mygraph = FloorPlan(MD2.points, MD2.locations, MD2.neighbors, MD2.rooms)
    print(mygraph.get_path((0,0), (-2.1336, 1.2192)))
    print(mygraph.get_path((-10,-10), (0, .3)))
    print(mygraph.get_path((-2.088009210316669, 1.1881645720898277),(-10,-10)))
    print(mygraph.get_closest((200,2)))