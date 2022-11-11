import heapq

class RoadGraph:
    def __init__(self, roads, cafes):
        """
        This in an init function for the road graph, initialise adj_lst_coffee, adj_lst_graph, adj_lst_graph_reverse,
        and calculating the list size.
        :Input:
        argv1: a list of tuple where each tuple contain (vertex,adjacent_vertex,weight) represent the the roads
        argv2: a tuple where each tuple contain (cafes waiting time) the waiting time for the cafe at that location
        :Output, return or postcondition: always init road graph, initialise adj_lst_coffee, adj_lst_graph,
        adj_lst_graph_reverse, and calculating the list size of a given roads & cafes
        :Time complexity: the highest complexity for this function will be O(E) where this funciton need to call
        create_graph function which have the highest complexity among all
                    -> where E is the number of edges in the graph
        :Aux space complexity: the highest space complexity among these function was the function call to create
        a graph -> which have the complexity of O(V+E) where V is the number of vertex in the graph ane E is the number
        of edges in the graph.
        """

        self.roads = roads
        self.cafes = cafes
        self.lst_size = self.lst_size_cal(self.roads)
        self.adj_lst_graph = self.create_graph(self.lst_size, roads, cafes)
        self.adj_lst_graph_reverse = self.create_graph(self.lst_size,self.flip(),self.cafes)
        self.adj_lst_coffee = self.create_adj_for_coffee(self.cafes, self.lst_size)
        self.q = []
        self.path = []

    def routing(self, start, end):
        """
        The function would then return the shortest route from the start location to the end location,
        going through at least 1 of the locations listed in cafes. The function will run Dijksra on the given start
        location in the graph, then run Dijksra on given end location of the reversed direction of the same graph, then
        sort out all the path from start -> all cafe and from end-> all cafe, sort out the shortest path after sum up
        the path from start -> all cafe and from end-> all cafe according by the cafe they visited, then print out that
        shortest path
        :Input:
        argv1:  start location
        argv2: end location
        argv3:  normal adj_lst of the graph
        argv4:  reversed adj_lst of the graph
        argv5: adj_lst of the caffe
        :Output, return or postcondition: return the shortest route from the start location to the end location,
        going through at least 1 of the locations listed in cafes.
        :Time complexity: running Dijksra on both of the graph is the highest complexity in this function, and the cost
        of running Dijksra is O(ElogV).
        Where E is the number of edges in the graph, and V is the number of vertex in the graph
        :Aux space complexity: running Dijksra on both of the graph is the highest complexity in this function,
        and the cost of running Dijksra is O(E+V).
        Where E is the number of edges in the graph, and V is the number of vertex in the graph
        """
        #init all the array
        dist = None
        pre = None
        dist_rev = None
        pre_rev = None

        #checking if the input is empty or not
        if self.lst_size == 0:
            return None

        # run Dijksra for both normal and reversed graph
        dist,pre = self.Dijkstra_algorithm(self.adj_lst_graph, start, self.lst_size)
        dist_rev,pre_rev = self.Dijkstra_algorithm(self.adj_lst_graph_reverse,end,self.lst_size)

        # adding the distance from start to all point then to all point to end
        for i in range(len(dist)):
            dist[i] = dist[i] + dist_rev[i]

        # adding the distance
        for i in range(len(dist)):
            if self.adj_lst_coffee[i] == []:
                dist[i] = 0
            else:
                dist[i] += self.adj_lst_coffee[i][0]

        # changing all the location not visisted to inf
        for i in range(len(dist)):
            if dist[i] == 0:
                dist[i] = float('inf')


        index = 0
        for i in range(len(dist)):
            if dist[index] > dist[i] and dist[i]!= 0:
                index = i
        # if there no path at all, return none
        if dist[index] == float('inf'):
            return None


        self.printPath(self.path,pre,index)
        self.path = self.path[::-1]
        self.path.append(index)
        self.printPath(self.path,pre_rev,index)
        if start == end :
            return self.path
        elif len(self.path) == 1:
            return None
        elif start not in self.path or end not in self.path:
            return None

        return self.path
    def create_graph(self, lst_size, roads, cafes):
        """
        create an adjacent list size N where N is the largest vertex/location in the road array, at each list[i]
        add a list of tuple (adjacent_vertex,weight/time to go there), each tuple will store the vertex that adjacent to list[i] and the
        weight of that edges
        :Input: a list of tuple where each tuple contain (vertex,adjacent_vertex,weight) represent location and road to the next location
        argv1: list of tuple of road
        argv2: the largest vertex inside the road array
        :Output, return or postcondition:   an adjacent list for graph represent the roads
        :Time complexity: append all the tuple present in the origin array into new adj_lst will cost O(E)
                        where E is the number of tuple/roads in the array
        :Aux space complexity: the highest space complexity of the function is O(V+E) because it will contain all the
                        vertex and edges in input roads array
        """
        #checking, if the input is empty or not
        if lst_size == 0:
            return 0
        # create a list of D size where D is the largest vertex
        adjacency_list = [[] for i in range(lst_size)]
        for i in range(len(roads)):
            adjacency_list[roads[i][0]].append([roads[i][1], roads[i][2]])

        return adjacency_list
    def create_adj_for_coffee(self, cafes, lst_size):
        """
        create an adjacent list size N where N is the largest vertex/location in the road graph, at each list[i]/index[i]
        add a tuple (cafes waiting time), each tuples at index[i] will store the waiting time for cafes at that location i
        and indicating there are a cafe exist in that location
        :Input: a tuple where each tuple contain (cafes waiting time) the waiting time for the cafe at that location
        argv1: list of tuple of cafes
        argv2: the largest location inside the road array
        :Output, return or postcondition:   an adjacent list for graph represent the cafes and waiting time
        :Time complexity: append all the tuple present in the origin array into new adj_lst will cost O(E)
                        where E is the number of tuple/cafes in the array
        :Aux space complexity: the highest space complexity of the function will be the size of the new adj_lst
                                which is O(V+E) where V is the number of caffe in the caffe array and E is the number
                                of waiting time at the caffe
        """
        #checking if the input is empty or not
        if lst_size == 0:
            return 0
        # creating the adj_list for the input
        adjacency_list = [[] for i in range(lst_size)]
        for i in range(len(cafes)):
            adjacency_list[cafes[i][0]].append(cafes[i][1])

        return adjacency_list
    def lst_size_cal(self, roads):
        """
        this function will find the largest vertex for the road array
        :Input: a list of tuple where each tuple contain (vertex,adjacent_vertex,weight) represent the the roads
        argv1:  list of tuple of roads
        :Output, return or postcondition: the return the required list size for the adj_list of the input list
        :Time complexity: going through the entire list and append this all the vertex present in the tuple to the new list
                            -> which will cost O(E) where E is number of tuples in the list
                            -> and find the largest vertex in the new list will also cost O(E)
                            -> ultimately the complexity for this function is O(E)
                            Where E is the nummber of edges
        :Aux space complexity: create a list containing all the vertex in each road -> O(E)
                                Where E is the nummber of edges
        """
        max_lst = []
        if len(roads) == 0:
            return 0
        # find the largest vertex
        for i in roads:
            max_lst.append(i[0])
            max_lst.append(i[1])
        lst_size = max(max_lst) + 1

        return lst_size
    def Dijkstra_algorithm(self, graph, starting_vertex, lst_size):
        """
        implementation of Dijksra algorithm, to find the shortest path from the given starting vertex
        to every single reachable location in the give graph
        :Input:
        argv1:  adj_list represent the graph that we want to run Dijkstra on
        argv2:  starting vertex that we want to use to find the path.
        argv3: lst_size of the graph/adj_lst
        :Output, return or postcondition: all the distance of all the shortest path from given starting vertex to all
        the location and all the path it will took to get there.
        :Time complexity: going through all the edges and vertex with the help of the adj_list
         in the worst case for Dijkstra will have the complexity of O(V+E) by just traveling through the graph.
         Then the cost of updating weighted/time after each edges you travel in the worst case, it mean that you have to
         perform E*logV operation for the entire graph because the complexity to updating in the heapq is Log(V)
         base on the size of heapq structure(but in this case it would be V), and relaxing/finalise all the the vertex
         by pop it out of the heapq cost V*logV
         where V is the number of vertex in the graph and E is the number of edges in the graph
         -> this will bring the complexity of Dijkstra to O(E*logV)
        :Aux space complexity: dist,pre will all have the space complexity of O(V+E) because the input of the graph
        which is the main cost of Aux space complexity in the

        """
        # create dist, pre lst
        dist = [float('inf') for i in range(lst_size)]
        pre = [None for i in range(lst_size)]
        dist[starting_vertex] = 0

        # init the starting locaiton at 0
        heapq.heappush(self.q, [dist[starting_vertex], starting_vertex])

        #run dijkstra
        while len(self.q) > 0:
            i = self.q
            _, u = heapq.heappop(self.q)
            for i in range(len(graph[u])):
                vertex = u
                edges = graph[u][i]
                self.relax(vertex,edges,dist,pre)
        return dist,pre
    def relax(self, vertex, edges, dist, pre):
        """
        a support function for Dijkstra, in this function we will updating the new dist to that vertex from
        the source if the new distance is shorter that the previous one and recorded the new path.
        :Input:
        argv1: vertex: the vertex we are currently on
        argv2: edges- will be the edges of that vertex reaching the adj_vertex
        argv3: dist array to update/record the dist
        argv4: pre- an array to update the next vertex predecessor.
        :Output, return or postcondition: the current shortest dist to all the vertex that reachable until this vertex
        of the graph.
        :Time complexity: cost of comparing distance will be O(1), cost of updating the new dist on the heapq will be
        Log(V) where V is number of vertex in the graph.
        -> O(logV)
        :Aux space complexity: O(V) which is the space complexity of dist and pre array.
                                Where V is the number of vertex in the graph.
        """

        if dist[edges[0]] > dist[vertex] + edges[1]:
            dist[edges[0]] = dist[vertex] + edges[1]
            pre[edges[0]] = vertex
            heapq.heappush(self.q, [dist[edges[0]], edges[0]])
    def flip(self):
        """
        this function will going to reverse the direction of each edges present in the roads array input.
        :Input:
        argv1:  roads array from the class
        :Output, return or postcondition: a list of road where the direction of the edges been reversed.
        :Time complexity: Going through every single edges will cost O(E) and change the start and end location of the
        road for all edges will also cost O(E)
        so the final complexity will be O(E) where E in the number of edges in the graph
        :Aux space complexity: create a new list to contain the edges for the newly reversed list
                               -> space complexity will be O(E) where E the number of edges
        """

        new_lst = []
        for i in range(len(self.roads)):
            new_lst.append([self.roads[i][1],self.roads[i][0],self.roads[i][2]])
        return new_lst
    def printPath(self,path, pre, e):
        """
        The function will print out the path from a given predecessor_list with the start vertex by continuously trace
        the previous vertex until reach a None where a None will indicate the path has already ended
        :Input: a path_list with predecessor_list and the starting vertex, where index(predecessor_list) will be see as
        a vertex and the value store in predecessor_list(index) will be there predecessor.
        argv1: a list to store the path
        argv2: predecessor_list where index(predecessor_list) will be see as a vertex and the value store
        in predecessor_list(index) will be there predecessor.
        argve3: e will be the vertex to start with
        :Output, return or postcondition: return a path of all the predecessor of the given vertex
        :Time complexity: the complexity of the graph is O(V) in the worst case where the path need to find contain all
        the vertex in the graph so you have to trace through the entire list
        ->V is the number of vertex in the graph which predecessor_list belong to
        :Aux space complexity: space complexity for the will be the size of the path_lst, where in the worst case path
        will contain all possible vertex -> O(V) where V is the number of vertex in the graph which
        predecessor_list belong to
        """
        if pre[e] == None:
            return self.path
        else:
            path.append(pre[e])
            self.printPath(self.path, pre, pre[e])

if __name__ == '__main__':
    roads = [(0, 1, 4), (1, 2, 2), (2, 3, 3), (3, 4, 1), (1, 5, 2),
             (5, 6, 5), (6, 3, 2), (6, 4, 3), (1, 7, 4), (7, 8, 2),
             (8, 7, 2), (7, 3, 2), (8, 0, 11), (4, 3, 1), (4, 8, 10)]
    cafes = [(5, 10), (6, 1), (7, 5), (0, 3), (8, 4)]
    a = RoadGraph(roads,cafes)
    b = a.routing(1, 1)
    print(b)