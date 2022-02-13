/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;
import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 2
	
	//using adjacency list for the graph representation
	public Map<GeographicPoint,MapNodes> adjList = new HashMap<>();
	
	public HashMap<GeographicPoint, Double> node = new HashMap<>(); //g(n)
	public HashMap<GeographicPoint, Double> estnode = new HashMap<>(); //g(n)
	
	int edges =0;
	int vertices =0;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 2
		//GraphLoader.createIntersectionsFile(this.file_name, S"data/intersections/simpletest.intersections");
		
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 2
		return vertices;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 2
			return adjList.keySet();
		
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 2
		return edges;
	}

	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 2
		vertices++;
		if(adjList.containsKey(location)){
			System.out.println("The Node Already exist");
		}
		else{
			MapNodes mp =new MapNodes(location);
			NodeDistance nd = new NodeDistance(location, 99999);
			node.put(location, (double) (99999));
			estnode.put(location, (double) (100));
			adjList.put(location, mp);
			return true;
		}
		return false;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		//TODO: Implement this method in WEEK 2
		edges++;
		MapEdges me = new MapEdges(from, to, roadName, roadType,length);
		adjList.get(from).addNeighbors(me);
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 2
		
		if (start == null || goal == null) {
			System.out.println("Start or goal node is null!  No path exists.");
			return new LinkedList<GeographicPoint>();
		}
		Queue<MapNodes> toExplore = new LinkedList<MapNodes>();
		
		HashSet<MapNodes> visited = new HashSet<MapNodes>();
		HashMap<MapNodes, MapNodes> parentMap = new HashMap<MapNodes, MapNodes>();
		toExplore.add(adjList.get(start));
		boolean found = false;
		
		while (!toExplore.isEmpty()) {
			MapNodes curr = toExplore.remove();
			if (curr == adjList.get(goal)) {
				found = true;
				break;
			}
			
			//System.out.println(curr.returnLocation().toString());
			List<MapEdges> neighbors = curr.getNeighbors();
			
			ListIterator<MapEdges> it = neighbors.listIterator(neighbors.size());
			while (it.hasPrevious()) {
				MapEdges next = it.previous();
				if (!visited.contains(adjList.get(next.end))) {
					//System.out.println(next.end.toString());
					visited.add(adjList.get(next.end));
					parentMap.put(adjList.get(next.end), curr);
					toExplore.add(adjList.get(next.end));
					nodeSearched.accept(next.end);
				}
			}
		}
		if (!found) {
			System.out.println("No path exists");
			return new ArrayList<GeographicPoint>();
		}
		
		
		// reconstruct the path
				LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
				MapNodes curr = adjList.get(goal);
				
				while (curr != adjList.get(start)) {
					//System.out.println(curr.toString());
					path.addFirst(curr.returnLocation());
					curr = parentMap.get(curr);
				}
				path.addFirst(start);
			return path;
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
	}
	

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	
	public List<GeographicPoint> dijkstra2(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		if (start == null || goal == null) {
			System.out.println("Start or goal node is null!  No path exists.");
			return new LinkedList<GeographicPoint>();
		}
		
		HashSet<MapNodes> visited = new HashSet<MapNodes>();
		HashMap<MapNodes, MapNodes> parentMap = new HashMap<MapNodes, MapNodes>();
		
		Comparator<NodeDistance> comparator = new DistanceComparator();
		PriorityQueue<NodeDistance> pqueue = new PriorityQueue<>(10, comparator);
		
		pqueue.add(new NodeDistance(start, 0));
		node.put(start, (double) 0);
		
		boolean found = false;
		
		while (!pqueue.isEmpty()) {
			NodeDistance curr = pqueue.poll();
			MapNodes current = adjList.get(curr.getLocation());
			
			
			if (current.returnLocation().equals(goal)) {
				found = true;
				break;
			}
			
			else if (!visited.contains(adjList.get(curr.getLocation()))) {
				visited.add(current);
				//System.out.println("Current : "+current.toString());
				//System.out.println("Distance : "+curr.getDistance() );
				List<MapEdges> neighbors = current.getNeighbors();
				double length = 0;
				
				ListIterator<MapEdges> it = neighbors.listIterator(neighbors.size());
				
					while (it.hasPrevious()) {
						MapEdges next = it.previous();
						
							//System.out.println(next.end.toString());
							
							//check if that particular node exist in a hash set
							//if not i add it
							//if yes, then i compare the distance
							
							length  = next.getDistance() + node.get(next.start) ;
							//+ next.start.distance(goal);
							//System.out.println(length);
							//System.out.println(node.get(next.end));
							if(length<(node.get(next.end)))
							{
								node.remove(next.end);
								node.put(next.end, length);
								
								parentMap.remove(adjList.get(next.end));
								parentMap.put(adjList.get(next.end), current);
								NodeDistance e = new NodeDistance(next.end, length);
								pqueue.add(e);
							}
							else 
							{
								
							}
							//toExplore.add(adjList.get(next.end));
							nodeSearched.accept(next.end);
						
					}
			}
		}	
		if (!found) {
			System.out.println("No path exists");
			return new ArrayList<GeographicPoint>();
		}
		
		
		// reconstruct the path
				LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
				MapNodes curr = adjList.get(goal);
				
				while (curr != adjList.get(start)) {
					//System.out.println(curr.toString());
					path.addFirst(curr.returnLocation());
					curr = parentMap.get(curr);
				}
				path.addFirst(start);
			return path;
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
	}

	public List<GeographicPoint> dijkstra2(GeographicPoint start, 
			  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
			{
			// TODO: Implement this method in WEEK 3
			
			if (start == null || goal == null) {
			System.out.println("Start or goal node is null!  No path exists.");
			return new LinkedList<GeographicPoint>();
			}
			
			HashSet<MapNodes> visited = new HashSet<MapNodes>();
			HashMap<MapNodes, MapNodes> parentMap = new HashMap<MapNodes, MapNodes>();
			
			Comparator<NodeDistance> comparator = new DistanceComparator();
			PriorityQueue<NodeDistance> pqueue = new PriorityQueue<>(10, comparator);
			
			pqueue.add(new NodeDistance(start, 0));
			node.put(start, (double) 0);
			
			boolean found = false;
			
			while (!pqueue.isEmpty()) {
			NodeDistance curr = pqueue.poll();
			MapNodes current = adjList.get(curr.getLocation());
			
			
			if (current.returnLocation().equals(goal)) {
			found = true;
			break;
			}
			
			else if (!visited.contains(adjList.get(curr.getLocation()))) {
			visited.add(current);
			//System.out.println("Current : "+current.toString());
			//System.out.println("Distance : "+curr.getDistance() );
			List<MapEdges> neighbors = current.getNeighbors();
			double length = 0;
			double currLength =0;
			ListIterator<MapEdges> it = neighbors.listIterator(neighbors.size());
			
			while (it.hasPrevious()) {
			MapEdges next = it.previous();
			
			//System.out.println(next.end.toString());
			
			//check if that particular node exist in a hash set
			//if not i add it
			//if yes, then i compare the distance
			
			length  = next.getDistance() + node.get(next.start) + next.getRoadType().length() ;
			currLength = node.get(next.end) + next.getRoadType().length();
			//+ next.start.distance(goal);
			//System.out.println(length);
			//System.out.println(node.get(next.end));
			if(length< currLength)
			{
				node.remove(next.end);
				node.put(next.end, length);
				
				parentMap.remove(adjList.get(next.end));
				parentMap.put(adjList.get(next.end), current);
				NodeDistance e = new NodeDistance(next.end, length);
				pqueue.add(e);
			}
			else 
			{
				
			}
			//toExplore.add(adjList.get(next.end));
			nodeSearched.accept(next.end);
			
			}
			}
			}	
			if (!found) {
			System.out.println("No path exists");
			return new ArrayList<GeographicPoint>();
			}
			
			
			// reconstruct the path
			LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
			MapNodes curr = adjList.get(goal);
			
			while (curr != adjList.get(start)) {
			//System.out.println(curr.toString());
			path.addFirst(curr.returnLocation());
			curr = parentMap.get(curr);
			}
			path.addFirst(start);
			return path;
			
			// Hook for visualization.  See writeup.
			//nodeSearched.accept(next.getLocation());
			
			}
			
				
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */

	
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		if (start == null || goal == null) {
			System.out.println("Start or goal node is null!  No path exists.");
			return new LinkedList<GeographicPoint>();
		}
		
		HashSet<MapNodes> visited = new HashSet<MapNodes>();
		HashMap<MapNodes, MapNodes> parentMap = new HashMap<MapNodes, MapNodes>();
		
		Comparator<NodeDistance> comparator = new DistanceComparator();
		PriorityQueue<NodeDistance> pqueue = new PriorityQueue<>(10, comparator);
		
		pqueue.add(new NodeDistance(start, 0));
		node.put(start, (double) 0);
		
		boolean found = false;
		
		while (!pqueue.isEmpty()) {
			NodeDistance curr = pqueue.poll();
			MapNodes current = adjList.get(curr.getLocation());
			
			
			if (current.returnLocation().equals(goal) ) {
				found = true;
				break;
			}
			
			else if (!visited.contains(adjList.get(curr.getLocation()))) {
				visited.add(current);
				//System.out.println("Current : "+current.toString());
				//System.out.println("Distance : "+curr.getDistance() );
				List<MapEdges> neighbors = current.getNeighbors();
				double length = 0;
				
				ListIterator<MapEdges> it = neighbors.listIterator(neighbors.size());
				
					while (it.hasPrevious()) {
						MapEdges next = it.previous();
						
							//System.out.println(next.end.toString());
							
							//check if that particular node exist in a hash set
							//if not i add it
							//if yes, then i compare the distance
							
							length  = next.getDistance() + node.get(next.start);
							//System.out.println(length);
							//System.out.println(node.get(next.end));
							if(length<node.get(next.end))
							{
								node.remove(next.end);
								node.put(next.end, length);
								
								parentMap.remove(adjList.get(next.end));
								parentMap.put(adjList.get(next.end), current);
								NodeDistance e = new NodeDistance(next.end, length);
								pqueue.add(e);
							}
							else 
							{
								
							}
							//toExplore.add(adjList.get(next.end));
							nodeSearched.accept(next.end);
						
					}
			}
		}	
		if (!found) {
			System.out.println("No path exists");
			return new ArrayList<GeographicPoint>();
		}
		
		
		// reconstruct the path
				LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
				MapNodes curr = adjList.get(goal);
				
				while (curr != adjList.get(start)) {
					//System.out.println(curr.toString());
					path.addFirst(curr.returnLocation());
					curr = parentMap.get(curr);
				}
				path.addFirst(start);
			return path;
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
	}

	public static void main(String[] args)
	{
		/*
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");
		
		System.out.println("Num of Nodes :" +theMap.getNumVertices());
		System.out.println("Number of Edges :" +theMap.getNumEdges());
		
		GeographicPoint start = new GeographicPoint(1.0,1.0);
		GeographicPoint goal = new GeographicPoint(8.0, -1.0);
		List<GeographicPoint> parent = theMap.bfs(start, goal);
		
		for(GeographicPoint ge : parent){
		
			System.out.println(ge.toString());
			
		}
	
		*/
		// You can use this method for testing.  
		
		// Use this code in Week 3 End of Week Quiz
		 MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		
		//GraphLoader.loadRoadMap("data/graders/mod3/map3.txt", theMap);
		
		System.out.println("DONE.");

		
		GeographicPoint start = new GeographicPoint(1.0,1.0);
		GeographicPoint goal = new GeographicPoint(8.0, -1.0);
		
		//List<GeographicPoint> route = theMap.dijkstra(start,goal);
		List<GeographicPoint> route = theMap.dijkstra(start, goal);
		//System.out.println("Displaying the Route");
		
		for(GeographicPoint ge : route){
			
			System.out.println(ge.toString());
			
		}
		/*
		

		*/
		
	}
	
}
