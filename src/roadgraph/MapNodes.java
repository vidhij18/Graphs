package roadgraph;
import geography.GeographicPoint;

import java.util.LinkedList;
import java.util.List;

public class MapNodes {

	private GeographicPoint location;
	private char dispChar;
	private List<MapEdges> neighbors = new LinkedList<>();
	
	public MapNodes(GeographicPoint location,char dispChar ){
		this.location = location;
		this.dispChar = dispChar;
	}
	
	public MapNodes(GeographicPoint location){
		this.location = location;
		this.dispChar = ' ';
	}
	
	public void setDispChar(char c){
		this.dispChar=c;
	}
	
	public char getDispChar(){
		return this.dispChar;
	}
	
	public void addNeighbors(MapEdges mn){
		neighbors.add(mn);
	}
	
	public List<MapEdges> getNeighbors(){
		return this.neighbors;
	}

	
	public GeographicPoint returnLocation(){
		return location;
	}
	
	public String toString(){
		return location.toString();
	}
}
