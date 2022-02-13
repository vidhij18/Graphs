package roadgraph;

import geography.GeographicPoint;

public class NodeDistance {

	private GeographicPoint location;
	private double distance;
	
	public NodeDistance(GeographicPoint gp, double d){
		this.location = gp;
		this.distance = d;
		
	}
	
	public GeographicPoint getLocation()
	{
		return this.location;
	}
	
	public double getDistance()
	{
		return this.distance;
	}
}
