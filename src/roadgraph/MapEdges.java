package roadgraph;

import geography.GeographicPoint;

public class MapEdges {
	
	GeographicPoint start;
	GeographicPoint end;
	private double distance;
	private String street_name;
	private String street_type;
	
	public MapEdges(GeographicPoint start, GeographicPoint end,
			String street_name, String street_type, double distance){
		
		this.start= start;
		this.end =  end;
		this.distance = distance;
		this.street_name = street_name;
		
	}
	
	public double getDistance()
	{
		return this.distance;
	}
	
	public void displayEdge(){
			System.out.println("Start Node : "+start.toString());
			System.out.println("End Node : "+end.toString());
			System.out.println("Distance : " +distance);
			System.out.println("Street Name  : "+street_name);
			System.out.println("Street Type  : "+street_type);
	}

	public String getRoadType()
	{
		return street_type;
	}
}
