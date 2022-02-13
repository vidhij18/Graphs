package roadgraph;
import java.util.Comparator;

public class DistanceComparator implements Comparator<NodeDistance>{

	@Override
	public int compare(NodeDistance x, NodeDistance y) {
		// TODO Auto-generated method stub
		
		if (x.getDistance() > y.getDistance())
        {
            return 1;
        }
        if (x.getDistance() < y.getDistance())
        {
            return -1;
        }
        return 0;
		
	}

}
