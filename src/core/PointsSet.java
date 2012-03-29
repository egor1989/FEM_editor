package core;

import java.util.Arrays;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;


/** PointsSet class (analog of HashSet, but nodes are equal when distance 
 * between them less than assemblyDistance. In this case we can't write traditional hashCode() 
 * because r equals r+assemblyDistance, r+assemblyDistance equals r+2*assemblyDistance e.t.c., 
 * so they must have equal hashCode().    
 **/

public class PointsSet<T extends PointData> implements Iterable<T> {
	
	private class NodesList extends LinkedList<T> {
		private static final long serialVersionUID = 1L;
		
	}
	
	double assembleDistance;
	double maxR = 0, minR = Double.MAX_VALUE;	
	int tableSize;

	Object[] table;
	double[] maxRTable;
	int size = 0;
	
	private NodesList table(int i) {
		return (NodesList) table[i];
	}

	class NodesSetIterator implements Iterator<T> {
		int index = 0;
		Iterator<T> listIterator;

		NodesSetIterator() {
			listIterator = table(0).iterator();
		}

		@Override
		public boolean hasNext() {
			if (listIterator.hasNext()) return true;			
			for (int i = index + 1; i < tableSize; i++) {
				if (table(i).size() > 0) return true; 
			}
			return false;
		}

		@Override
		public T next() {
			if (listIterator.hasNext()) return listIterator.next();
			do {
				index++;
				if (index == tableSize) return null;
				listIterator = table(index).iterator();					
				if (listIterator.hasNext()) return listIterator.next();
			} while (true);								
		}

		@Override
		public void remove() {				
			//				Nothing to do
				throw new UnsupportedOperationException("Remove is not supported in NodesSet");
		}			
	}	

	public PointsSet(int tableSize, double assembleDistance) {
		this.tableSize = tableSize;
		this.assembleDistance = assembleDistance;
		table = new Object[tableSize];
		maxRTable = new double[tableSize];
		Arrays.fill(maxRTable, 0);
		for (int i = 0; i < tableSize; i++) {
			table[i] = new NodesList(); 				
		}
		maxR = 0;
		minR = Double.MAX_VALUE;		
	}
	
	private double calculateR(Coordinate c) {			
		double x = c.getX();
		double y = c.getY();
		double z = c.getZ();		
		return Math.sqrt(x*x+y*y+z*z);				
	}
	
	private double calculateR(T pointData) {
		Coordinate c = ((PointData)pointData).getCoordinate();					
		double r = calculateR(c);
		((PointData)pointData).setR(r);
		return r;
	}
	
	private boolean equal(T pointData1, T pointData2) {		
		return (distance(pointData1, pointData2) < assembleDistance);
	}
	
	private double distance(T pointData1, T pointData2) {
		Coordinate c1 = ((PointData)pointData1).getCoordinate();
		Coordinate c2 = ((PointData)pointData2).getCoordinate();
		return c1.distance(c2);
	}
	
	private double distance(T pointData1, Coordinate c) {
		Coordinate c1 = ((PointData)pointData1).getCoordinate();		
		return c1.distance(c);
	}
	
	public void calculateMaxMinR(Coordinate c) {
		double r = calculateR(c);
		if(r < minR) minR = r;
		if(r > maxR) maxR = r;		
	}
	
	public int size() {
		return size;
	}
	
	private double getR(T data) {
		return ((PointData)data).getR();
	}
		
	synchronized public T add(T pointData) {
		
		double r = calculateR(pointData);
		int index = (int)(tableSize * (r - minR)/(maxR - minR));
		if (index >= tableSize) index = tableSize-1;			

		if (index > 0) {
			if (table(index - 1).size() > 0) {
				T n = table(index - 1).getLast();
				if (equal(n, pointData)) return n;						
			}
		}			
		if (index < tableSize - 1) {
			if (table(index + 1).size() > 0) {
				T n = table(index + 1).getFirst();
				if (equal(n, pointData)) return n; 	
			}								
		}		
		if (r > maxRTable[index]) {
			if (table(index).size() > 0) {
				T n = table(index).getLast(); 
				if (equal(n, pointData)) return n;
			}
			table(index).addLast(pointData);
			maxRTable[index] = r;
			size++;			 			
			return pointData;
		}

		int i = 0;
		for (T n: table(index)) {			
			if (equal(n, pointData)) return n; 
			if (getR(n) > getR(pointData)) {					
				table(index).add(i, pointData);
				size++;				 			
				return pointData;					
			}
			i++;
		}
		table(index).add(pointData);
		size++;	
		return pointData;
	}

	@Override
	public Iterator<T> iterator() {			
		return new NodesSetIterator();
	}		
		
	
	public List<T> nearest(Coordinate c, double distance) {
		LinkedList<T> list = new LinkedList<T>();
		
		double r = calculateR(c);
		double dr = (maxR - minR)/tableSize;
		int index = (int)(tableSize * (r - minR)/(maxR - minR));
		if (index >= tableSize) index = tableSize-1;			
		for (T n : table(index)) {
			if (getR(n) < r - distance) continue;
			if (getR(n) > r + distance) break;
			if (distance(n,c) < distance) list.add(n); 			
		}
		
		for (int i = index - 1; i >= 0 && dr*(index-i) < distance; i--) {
			Iterator<T> it = table(i).descendingIterator(); 
			while (it.hasNext()) {
				T n = it.next(); 
				if (getR(n) < r - distance) break;				
				if (distance(n,c) < distance) list.add(n); 			
			}			
		}
		
		for (int i = index + 1; i < tableSize && dr*(i-index) < distance; i++) {
			for (T n : table(i)) {				
				if (getR(n) > r + distance) break;
				if (distance(n,c) < distance) list.add(n); 			
			}	
		}		
		return list;
	}
	
}

