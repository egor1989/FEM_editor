package core;


public interface PointData {
	
	public Coordinate getCoordinate();
	
	// distance from some point for ordering purposes
	public double getR();
	public void setR(double r);
}
