package core;

public class Vector extends Coordinate {

	public Vector(double x, double y, double z) {		
		super(x, y, z);
	}
	
	public Vector(Coordinate a) {
		super(a);
	}
	
	/**
	 * Vector from coordinate a to b
	 * @param a
	 * @param b
	 */
	public Vector(Coordinate a, Coordinate b) {
		super(b);
		minusEquals(a);		
	}
	
	public static Vector crossProduct(Vector a, Vector b) {
		return new Vector(a.y*b.z-a.z*b.y, 
			              a.z*b.x-a.x*b.z,
			              a.x*b.y-a.y*b.x);	
	}
	
	public double length() {
		return Math.sqrt(x*x+y*y+z*z);
	}
	
	public Vector plus(Vector a) {
		return new Vector(x+a.x, y+a.y, z+a.z);				
	}
	
	public Vector minus(Vector a) {
		return new Vector(x-a.x, y-a.y, z-a.z);				
	}
	
	public Vector mult(Vector a) {
		return new Vector(x*a.x, y*a.y, z*a.z);				
	}
	
	public Vector mult(double a) {
		return new Vector(x*a, y*a, z*a);				
	}
	
	

}
