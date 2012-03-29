package core;

import java.util.HashSet;
import java.util.ArrayList;

/** Node class. Contains information about assembled node and all connected nodes and elements*/
public class MeshNode implements PointData {
		
	private Coordinate coordinate;
	private boolean numbered = false;
	private int number;		
	private ArrayList<MeshNode> connectedNodes = new ArrayList<MeshNode>();
	private ArrayList<Element> elements = new ArrayList<Element>();
	private HashSet<DOF> DOFs = new HashSet<DOF>();
	private double r;
	
	public MeshNode(Coordinate coordinate, int number) {
		this.coordinate = coordinate;
		this.number = number;					
	} 
	
	public boolean isNumbered() {
		return numbered;
	}

	public void setNumbered(boolean numbered) {
		this.numbered = numbered;
	}

	public int getNumber() {
		return number;
	}

	public void setNumber(int number) {
		this.number = number;
	}
	
	public HashSet<DOF> getDOFs() {
		return DOFs;
	}

	public void addConnectedNode(MeshNode node) {
		if (connectedNodes.contains(node)) return;		
		connectedNodes.add(node);
	}
	
	public void addElement(Element element) {
		if (elements.contains(element)) return;		
		elements.add(element);
	}
	
	public ArrayList<MeshNode> getConnectedNodes() {
		return connectedNodes;		
	}
	
	public ArrayList<Element> getElements() {
		return elements;		
	}

	@Override
	public Coordinate getCoordinate() {
		return coordinate;
	}

	@Override
	public double getR() {
		return r;
	}

	@Override
	public void setR(double r) {
		this.r = r;
	}
	
	@Override
	public String toString() {
		return number+"  "+coordinate.getX()
		+"  "+coordinate.getY()
		+"  "+coordinate.getZ()+"  "+hashCode();
	}

}

