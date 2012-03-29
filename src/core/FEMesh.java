package core;

import java.util.ArrayList;

public interface FEMesh {
//	public ArrayList<Coordinate> getNodes();
	public ArrayList<Element> getElements();
//	public ArrayList<int[]> getElementNodeNumbers();
	public ArrayList<Element> getContactElements();
//	public ArrayList<int[]> getContactElementNodeNumbers();
	
}
