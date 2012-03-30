package models;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import Jama.Matrix;
import core.Coordinate;
import core.DOF;
import core.DoubleParameter;
import core.Element;
import core.ElementException;
import core.ElementNode;
import core.Identificator;
import core.Material;
import core.MeshNode;
import core.Parameter;
import core.Time;
import core.Vector;
import editor.Mesh;

public abstract class ContactFE extends AbstractFE {

	protected double distance = 0.01;
	protected double factor = 0.01;
	protected LinkedList<MeshNode> ownNodes;
	
	
	public ContactFE(int intPointsNumber, int order, Material material, Time t) {
		super(intPointsNumber, order, material, t);
		// TODO Auto-generated constructor stub
	}

	@Override
	public ArrayList<Parameter> parameters() {
		ArrayList<Parameter> parameters = super.parameters();				
		parameters.add(new DoubleParameter("Factor", factor, 0, Double.MAX_VALUE));
		parameters.add(new DoubleParameter("Distance", distance, 0, Double.MAX_VALUE));		
		return parameters;
	}
	
	protected List<MeshNode> findNearestNodes() {
		if (nodes == null) throw new ElementException("Nodes of element were not defined in element " + this.getClass().getName() + 
		". Put nodes definition in constructor of element");
		Coordinate middle;
		double searchDistance;
		if (nodes.length == 1) {
			middle = nodes[0].getCoordinate();
			searchDistance = distance;
		}
		else {
			middle = new Coordinate(0,0,0);
			for (ElementNode n : nodes) {
				middle.plusEquals(n.getCoordinate());			
			}
			middle.multEquals(1/nodes.length);
			double rmax = 0;
			for (ElementNode n : nodes) {
				double r = middle.distance(n.getCoordinate());
				if (r > rmax) rmax = r;
			}	
			searchDistance = rmax + distance;
		}
		List<MeshNode> list = model.getContactNodes().nearest(middle, searchDistance);		
		return deleteOwnNodes(list);		
	}
	
	private List<MeshNode> deleteOwnNodes(List<MeshNode> nearest) {
		LinkedList<MeshNode> toRemove = new LinkedList<MeshNode>(); 
		for (MeshNode n : nearest) {
			for (Element e : n.getElements()) {
				if (e == this) {
					toRemove.add(n);
					break;
				}
			}
		}
		ownNodes = toRemove;
		nearest.removeAll(toRemove);
		return nearest;
	}
	
	public List<MeshNode> contactToPoint(Coordinate c) {
		return model.getContactNodes().nearest(c, distance);
		
	}
	
	protected class ContactNode {
		MeshNode node;
		double r;		
		double xi;
		Vector p;
		
		public ContactNode(MeshNode n, double r, double xi, Vector p) {
			this.node = n;
			this.r = r;
			this.xi = xi;
			this.p = p;
		}		
	}

	/**
	 * Creates the list of contact nodes;
	 * ab - contact line;
	 * c - contact node;
	 * d - point on line ab (perpendicular from c);
	 * xi - local coordinate
	 * p - perpendicular to ab
	 */
	public List<ContactNode> contactToLine(Coordinate a, Coordinate b) {
		List<ContactNode> nearest = new LinkedList<ContactNode>();
		Vector ab = new Vector(a,b);		
		for (MeshNode n : findNearestNodes()) {
			Coordinate c = n.getCoordinate();
			Vector ac = new Vector(a,c);
			Vector normal = Vector.crossProduct(ab, ac); // normal to plain (ab,ac)
			double r = normal.length()/ab.length(); // distance from c to ab
			if (r >= distance) continue;
			
			Vector p = Vector.crossProduct(ab, normal); //  perpendicular to ab
			p.multEquals(1/p.length()); //normalize it
			Vector ad = ac.minus(p.mult(r));
			double xi = ad.length()/ab.length(); // local coordinate of contact point;
			nearest.add(new ContactNode(n, r, xi, p));
		}
		return nearest;	
	}
	
	
	@Override
	public Mesh draw(Identificator resultId) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public ElementCoordinates getElementCoordinates() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public LocalCoordinates getLocalCoordinates() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public DOF[] getNMatrixMembers() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public Matrix getN(Coordinate c) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public Identificator[] getBMatrixMembers() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public Matrix getB(Coordinate c) {
		// TODO Auto-generated method stub
		return null;
	}

}
