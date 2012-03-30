package models.mechanics.rods;

import Jama.Matrix;
import core.Coordinate;
import core.DOF;
import core.ElementNode;
import core.Identificator;
import core.Material;
import core.Time;
import editor.Mesh;
import models.AbstractFE;
import models.ElementCoordinates;
import models.LCoordinates;
import models.LocalCoordinates;
import models.mechanics.DOFs;
import models.mechanics.InternalIdents;
import models.mechanics.ResultIdents;

public class Rod extends AbstractFE {

	private Coordinate[] coordinates;
	private LCoordinates L;
	private Matrix B;
	
	protected static DOF[] dofs = {DOFs.displacementX,DOFs.displacementY, DOFs.displacementZ, 
		                           DOFs.rotationAroundX, DOFs.rotationAroundY, DOFs.rotationAroundZ};
	
	private static Identificator[] BMatrixMembers = {InternalIdents.epsilonX, InternalIdents.epsilonY, InternalIdents.gammaXY};
	
	public Rod(Coordinate[] coordinates, Material material, Time t) {
		super(3, 3, material, t);
		this.coordinates = coordinates; 
		nodes = new ElementNode[] {
				new ElementNode(coordinates[0], dofs),
				new ElementNode(coordinates[1], dofs),				
		};
				
	}

	protected Coordinate[] getDeformedCoordinates() {
		double[] x = solutionData.getX();		
		Coordinate[] coords = new Coordinate[nodes.length];
		for (int i = 0; i < nodes.length; i++) {
			coords[i] = nodes[i].getCoordinate();
			if (ResultIdents.deformedMeshScale != 0) {
				Coordinate deformed = new Coordinate(x[i*6+0], x[i*6+1], x[i*6+2]);
				deformed.multEquals(ResultIdents.deformedMeshScale);
				coords[i].plusEquals(deformed);
			}
		}		
		return coords;
	}
	
	protected void setResults(Mesh m, Identificator resultId, int[] points) {
		double[] x = solutionData.getX();
		for (int dof = 0; dof < 6; dof++) {
			if (resultId == dofs[0]) {
				for (int i = 0; i < points.length; i++) {
					m.setResult(points[i], x[i*2+dof]);
				}			
				return;
			}	
		}
		if (resultId == ResultIdents.maxDisplacement) {
			for (int i =0; i < points.length; i++) {
				double d = Math.sqrt(x[i*2+0]*x[i*2+0]+x[i*2+1]*x[i*2+1]+x[i*2+2]*x[i*2+2]);
				m.setResult(points[i], d);
			}			
			return;			
		}					
		return;		
	}
	
	@Override
	public Mesh draw(Identificator resultId) {
		Mesh m = new Mesh();
		Coordinate[] coords = getDeformedCoordinates();
		int[] points = { m.point(coords[0]), m.point(coords[1])};
		setResults(m,resultId,points);
		m.setLineWidth(3);
		m.line(points[0], points[1]);
		return null;
	}

	@Override
	public ElementCoordinates getElementCoordinates() {
		return new LCoordinates(coordinates);
	}

	@Override
	public LocalCoordinates getLocalCoordinates() {		
		return null;
	}

	@Override
	public DOF[] getNMatrixMembers() {
		return dofs;
	}

	@Override
	public Matrix getN(Coordinate c) {		
		return null;
	}

	@Override
	public Identificator[] getBMatrixMembers() {
		return BMatrixMembers;
	}

	@Override
	public Matrix getB(Coordinate c) {
		return null;
	}

}
