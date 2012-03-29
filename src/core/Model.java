package core;

import java.util.ArrayList;

import editor.Drawing;

public abstract class Model {
		
	private Time time; //Global time scale in model;	
	private PointsSet<MeshNode> modelNodes; // Global model nodes	
	private PointsSet<MeshNode> contactNodes; // Global model contact nodes
	private double coordinateTolerance = 1E-8; // Tolerance for element coordinates
	private double assembleDistance = 1E-6; // distance to assemble element meshes
	private ArrayList<FEMesh> meshes = null;
	private Drawing drawing = null;
	
	/**
	 * Default constructor creates global timescale;
	 */
	public Model() {
		time = new Time(0);		
	}
	
	/**
	 * @return global time scale in model;
	 */
	public Time getTime() {
		return time;
	}
	
	public PointsSet<MeshNode> getModelNodes() {
		return modelNodes;
	}

	public void setModelNodes(PointsSet<MeshNode> modelNodes) {
		this.modelNodes = modelNodes;
	}

	public PointsSet<MeshNode> getContactNodes() {
		return contactNodes;
	}

	public void setContactNodes(PointsSet<MeshNode> contactNodes) {
		this.contactNodes = contactNodes;
	}

	
	public double getCoordinateTolerance() {
		return coordinateTolerance;
	}

	public void setCoordinateTolerance(double coordinateTolerance) {
		this.coordinateTolerance = coordinateTolerance;
	}

	public double getAssembleDistance() {
		return assembleDistance;
	}

	public void setAssembleDistance(double assembleDistance) {
		this.assembleDistance = assembleDistance;
	}

	public abstract Drawing generateDrawing();
	
	public abstract ArrayList<FEMesh> generateMeshes();
	
	/**
	 * @return geometry of model
	 */
	public Drawing getDrawing() {
		if (drawing == null) drawing = generateDrawing();
		return drawing;
	}
	
	/**
	 * @return array of FE meshes of model
	 */
	public ArrayList<FEMesh> getFEMeshes() {
		if (meshes == null) meshes = generateMeshes();
		return meshes;
	}	
	
	
}
