package editor;

import java.awt.Color;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;

import javax.media.j3d.Appearance;
import javax.media.j3d.Bounds;
import javax.media.j3d.BranchGroup;
import javax.media.j3d.ColoringAttributes;
import javax.media.j3d.GeometryArray;
import javax.media.j3d.ImageComponent2D;
import javax.media.j3d.IndexedGeometryArray;
import javax.media.j3d.LineAttributes;
import javax.media.j3d.LineStripArray;
import javax.media.j3d.Material;
import javax.media.j3d.Node;
import javax.media.j3d.PolygonAttributes;
import javax.media.j3d.Shape3D;
import javax.media.j3d.TexCoordGeneration;
import javax.media.j3d.Texture;
import javax.media.j3d.Texture2D;
import javax.media.j3d.TextureAttributes;
import javax.media.j3d.TransparencyAttributes;
import javax.vecmath.Color3f;
import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;

import org.jcae.opencascade.jni.*;

import com.sun.j3d.utils.geometry.GeometryInfo;
import com.sun.j3d.utils.geometry.NormalGenerator;
import com.sun.j3d.utils.geometry.Stripifier;
import com.sun.j3d.utils.image.TextureLoader;

/**
 * @author Constantin Shashkin
 * In this class there was used a part of code by Jerome Robert (from Viewer3D class).
 */

public class Shape implements Visible {
	
	/**
	 * Internal class or 3D mesh of OCC objects.
	 */
	public class FaceMesh
	{
		private float[] nodes;
		private int[] mesh;
				
		public FaceMesh(float[] nodes, int[] mesh)	{
			if(nodes.length%3!=0 || mesh.length%3!=0)
				throw new IllegalArgumentException();
			this.nodes = nodes;
			this.mesh = mesh;
		}
	
		public float[] getNodes() {
			return nodes;
		}
		
		public int[] getMesh() {
			return mesh;
		}
	}
	
	protected int type;
	
	static final int EDGE = 1, FACE = 2, SOLID = 3, COMPOUND = 4, MESH = 5;
	
	protected TopoDS_Shape shape = null;  //OCC base shape. Can be null if we have MESH type 
	private ArrayList<FaceMesh> faceMeshes; // face meshes of OCC shape
	private ArrayList<float[]> edgeArrays; // edge arrays of OCC shape
	private BranchGroup facesNode = null, edgesNode = null, textNode = null; // Java3D nodes of faces edges and text
	private ArrayList<Shape3D> faces;  // shape faces, edges and text members
	private ArrayList<Shape3D> edges;
	private ArrayList<Shape3D> text;
	private boolean selected = false; // selected state of shape
	protected boolean cutted = false; // used by logical operations
	private boolean drawMesh = false; // true if we draw FE mesh instead of shape
	private Mesh mesh = null; // FE mesh
	
	private double meshSize = 1; // gloobal mesh size in shape
	
	/**
	 * Internal class for saving vertex mesh info
	 */
	private class Vertex {
		TopoDS_Vertex vertex;
		double size;
		
		Vertex(TopoDS_Vertex v) {
			vertex = v;
			size = meshSize;			
		}
		
		double[] getXYZ() {
			return BRep_Tool.pnt(vertex);
		}
		
		double distance(double x, double y, double z) {
			double[] xyz = getXYZ();
			return Math.sqrt((xyz[0]-x)*(xyz[0]-x)+(xyz[1]-y)*(xyz[1]-y)+(xyz[2]-z)*(xyz[2]-z));			
		}
	}
	
	/**
	 * Internal class for saving edge mesh info
	 */
	private class Edge {
		TopoDS_Edge edge;
		int numElements;
		
		Edge(TopoDS_Edge e) {
			edge = e;			
			numElements = -1;
		}
		
		
		double distance(double x, double y, double z) {
			double[] range = BRep_Tool.range(edge);			
			Geom_Curve curve = BRep_Tool.curve(edge, range);			
			GeomAdaptor_Curve ga = new GeomAdaptor_Curve(curve);
			double minR = Double.MAX_VALUE; 
			for (double u = range[0]; u <= range[1]; u+=(range[1]-range[0])/10) {
				double[] xyz = ga.value(u);
				double r = Math.sqrt((xyz[0]-x)*(xyz[0]-x)+(xyz[1]-y)*(xyz[1]-y)+(xyz[2]-z)*(xyz[2]-z));
				if (r<minR) minR = r;
			}
			return minR;						
		}
		
		
	}
	
	/**
	 * Storage for vertex and edge mesh info (without any special information = null)
	 */
	private ArrayList<Vertex> vertexMeshSize = null;
	private ArrayList<Edge> edgeMeshSize = null;
	
	VisualSettings vs;
		
	public Shape(int type, TopoDS_Shape shape, VisualSettings vs) {
		this.type = type;
		this.vs = new VisualSettings(vs);		
		setShape(shape);
	}
	
	public void setShape(TopoDS_Shape shape) {
		vertexMeshSize = null;
		edgeMeshSize = null;
		this.shape = shape;
		createFaceMeshes();
		createFaces();
		createEdgeArrays();
		createEdges();
	}
	

	public Shape(int type, Mesh mesh, VisualSettings vs) {
		this.type = type;		
		this.vs = new VisualSettings(vs);
		this.mesh = mesh;
		mesh.create();
		drawMesh = true;		
	}
	
	public void setMesh(Mesh m) {		
		mesh = m;
		mesh.create();
		drawMesh();
	}
	
	public Mesh getMesh() {
		return mesh;
	}
	
	public void drawMesh() {
		if (mesh == null) return;
		drawMesh = true;		
	}
	
	public void drawShape() {
		if (shape == null) return;
		drawMesh = false;		
	}
	
	public void setVisualSettings(VisualSettings vs) {
		this.vs = vs; 
	}
	
	public VisualSettings getVisualSettings() {
		return vs; 
	}
		
	public int getType() {
		return type;
	}
	
	private void createFaceMeshes() {	
		int meshIter = 3;
		TopExp_Explorer explorer = new TopExp_Explorer();
		TopLoc_Location loc = new TopLoc_Location();
		faceMeshes=new ArrayList<FaceMesh>();
				
		for (explorer.init(shape, TopAbs_ShapeEnum.FACE); explorer.more(); explorer.next())
		{						
			TopoDS_Shape s = explorer.current();
			if (!(s instanceof TopoDS_Face)) continue; // should not happen!
			TopoDS_Face face = (TopoDS_Face)s;
			Poly_Triangulation pt = BRep_Tool.triangulation(face,loc);
			
			float error=0.001f*getMaxBound(s)*4;
			//float error=0.0001f;
			int iter=0;
			while((pt==null)&(iter<meshIter)){
				new BRepMesh_IncrementalMesh(face,error, false);
				//new BRepMesh_IncrementalMesh(face,error, true);
				pt = BRep_Tool.triangulation(face,loc);				
				error/=10;
				iter++;
			}
						
			if (pt==null)
			{
				System.err.println("Triangulation failed for face "+face+". Trying other mesh parameters.");
				faceMeshes.add(new FaceMesh(new float[0], new int[0]));
				continue;
	
			}		
			
			double[] dnodes = pt.nodes();
			final int[] itriangles = pt.triangles();						
			if(face.orientation()==TopAbs_Orientation.REVERSED)
			{
				reverseMesh(itriangles);
			}
			

			final float[] fnodes=new float[dnodes.length];			
			
			if(loc.isIdentity())
			{
				for(int i=0; i<dnodes.length; i++)
				{
					fnodes[i]=(float) dnodes[i];
				}				
			}
			else
				transformMesh(loc, dnodes, fnodes);
				
			faceMeshes.add(new FaceMesh(fnodes, itriangles));
		}
	}
	
	/**
	 * Compute the bounding box of the shape and
	 * return the maximum bound value
	 * @param shape
	 * @return
	 */
	private static float getMaxBound(TopoDS_Shape shape){
		Bnd_Box box = new Bnd_Box(); 
		BRepBndLib.add(shape,box);
		double[] bbox = box.get();
		double minBoundingBox=
			Math.max(Math.max(bbox[3]-bbox[0], bbox[4]-bbox[1]), bbox[5]-bbox[2]);
		return (float)minBoundingBox;
	}
	
	private void transformMesh(TopLoc_Location loc, double[] src, float[] dst)
	{
		double[] matrix=new double[16];
		loc.transformation().getValues(matrix);
		Matrix4d m4d=new Matrix4d(matrix);
		Point3d p3d=new Point3d();
		for(int i=0; i<src.length; i+=3)
		{
			p3d.x=src[i+0];
			p3d.y=src[i+1];
			p3d.z=src[i+2];
			m4d.transform(p3d);
			dst[i+0]=(float) p3d.x;
			dst[i+1]=(float) p3d.y;
			dst[i+2]=(float) p3d.z;
		}		
	}
	
	/**
	 * @param itriangles
	 */
	static private void reverseMesh(int[] itriangles)
	{
		int tmp;
		for(int i=0; i<itriangles.length; i+=3)
		{
			tmp=itriangles[i];
			itriangles[i]=itriangles[i+1];
			itriangles[i+1]=tmp;
		}
	}
	
	protected void createFaces() {
		faces=new ArrayList<Shape3D>();
		Iterator<FaceMesh> it=faceMeshes.iterator();
		BranchGroup toReturn=new BranchGroup();
		toReturn.setCapability(BranchGroup.ALLOW_DETACH);
		int n=0;
		
		while(it.hasNext())
		{			
			FaceMesh fm=it.next();
			
			//Case of an unmeshed face
			if(fm.getNodes().length==0){
				n++;
				continue;
			}
			
			int[] reversed = fm.getMesh().clone();
			
			reverseMesh(reversed);
			
			GeometryInfo gi=new GeometryInfo(GeometryInfo.TRIANGLE_ARRAY);
			gi.setCoordinates(fm.getNodes());
			gi.setCoordinateIndices(fm.getMesh());			
			NormalGenerator ng = new NormalGenerator();
			ng.generateNormals(gi);
			Stripifier st = new Stripifier();
			st.stripify(gi);
	        
			GeometryArray g=gi.getGeometryArray();
			g.setCapability(GeometryArray.ALLOW_COUNT_READ);
			g.setCapability(GeometryArray.ALLOW_FORMAT_READ);
			g.setCapability(GeometryArray.ALLOW_COORDINATE_READ);
			g.setCapability(IndexedGeometryArray.ALLOW_COORDINATE_INDEX_READ);	
			
			
			
			 
			Shape3D shape3d=new Shape3D(g);
			shape3d.setAppearance(vs.getFaceAppearance());
			shape3d.setCapability(Shape3D.ALLOW_APPEARANCE_READ);
			shape3d.setCapability(Shape3D.ALLOW_APPEARANCE_WRITE);			
			shape3d.setCapability(Shape3D.ALLOW_GEOMETRY_READ);
			shape3d.setCapability(Node.ALLOW_PICKABLE_WRITE);
			toReturn.addChild(shape3d);
			faces.add(shape3d);
			
			
			gi=new GeometryInfo(GeometryInfo.TRIANGLE_ARRAY);
			gi.setCoordinates(fm.getNodes());
			gi.setCoordinateIndices(reversed);
			ng.generateNormals(gi);
			st = new Stripifier();
			st.stripify(gi);
			g=gi.getGeometryArray();
			
			shape3d=new Shape3D(g);
			shape3d.setAppearance(vs.getFaceAppearance());
			shape3d.setCapability(Shape3D.ALLOW_APPEARANCE_READ);
			shape3d.setCapability(Shape3D.ALLOW_APPEARANCE_WRITE);
			shape3d.setCapability(Shape3D.ALLOW_GEOMETRY_READ);
			shape3d.setCapability(Node.ALLOW_PICKABLE_WRITE);
			toReturn.addChild(shape3d);
			faces.add(shape3d);

			n++;
		}
		facesNode = toReturn;
	}
	
	public BranchGroup getFaces() {
		if (drawMesh) return mesh.getFaces(); else return facesNode;
	}
	
	private void createEdgeArrays() {	
		edgeArrays = new ArrayList<float[]>();
		TopExp_Explorer explorer = new TopExp_Explorer();
		HashSet<TopoDS_Edge> alreadyDone=new HashSet<TopoDS_Edge>();
	    Bnd_Box box = new Bnd_Box(); 
		BRepBndLib.add(shape,box);
		double[] bbox = box.get();
	    //double[] bbox=computeBoundingBox();
		double boundingBoxDeflection=0.0005*
			Math.max(Math.max(bbox[3]-bbox[0], bbox[4]-bbox[1]), bbox[5]-bbox[2]);

		for (explorer.init(shape, TopAbs_ShapeEnum.EDGE); explorer.more(); explorer.next())
		{
		    TopoDS_Shape s = explorer.current();		    
		    if (!(s instanceof TopoDS_Edge)) continue; // should not happen!
		    TopoDS_Edge e = (TopoDS_Edge)s;
		    
		    if(!alreadyDone.add(e))
		    	continue;
						
			double[] range = BRep_Tool.range(e);
		    Geom_Curve gc = BRep_Tool.curve(e, range);
		    float[] array;
		    if(gc!=null)
		    {
			    GeomAdaptor_Curve adaptator = new GeomAdaptor_Curve(gc);
				GCPnts_UniformDeflection deflector = new GCPnts_UniformDeflection();

				deflector.initialize(adaptator, boundingBoxDeflection, range[0], range[1]);
				int npts = deflector.nbPoints();
				
				// Allocate one additional point at each end  = parametric value 0, 1
				array = new float[(npts+2)*3];		    
			    int j=0;
			    double[] values = adaptator.value(range[0]);
			    array[j++] = (float) values[0];
			    array[j++] = (float) values[1];
			    array[j++] = (float) values[2];
			    // All intermediary points
				for (int i=0; i<npts; ++i) {
				    values = adaptator.value(deflector.parameter(i+1));
				    array[j++] = (float) values[0];
				    array[j++] = (float) values[1];
				    array[j++] = (float) values[2];
				}
				// Add last point
			    values = adaptator.value(range[1]);
			    array[j++] = (float) values[0];
			    array[j++] = (float) values[1];
			    array[j++] = (float) values[2];
			    edgeArrays.add(array);
		    }
		    else
		    {
		    	if (!BRep_Tool.degenerated(e))
		    	{
				    // So, there is no curve, and the edge is not degenerated?
				    // => draw lines between the vertices and ignore curvature  
				    // best approximation we can do
					ArrayList<double[]> aa = new ArrayList<double[]>(); // store points here
					for (TopExp_Explorer explorer2 = new TopExp_Explorer(s, TopAbs_ShapeEnum.VERTEX);
						explorer2.more(); explorer2.next())
					{
					    TopoDS_Shape sv = explorer2.current();
					    if (!(sv instanceof TopoDS_Vertex)) continue; // should not happen!
					    TopoDS_Vertex v = (TopoDS_Vertex)sv;
					    aa.add(BRep_Tool.pnt(v));
					}
					array = new float[aa.size()*3];
					for(int i=0, j=0; i<aa.size(); i++)
					{
						double[] f=aa.get(i);
						array[j++]=(float) f[0];
						array[j++]=(float) f[1];
						array[j++]=(float) f[2];
					}
					edgeArrays.add(array);
				}
		    }
		}		
	}
	
	protected void createEdges() {
		edges=new ArrayList<Shape3D>();
		Iterator<float[]> it = edgeArrays.iterator();
		BranchGroup toReturn=new BranchGroup();
		toReturn.setCapability(BranchGroup.ALLOW_DETACH);
		int n=0;
		
		while(it.hasNext())
		{
			float[] coordinates=it.next();			
			LineStripArray lsa=new LineStripArray(coordinates.length/3,
				GeometryArray.COORDINATES,
				new int[]{coordinates.length/3});
			
			lsa.setCapability(GeometryArray.ALLOW_COLOR_READ);
			lsa.setCapability(GeometryArray.ALLOW_COLOR_WRITE);
			lsa.setCapability(GeometryArray.ALLOW_COORDINATE_READ);
			lsa.setCapability(GeometryArray.ALLOW_COUNT_READ);
			lsa.setCapability(GeometryArray.ALLOW_FORMAT_READ);
			
			lsa.setCoordinates(0, coordinates);
			Shape3D shape3d=new Shape3D(lsa);
			
			shape3d.setAppearance(vs.getLineAppearance());
			shape3d.setCapability(Shape3D.ALLOW_APPEARANCE_READ);
			shape3d.setCapability(Shape3D.ALLOW_APPEARANCE_WRITE);
			shape3d.setCapability(Shape3D.ALLOW_GEOMETRY_READ);
			shape3d.setCapability(Node.ALLOW_PICKABLE_WRITE);
			toReturn.addChild(shape3d);
			edges.add(shape3d);
			n++;
		}
		edgesNode = toReturn;
	}
	
	public BranchGroup getEdges() {
		if (drawMesh) return mesh.getEdges(); else 	return edgesNode;
	}
	
	public BranchGroup getText() {
		if (drawMesh) return mesh.getText(); else return null;
	}
	
	public Bounds getBounds() {
		if (drawMesh) return mesh.getBounds();
		Bounds b = null;
		if (facesNode != null) b = facesNode.getBounds();
		if (edgesNode != null) b.combine(edgesNode.getBounds());
		if (textNode != null) b.combine(textNode.getBounds());
		return b;
	}
	
	protected TopoDS_Shape haveCommon(Shape s) {
		if (!getBounds().intersect(s.getBounds())) return null;		
		TopoDS_Shape common = new BRepAlgoAPI_Common(shape, s.shape).shape();
		if (isEmpty(common)) return null;
		TopoDS_Shape someShape = common;
		if (s.getType() > type)
			someShape = new BRepAlgoAPI_Cut(shape, s.shape).shape();
		if (s.getType() < type)
			someShape = new BRepAlgoAPI_Cut(s.shape, shape).shape();
		if (isEmpty(someShape)) return null;
		return common;
	}
	
	public boolean isIntersecting(Shape s) {
		return (haveCommon(s) != null);
	}
	
	protected static boolean isEmpty(TopoDS_Shape s) {
		if (s == null) return true;
		TopExp_Explorer explorer = new TopExp_Explorer();
		explorer.init(s, TopAbs_ShapeEnum.FACE); 
		boolean haveFaces = explorer.more();
		explorer.init(s, TopAbs_ShapeEnum.EDGE);
		boolean haveEdges = explorer.more();
		return !(haveFaces || haveEdges);
	}
	
	public boolean isEmpty() {
		return isEmpty(shape);
	}
	
	public void select() {
		selected = !selected;
		setSelected(selected);
	}
	
	public void setSelected(boolean v) {
		if (drawMesh) {
			mesh.setSelected(v);
			return;
		}
		selected = v;
		for (Shape3D s : faces) {			
			if (selected) s.setAppearance(vs.getSelectedFaceAppearance());
			else s.setAppearance(vs.getFaceAppearance());
		}
		for (Shape3D s : edges) {			
			if (selected) s.setAppearance(vs.getSelectedLineAppearance());
			else s.setAppearance(vs.getLineAppearance());
		}
	}
	
	public boolean isSelected() {
		return selected;
	}
	
	private void getShapeVertices() {
		TopExp_Explorer explorer = new TopExp_Explorer();
		vertexMeshSize = new ArrayList<Vertex>();
		for (explorer.init(shape, TopAbs_ShapeEnum.VERTEX); explorer.more(); explorer.next()) {						
			TopoDS_Shape sh = explorer.current();			
			TopoDS_Vertex vertex = (TopoDS_Vertex)sh;				
			vertexMeshSize.add(new Vertex(vertex));				
		}
	}
	
	private void getShapeEdges() {
		TopExp_Explorer explorer = new TopExp_Explorer();
		edgeMeshSize = new ArrayList<Edge>();
		for (explorer.init(shape, TopAbs_ShapeEnum.EDGE); explorer.more(); explorer.next()) {						
			TopoDS_Shape sh = explorer.current();			
			TopoDS_Edge edge = (TopoDS_Edge)sh;				
			edgeMeshSize.add(new Edge(edge));				
		}
	}
	
	public void setMeshSize(double size) {
		meshSize = size;
	}
	
	public double getMeshSize() {
		return meshSize;
	}
	
	public void setMeshSize(double x, double y, double z, double size) {
		if (vertexMeshSize == null) getShapeVertices();
		double minDist = Double.MAX_VALUE;
		Vertex vert = null;
		for (Vertex v : vertexMeshSize) {
			double d = v.distance(x, y, z);
			if (d < minDist) {
				minDist = d;
				vert = v;
			}
		}		
		if (vert != null) {
			vert.size = size;
		}
	}
	
	public void setMeshSize(Point3d p, double size) {
		setMeshSize(p.x, p.y, p.z, size);
	}
	
	public void setMeshSize(Point3d p1, Point3d p2, double size) {

	}
	
	public void setMeshSize(double x, double y, double z, int n) {
		if (edgeMeshSize == null) getShapeEdges();
		double minDist = Double.MAX_VALUE;
		Edge edge = null;
		for (Edge e : edgeMeshSize) {
			double d = e.distance(x, y, z);
			if (d < minDist) {
				minDist = d;
				edge = e;
			}
		}
		if (edge != null) {
			edge.numElements = n;
		}
	}
	
	
	
	
	public void mesh() {
		Mesh m = new Mesh(vs);
		Mesher.CreateGmodel(TopoDS_Shape.getCPtr(shape));
		Mesher.SetMeshSize(meshSize);
		if (vertexMeshSize != null) {
			
			for (Vertex v : vertexMeshSize) {
				if (v.size != meshSize)
					Mesher.SetMeshSizeInPoint(TopoDS_Vertex.getCPtr(v.vertex), v.size);
			}
		}
					
		if (edgeMeshSize != null) {
			for (Edge e : edgeMeshSize) {
				if (e.numElements != -1)
					Mesher.SetPointsTransfiniteInEdge(TopoDS_Edge.getCPtr(e.edge), e.numElements);
			}
		}			
		double[] points = Mesher.MeshShape();
		int[] indtetr = Mesher.Indtetr();
		int[] indtrian = Mesher.Indtrian();
		int[] indline = Mesher.Indline();
		m.addPoints(points);
		m.addTetrahedrons(indtetr);
		m.addTriangles(indtrian);
		m.addLines(indline);			
		setMesh(m);
	}
	

}
