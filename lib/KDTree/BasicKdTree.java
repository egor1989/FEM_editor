/*
 *  Copyright (C) 2010 Duy Nguyen <duyn.ng@gmail.com>.
 */

package duyn.algorithm.nearestneighbours;
import java.util.*;

/**
 * A basic kd-tree for tutorial purposes.
 *
 * @author duyn
 */
public class BasicKdTree<X extends Exemplar> {
	// Basic tree structure
	X data = null;
	BasicKdTree<X> left = null, right = null;

	// Split point. Dimension is defined when tree is created, value
	// not set until splitting happens.
	int splitDim = 0;
	double split = Double.NaN;

	public void
	add(X ex) {
		BasicKdTree.addToTree(this, ex);
	}

	public Iterable<? extends PrioNode<X>>
	search(double[] query, int nResults) {
		return BasicKdTree.search(this, query, nResults);
	}
	
	//// IMPLEMENTATION DETAILS ////
	
	private int
	dimensions() { return data.domain.length; }

	private boolean
	isTree() { return left != null; }

	private static <X extends Exemplar> void
	addToTree(BasicKdTree<X> tree, X ex) {
		while(tree != null) {
			if (tree.isTree()) {
				// Traverse in search of a leaf
				tree = ex.domain[tree.splitDim] <= tree.split
					? tree.left : tree.right;
			} else {
				if (tree.data == null) {
					tree.data = ex;
				} else {
					// Split tree and add
					
					// Find smallest exemplar to be our split point
					final int d = tree.splitDim;
					X leftX = ex, rightX = tree.data;
					if (rightX.domain[d] < leftX.domain[d]) {
						leftX = tree.data;
						rightX = ex;
					}
					tree.split = 0.5*(leftX.domain[d] + rightX.domain[d]);
					
					final int nextSplitDim =
						(tree.splitDim + 1)%tree.dimensions();
					
					tree.left = new BasicKdTree<X>();
					tree.left.splitDim = nextSplitDim;
					tree.left.data = leftX;
					
					tree.right = new BasicKdTree<X>();
					tree.right.splitDim = nextSplitDim;
					tree.right.data = rightX;
				}

				// Done.
				tree = null;
			}
		}
	}
	
	private static <X extends Exemplar> Iterable<? extends PrioNode<X>>
	search(BasicKdTree<X> tree, double[] query, int nResults) {
		final Queue<PrioNode<X>> results =
			new PriorityQueue<PrioNode<X>>(nResults,
				new Comparator<PrioNode<X>>() {

					// min-heap
					public int
					compare(PrioNode<X> o1, PrioNode<X> o2) {
						return o1.priority == o2.priority ? 0
							: o1.priority > o2.priority ? -1
							: 1;
					}

				}
			);
		final Deque<BasicKdTree<X>> stack
			= new LinkedList<BasicKdTree<X>>();
		stack.addLast(tree);
		while (!stack.isEmpty()) {
			tree = stack.removeLast();
			
			if (tree.isTree()) {
				// Guess nearest tree to query point
				BasicKdTree<X> nearTree = tree.left, farTree = tree.right;
				if (query[tree.splitDim] > tree.split) {
					nearTree = tree.right;
					farTree = tree.left;
				}
				
				// Only search far tree if our search sphere might
				// overlap with splitting plane
				if (results.size() < nResults
					|| sq(query[tree.splitDim] - tree.split)
						<= results.peek().priority)
				{
					stack.addLast(farTree);
				}

				// Always search the nearest branch
				stack.addLast(nearTree);
			} else {
				final double dSq = distanceSqFrom(query, tree.data.domain);
				if (results.size() < nResults
					|| dSq < results.peek().priority)
				{
					while (results.size() >= nResults) {
						results.poll();
					}

					results.offer(new PrioNode<X>(dSq, tree.data));
				}
			}
		}
		return results;
	}
	
	private static double
	distanceSqFrom(double[] p1, double[] p2) {
		// Note: profiling shows this is called lots of times, so it pays
		// to be well optimised
		double dSq = 0;
		for(int d = 0; d < p1.length; d++) {
			final double dst = p1[d] - p2[d];
			if (dst != 0)
				dSq += dst*dst;
		}
		return dSq;
	}
	
	private static double
	sq(double n) { return n*n; }
}
