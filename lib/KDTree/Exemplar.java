/*
 *  Copyright (C) 2010 Duy Nguyen <duyn.ng@gmail.com>.
 */

package duyn.algorithm.nearestneighbours;
import java.util.Arrays;
/**
 * A sample point in multi-dimensional space. Needed because each sample
 * may contain an arbitrary payload.
 *
 * Note: this class does not make allowance for a payload. Sub-class if
 * you want to store something more than just data points.
 *
 * @author duyn
 */
public class Exemplar {
	public final double[] domain;

	public Exemplar(double[] domain) {
		this.domain = domain;
	}

	public final boolean
	collocated(final Exemplar other) {
		return Arrays.equals(domain, other.domain);
	}
}
