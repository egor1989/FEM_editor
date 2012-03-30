package models.mechanics;

import core.Material;
import core.Time;
import models.ContactFE;

public class ConnectionLine extends ContactFE {

	double factor;
	
	public ConnectionLine(int intPointsNumber, int order, Material material, Time t) {
		super(1, 1, material, t);
	}

}
