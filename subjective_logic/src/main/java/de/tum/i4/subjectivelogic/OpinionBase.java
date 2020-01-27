/**
 * 
 */
package de.tum.i4.subjectivelogic;

import java.beans.PropertyChangeListener;
import java.beans.PropertyChangeSupport;

/**
 * @author squijano
 *
 */
public abstract class OpinionBase implements Opinion, Cloneable{

	private static final long serialVersionUID = 4746249216586553189L;
	
	public static final double TOLERANCE = 1.0E-010D;
	protected static final double TOLERANCE_ADJUST = 100000000000.0D;
	private static final double LOG_HALF = Math.log10(0.5D);
	protected static final double MAX_VALUE = Double.MAX_VALUE;
	protected PropertyChangeSupport changeSupport =  new PropertyChangeSupport(this);
	
	public static double adjust(double x) {
		double _x;
		
		_x = Math.round(x * 100000000000.0D) / 100000000000.0D;
		
		return x >= Double.MAX_VALUE ? Double.MAX_VALUE : x == (0.0D / 0.0D) ? (0.0D / 0.0D) : _x;
	}
	
	public static double constrain(double x) {
		return Math.min(1.0D, Math.max(0.0D, x));
	}
	
	public static final double erosionFactorFromHalfLife(double halfLife) {
		double erosionFactor = 0.0D;
		
		if (halfLife < 0.0D) {
			throw new IllegalArgumentException("Half-life h, must be 0 <= h");
		}
		
		if (halfLife == 0.0D) {
			erosionFactor = 0.0D;
		}
		else {
			erosionFactor = constrain(1.0D - 1.0D / Math.pow(2.0D, 1.0D / halfLife));
		}
		
		return erosionFactor;
	}
	
	public static final double erosionFactorFromHalfLife(double halfLife, double time) {
		double erosionFactor = 0.0D;
		
		if (halfLife < 0.0D) {
			throw new IllegalArgumentException("Half-life h, must be 0<=h");
		}
		
		if (time < 0.0D) {
			throw new IllegalArgumentException("Time t, must be 0<=t");
		}
		
		if ((time == 0.0D) || (halfLife == 0.0D)) {
			erosionFactor = 0.0D;
		}
		else {
			erosionFactor = Math.pow(erosionFactorFromHalfLife(halfLife), time);
		}
		
		return erosionFactor;
	}
	
	public static final double erosionFactorFromResidual(double residual, double time) {
		double erosionFactor = 0.0D;
		
		if ((residual < 0.0D) || (residual > 1.0D)) {
			throw new IllegalArgumentException("Residual r, must be 0<=r<=1");
		}
		
		if (time < 1.0D) {
			throw new IllegalArgumentException("Time t, must be 0<=t");
		}
		
		if ((time == 0.0D) || (residual == 1.0D)) {
			erosionFactor = 0.0D;
		}
		
		if (residual == 0.0D) {
			residual = 1.0E-010D;
		}
		
		erosionFactor = 1.0 - Math.pow(residual, 1.0D / time);
		
		return erosionFactor;
	}
	
	public static final double halfLifeFromErosionFactor(double erosionFactor) {
		double halfLife = 0.0D;
		
		if ((erosionFactor < 0.0D) || (erosionFactor > 1.0D)) {
			throw new IllegalArgumentException("Erosion Factor e, must be 0<=e<=1");
		}
		
		if (erosionFactor == 0.0D) {
			halfLife = 0.0D;
		}
		else {
			halfLife = adjust(LOG_HALF / Math.log10(1.0D - erosionFactor));
		}
		
		return halfLife;
	}
	
	public static final double halfLifeFromResidual(double residual, double time) {
		return halfLifeFromErosionFactor(erosionFactorFromResidual(residual, time));
	}
	
	public int compareTo(Opinion o) {
		return OpinionComparator.DEFAULT.compare(this, o);
	}
	
	public Object clone() throws CloneNotSupportedException{
		return super.clone();
	}
	
	/// Property change listeners
	
	public void addPropertyChangeListener(PropertyChangeListener listener) {
		this.changeSupport.addPropertyChangeListener(listener);
	}
	
	public void addPropertyChangeListener(String propertyName, PropertyChangeListener listener) {
		this.changeSupport.addPropertyChangeListener(propertyName, listener);
	}
	
	public PropertyChangeListener[] getPropertyChangeListeners() {
		return this.changeSupport.getPropertyChangeListeners();
	}
	
	public PropertyChangeListener[] getPropertyChangeListeners(String propertyName) {
		return this.getPropertyChangeListeners(propertyName);
	}
	
	public void removePropertyChangeListener(PropertyChangeListener listener) {
		this.changeSupport.removePropertyChangeListener(listener);
	}
	
	public void removePropertyChangeListener(String propertyName, PropertyChangeListener listener) {
		this.changeSupport.removePropertyChangeListener(propertyName, listener);
	}
	
	public boolean hasListeners(String propertyName) {
		return this.changeSupport.hasListeners(propertyName);
	}
	
	/// Property change listeners
	
}
