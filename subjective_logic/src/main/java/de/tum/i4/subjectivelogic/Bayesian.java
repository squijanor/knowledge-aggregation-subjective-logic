/**
 * 
 */
package de.tum.i4.subjectivelogic;

/**
 * @author squijano
 *
 */
public interface Bayesian {

	/**
	 * Returns the support value of a variable/proposition being false
	 * 
	 * @return	the negative support
	 */
	double getNegative();
	
	/**
	 * Returns the support value of a variable/proposition being true
	 * 
	 * @return	the positive support
	 */
	double getPositive();
	
	/**
	 * Returns the highest value among the positive and negative support values
	 *  
	 * @return	the maximum support value
	 */
	double max();
	
	/**
	 * Returns the lowest value among the positive and negative support values
	 * 
	 * @return	the minimum support value
	 */
	double min();
	
	/**
	 * Returns the negative and positive (in that order) support values of this Opinion
	 *  
	 * @return	array of negative and positive support values
	 */
	double[] values();
}
