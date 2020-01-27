/**
 * 
 */
package de.tum.i4.subjectivelogic;

import java.beans.PropertyChangeListener;
import java.io.Serializable;

/**
 * @author squijano
 *
 */
public interface Opinion extends Serializable, Comparable<Opinion>{

	/**
	 * Returns the base rate associated to this Opinion
	 * 
	 * @return	the base rate
	 */
	double getBaseRate();
	
	/**
	 * Returns the expected probability associated to this Opinion
	 * 
	 * @return	the expected probability
	 */
	double getExpectation();
	
	/**
	 * Returns a Subjective Opinion, w=(b, d, u, a), representation of this Opinion
	 * 
	 * @return	the subjective opinion
	 */
	SubjectiveOpinion toSubjectiveOpinion();
	
	/**
	 * Returns an Evidential Opinon (pure bayesian), w=(r,a), representation of this Opinion
	 * 
	 * @return	the Pure Bayesian opinion
	 */
	PureBayesian toPureBayesian();
	
	/**
	 * Compares the given opinion with this instance
	 */
	int compareTo(Opinion o);
	
	void addPropertyChangeListener(PropertyChangeListener listener);
	
	void addPropertyChangeListener(String propertyName, PropertyChangeListener listener);
	
	PropertyChangeListener[] getPropertyChangeListeners();
	
	PropertyChangeListener[] getPropertyChangeListeners(String propertyName);
	
	void removePropertyChangeListener(PropertyChangeListener listener);
	
	void removePropertyChangeListener(String propertyName, PropertyChangeListener listener);
	
	boolean hasListeners(String propertyName);
}
