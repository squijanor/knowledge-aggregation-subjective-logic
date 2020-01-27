/**
 * 
 * OpinionComparator implements compare method to check for equality of Opinions.
 * By default, the comparator will check for equality of expected probability, 
 * uncertainty, base rate and belief of opinions, unless the CERTAINTY
 * comparator is explicitly called.
 *  
 */
package de.tum.i4.subjectivelogic;

import java.util.Comparator;

/**
 * @author squijano
 *
 */
public abstract class OpinionComparator implements Comparator<Opinion> {

	/**
	 * Comparator overriding compare method to check for opinions' equality of the
	 * amount of uncertainty
	 * 
	 */
	public static final OpinionComparator CERTAINTY = new OpinionComparator() {
		@Override
		public int compare(Opinion o1, Opinion o2) {
			int result = 0;

			if ((o1 == null) || (o2 == null)) {
				throw new NullPointerException("Opinions cannot be null");
			}

			double u_x = o1.toSubjectiveOpinion().getUncertainty();
			double u_y = o2.toSubjectiveOpinion().getUncertainty();

			if (u_x < u_y) {
				result = 1;
			} else if (u_x > u_y) {
				result = -1;
			} else {
				result = 0;
			}

			return result;
		}
	};

	/**
	 * Comparator overriding compare method to check opinions' equality of expected
	 * probability, uncertainty, base rate and belief
	 *
	 */
	public static final OpinionComparator EXPECTATION = new OpinionComparator() {
		@Override
		public int compare(Opinion o1, Opinion o2) {
			int result = 0;

			if ((o1 == null) || (o2 == null)) {
				throw new NullPointerException("Opinions cannot be null");
			}

			SubjectiveOpinion so1 = o1.toSubjectiveOpinion();
			SubjectiveOpinion so2 = o2.toSubjectiveOpinion();

			double e_x = so1.getExpectation();
			double e_y = so2.getExpectation();

			if (e_x > e_y) {
				result = 1;
			} else if (e_x < e_y) {
				result = -1;
			} else {
				double u_x = so1.getUncertainty();
				double u_y = so2.getUncertainty();

				if (u_x < u_y) {
					result = 1;
				} else if (u_x > u_y) {
					result = -1;
				} else {
					double a_x = so1.getBaseRate();
					double a_y = so2.getBaseRate();

					if (a_x < a_y) {
						result = 1;
					} else if (a_x > a_y) {
						result = -1;
					} else {
						double b_x = so1.getBelief();
						double b_y = so2.getBelief();

						if (b_x > b_y) {
							result = 1;
						} else if (b_x < b_y) {
							result = -1;
						} else {
							result = 0;
						}
					}
				}
			}

			return result;
		}

	};

	/**
	 * Default opinion comparator. It will compare (in this order) the expected
	 * probability, uncertainty, base rate and belief of opinions
	 */
	public static final OpinionComparator DEFAULT = EXPECTATION;

	/**
	 * Compares two opinions and returns the maximal one
	 * 
	 * @param o1 the first opinion
	 * @param o2 the second opinion
	 * @return the maximal opinion
	 */
	public Opinion max(Opinion o1, Opinion o2) {
		Opinion max = null;

		if (compare(o1, o2) < 0) {
			max = o2;
		} else {
			max = o1;
		}

		return max;
	}

	/**
	 * Compares the given opinions and returns the highest one (highest expected
	 * probability)
	 * 
	 * @param opinions the opinions to compare
	 * @return the maximal opinion
	 */
	public Opinion max(Opinion[] opinions) {
		Opinion max = null;

		if (opinions == null) {
			throw new NullPointerException("Opinions must not be null");
		}

		for (int i = 0, size = opinions.length; i < size; i++) {
			Opinion o = opinions[i];

			if ((max == null) || ((o != null) && (compare(max, o) < 0))) {
				max = o;
			}
		}

		return max;
	}

	/**
	 * Compares two opinions and returns the minimal one
	 * 
	 * @param o1 first opinion
	 * @param o2 second opinion
	 * @return the minimal opinion
	 */
	public Opinion min(Opinion o1, Opinion o2) {
		Opinion min = null;

		if (compare(o1, o2) > 0) {
			min = o2;
		} else {
			min = o1;
		}

		return min;
	}

	/**
	 * Compares the given opinions and returns the lowest one (lowest expected
	 * probability, by default)
	 * 
	 * @param opinions the opinions to compare
	 * @return the minimal opinion
	 */
	public Opinion min(Opinion[] opinions) {
		Opinion min = null;

		if (opinions == null) {
			throw new NullPointerException("Opinions must not be null");
		}

		for (int i = 0, size = opinions.length; i < size; i++) {
			Opinion o = opinions[i];

			if ((min == null) || ((o != null) && (compare(min, o) > 0))) {
				min = o;
			}
		}

		return min;
	}
}
