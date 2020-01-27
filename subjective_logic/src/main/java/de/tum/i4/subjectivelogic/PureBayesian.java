/**
 * 
 */
package de.tum.i4.subjectivelogic;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

/**
 * @author squijano
 *
 */
public class PureBayesian extends OpinionBase implements Bayesian {

	private static final long serialVersionUID = 5717003458709328621L;

	/**
	 * String format representation of an evidential (Bayesian) opinion
	 */
	private static final String TO_STRING_FORMAT = "w=(r=%1$1.3f, s=%2$1.3f, a=%3$1.3f, e=%4$1.3f";

	/**
	 * Base rate associated to this opinion
	 */
	private double baseRate = 0.5D;

	/**
	 * Expected probability associated to this opinion
	 */
	private double expectation = 0.0D;

	/**
	 * Support value of a variable/proposition being true
	 */
	private double negative = 0.0D;

	/**
	 * Support value of a variable/proposition being false
	 */
	private double positive = 0.0D;

	/**
	 * Flag to recalculate opinion params in case of a property change is triggered
	 */
	private boolean recalculate = false;

	/**
	 * Number of observations (positive and negative)
	 */
	private double rs2 = 2.0D;

	public PureBayesian() {

	}

	/**
	 * Creates a new PureBayesian Opinion with the given positive and negative
	 * support values
	 * 
	 * @param r positive support value
	 * @param s negative support value
	 */
	public PureBayesian(double r, double s) {
		setPositive(r);
		setNegative(s);
	}

	/**
	 * Creates a new PureBayesian Opinion with the given support values and base
	 * rate
	 * 
	 * @param r        positive support value
	 * @param s        negative support value
	 * @param baseRate the associated base rate
	 */
	public PureBayesian(double r, double s, double baseRate) {
		this(r, s);
		setBaseRate(baseRate);
	}

	/**
	 * Casts a generic Opinion to a PureBayesian opinion and creates a copy of it
	 * 
	 * @param o the opinion to copy
	 */
	public PureBayesian(Opinion o) {
		PureBayesian x = null;

		if (o == null) {
			throw new NullPointerException("Opinion must not be null");
		}

		if ((o instanceof SubjectiveOpinion)) {
			x = o.toPureBayesian();
		} else if ((o instanceof DiscreteBayesian)) {
			x = o.toPureBayesian();
		} else {
			throw new ClassCastException("Opinion is not and instance of Bayesian, Subjective Opinion");
		}

		synchronized (o) {
			this.positive = x.positive;
			this.negative = x.negative;
			this.baseRate = x.baseRate;
			this.rs2 = x.rs2;
			this.expectation = x.expectation;
			this.recalculate = x.recalculate;
		}
	}

	/**
	 * Copy constructor
	 * 
	 * @param o the opinion to copy
	 */
	public PureBayesian(PureBayesian o) {
		if (o == null) {
			throw new NullPointerException("Opinion must not be null");
		}

		synchronized (o) {
			this.positive = o.positive;
			this.negative = o.negative;
			this.baseRate = o.baseRate;
			this.rs2 = o.rs2;
			this.expectation = o.expectation;
			this.recalculate = o.recalculate;
		}
	}

	/**
	 * Returns this instance's base rate
	 */
	@Override
	public double getBaseRate() {
		return this.baseRate;
	}

	/**
	 * Updates this opinion's base rate
	 * 
	 * @param baseRate the new base rate
	 */
	public void setBaseRate(double baseRate) {
		double old = this.baseRate;

		if ((baseRate < 0.0D) || (baseRate > 1.0D)) {
			throw new IllegalArgumentException("Base rate, x, must be 0<=x<=1");
		}

		if (!(old == baseRate)) {
			synchronized (this) {
				this.baseRate = baseRate;
				this.recalculate = true;
			}
			this.changeSupport.firePropertyChange("base rate", old, baseRate);
		}
	}

	/**
	 * Returns this instance's expected probability
	 */
	@Override
	public double getExpectation() {
		return this.expectation;
	}

	public PureBayesian adjustExpectation(double value) {
		return this.toSubjectiveOpinion().adjustExpectation(value).toPureBayesian();
	}

	/**
	 * Returns this opinion's negative support value
	 */
	@Override
	public double getNegative() {
		return this.negative;
	}

	/**
	 * Updates this opinion's negative support value
	 * 
	 * @param s the new negative support value
	 */
	public void setNegative(double s) {
		double old = this.negative;

		if (s < 0.0D) {
			throw new IllegalArgumentException("Value x, must be: 0<=x");
		}

		if (!(old == s)) {
			synchronized (this) {
				this.negative = Math.min(200000000000.0D, s);
				this.recalculate = true;
			}
			this.changeSupport.firePropertyChange("negative", old, s);
		}
	}

	/**
	 * Returns this opinion's positive support value
	 */
	@Override
	public double getPositive() {
		return this.positive;
	}

	/**
	 * Updates this opinion's positive support value
	 * 
	 * @param r the new positive support value
	 */
	public void setPositive(double r) {
		double old = this.positive;

		if (r < 0.0D) {
			throw new IllegalArgumentException("Value x, must be: 0<=x");
		}

		if (!(old == r)) {
			synchronized (this) {
				this.negative = Math.min(200000000000.0D, r);
				this.recalculate = true;
			}
			this.changeSupport.firePropertyChange("positive", old, r);
		}
	}

	/**
	 * Returns this opinion's weight, that is the positive plus negative support
	 * values
	 * 
	 * @return this opinion weight
	 */
	public synchronized double getWeight() {
		return this.positive + this.negative;
	}

	public PureBayesian increasedUncertainty() {
		return this.toSubjectiveOpinion().increasedUncertainty().toPureBayesian();
	}

	public PureBayesian maximizedUncertainty() {
		return this.toSubjectiveOpinion().uncertainOpinion().toPureBayesian();
	}

	/**
	 * Recalculates and updates this opinion's parameters (r, s, a, e)
	 */
	private void setDependants() {
		synchronized (this) {
			if (this.recalculate) {
				this.positive = OpinionBase.adjust(this.positive);
				this.negative = OpinionBase.adjust(this.negative);
				this.rs2 = (this.positive + this.negative + 2.0D);
				this.expectation = ((this.positive + this.baseRate * 2.0D) / this.rs2);
				this.recalculate = false;
			}
		}
	}

	/**
	 * Returns this opinion's size, that is the number of observations. By default a
	 * Pure Bayesian has size 2
	 * 
	 * @return this opinion's size
	 */
	public int size() {
		return 2;
	}

	@Override
	public int compareTo(Opinion o) {
		return super.compareTo(o);
	}

	public String toString() {
		String opinion = "";
		synchronized (this) {
			setDependants();
			opinion = String.format(TO_STRING_FORMAT, this.positive, this.negative, this.baseRate, getExpectation());
		}

		return opinion;
	}

	/**
	 * Checks if the given instance is equal to this Opinion. The objects are equal
	 * if the given object is a PureBayesian instance and, the positive and negative
	 * support values and the base rates are equal
	 */
	public boolean equals(Object obj) {
		boolean eq = false;
		if (obj == null) {
			eq = false;
		} else {
			if ((obj instanceof PureBayesian)) {
				PureBayesian opinion = (PureBayesian) obj;
				synchronized (this) {
					eq = (opinion.positive == this.positive);
					eq = eq && (opinion.negative == this.negative);
					eq = eq && (opinion.baseRate == this.baseRate);
				}
			}
		}

		return eq;
	}

	@Override
	public double max() {
		return Math.max(this.negative, this.positive);
	}

	@Override
	public double min() {
		return Math.min(this.negative, this.positive);
	}

	/**
	 * Returns this negative and positive (in that order) values as an array
	 */
	@Override
	public double[] values() {
		return new double[] { this.negative, this.positive };
	}

	/**
	 * Returns this instance
	 */
	@Override
	public PureBayesian toPureBayesian() {
		return this;
	}

	/**
	 * Returns this opinion as a SubjectiveOpinion instance
	 */
	@Override
	public SubjectiveOpinion toSubjectiveOpinion() {
		SubjectiveOpinion so = new SubjectiveOpinion();

		synchronized (this) {
			setDependants();
			so.setBelief(this.positive / this.rs2, 2.0D / this.rs2);
			so.setBaseRate(this.baseRate);
		}

		return so;
	}

	/**
	 * Converts this opinion to a DiscreteBayesian opinion of the given size
	 * 
	 * @param size size of the new DiscreteBayesian opinion
	 * @return a DiscreteBayesian opinion
	 */
	public DiscreteBayesian toDiscreteBayesian(int size) {
		DiscreteBayesian opinion = null;

		synchronized (this) {
			if (size < 2) {
				throw new IllegalArgumentException("Conversion not posible");
			}

			opinion = new DiscreteBayesian(new double[] { this.negative, this.positive });

			if (size == 2) {
				return opinion;
			}

			return opinion.toDiscreteBayesian(size);
		}
	}

	/**
	 * Converts this opinion to a DiscreteBayesian opinion
	 * 
	 * @return a DiscreteBayesian opinion
	 */
	public DiscreteBayesian toDiscreteBayesian() {
		return toDiscreteBayesian(2);
	}

	/////////////////////////
	/// General Operators ///
	/////////////////////////

	/**
	 * Returns the consensus of the given opinions
	 * 
	 * @param x the first opinion
	 * @param y the second opinion
	 * @return the consensus result as a PureBayesian opinion
	 */
	private static PureBayesian consensus(PureBayesian x, PureBayesian y) {
		PureBayesian opinion = null;

		if ((x == null) || (y == null)) {
			throw new NullPointerException("Opinions must not be null");
		}

		opinion = new PureBayesian();

		x.positive += y.positive;
		x.negative += y.negative;

		opinion.recalculate = true;

		return opinion;
	}

	/**
	 * Normalizes the expected probability of the given opinions according to the
	 * total expected probability
	 * 
	 * @param opinions the opinions to normalize
	 * @return the normalized opinions
	 */
	public static final List<PureBayesian> normalize(Collection<? extends PureBayesian> opinions) {
		List<PureBayesian> newOpinions = null;
		double sum = 0.0D;

		if (opinions == null) {
			throw new NullPointerException("Opinions must not be null");
		}

		newOpinions = new ArrayList<>();

		for (PureBayesian opinion : opinions) {
			sum += opinion.getExpectation();
		}

		for (PureBayesian o : opinions) {
			if (sum == 0.0D) {
				newOpinions.add(new PureBayesian(0.0D, 0.0D));
			} else {
				newOpinions.add(new PureBayesian(o.adjustExpectation(o.getExpectation() / sum)));
			}
		}

		return newOpinions;
	}

	/**
	 * Erodes an Opinion with the given factor
	 * 
	 * @param x      the opinion to erode
	 * @param factor the factor to use
	 * @return the eroded opinion
	 */
	private static PureBayesian erosion(PureBayesian x, double factor) {
		PureBayesian o = null;

		if (x == null) {
			throw new NullPointerException("Opinion must not be null");
		}

		if ((factor < 0.0D) || (factor > 1.0D)) {
			throw new IllegalArgumentException("Erosion factor, f must be 0<=f<=1");
		}

		synchronized (x) {
			o = new PureBayesian();

			double f = 1.0D - factor;

			x.positive *= f;
			x.negative *= f;
			o.baseRate = x.baseRate;

			o.recalculate = true;
		}

		return o;
	}

	/**
	 * Erodes this Opinion with the given factor
	 * 
	 * @param factor the factor to use
	 * @return the eroded opinion
	 */
	public final PureBayesian erode(double factor) {
		return erosion(this, factor);
	}

	/**
	 * Calculates the decay value of this opinion
	 * 
	 * @param halfLife
	 * @param time
	 * @return the decayed PureBayesian opinion
	 */
	public final PureBayesian decay(double halfLife, double time) {
		return erosion(this, OpinionBase.erosionFactorFromHalfLife(halfLife, time));
	}

	/**
	 * Calculates a discount scale from the given opinion using a PureBayesian
	 * discounter
	 * 
	 * @param discounter a PureBayesian opinion to use as discounter
	 * @param opinion    a opinion to be discounted
	 * @return the calculated discount scale
	 */
	private static double discountScale(PureBayesian discounter, PureBayesian opinion) {
		double r = discounter.getPositive();
		double s = discounter.getNegative();
		// double w = discounter.getWeight();

		double divisor = (opinion.getWeight() + 2.0D) * (s + 2.0D) + 2.0D * r;
		double discount = 0.0D;
		if (divisor != 0.0D) {
			discount = 2.0D * r / divisor;
		}

		return discount;
	}

	/**
	 * Calculates a discount scale from the given opinion using a SubjectiveOpinion
	 * discounter
	 * 
	 * @param discounter a SubjectiveOpinion opinion to use as discounter
	 * @param opinion    a opinion to be discounted
	 * @return the calculated discount scale
	 */
	private static double discountScale(SubjectiveOpinion discounter, PureBayesian opinion) {
		double b = discounter.getBelief();
		double d = discounter.getDisbelief();
		double u = discounter.getUncertainty();

		double divisor = (d + u) * (opinion.getWeight() + 2.0D) + 2.0D * b;
		double discount = 0.0D;
		if (divisor != 0.0D) {
			discount = 2.0D * b / divisor;
		}

		return discount;
	}

	/**
	 * Discounts the given discounter from the passed opinion
	 * 
	 * @param discounter the discounter to use
	 * @param opinion    the opinion to discount
	 * @return the new discounted opinion
	 */
	private static PureBayesian discount(Opinion discounter, PureBayesian opinion) {
		PureBayesian y = null;
		PureBayesian o = null;
		double scale = 0.0D;

		if ((discounter == null) || (opinion == null)) {
			throw new NullPointerException("Opinions must not be null");
		}

		y = new PureBayesian(opinion);
		o = new PureBayesian();

		if ((discounter instanceof PureBayesian)) {
			scale = discountScale(new PureBayesian(discounter), opinion);
		} else {
			if ((discounter instanceof DiscreteBayesian)) {
				scale = discountScale(discounter.toPureBayesian(), opinion);
			} else {
				scale = discountScale(new SubjectiveOpinion(discounter), opinion);
			}
		}

		y.positive *= scale;
		y.negative *= scale;
		o.recalculate = true;

		return o;
	}

	/**
	 * Discounts from this instance the given opinion
	 * 
	 * @param y the opinion to discount
	 * @return the discount result as a PureBayesian opinion
	 */
	public PureBayesian discount(Opinion y) {
		return this.toSubjectiveOpinion().discountBy(y).toPureBayesian();
	}

	/**
	 * Discounts by given opinion this instance
	 * 
	 * @param y the opinion to discount
	 * @return the discount result as a PureBayesian opinion
	 */
	public PureBayesian discountBy(Opinion y) {
		return discount(y, this);
	}

	//////////////////////////
	/// Bayesian Operators ///
	//////////////////////////

	/**
	 * Adds to this instance the given opinion
	 * 
	 * @param o the opinion to add
	 * @return the addition result as a PureBayesian opinion
	 */
	public PureBayesian add(Opinion o) {
		return this.toSubjectiveOpinion().add(o).toPureBayesian();
	}

	/**
	 * Adds all the given opinions
	 * 
	 * @param opinions the opinions to add
	 * @return the addition result as a PureBayesian opinion
	 */
	public static final PureBayesian add(Collection<? extends Opinion> opinions) {
		return SubjectiveOpinion.add(opinions).toPureBayesian();
	}

	/**
	 * Subtracts the given opinion from this Opinion
	 * 
	 * @param o the opinion to subtract
	 * @return the subtraction result as a PureBayesian opinion
	 */
	public PureBayesian subtract(Opinion o) {
		return this.toSubjectiveOpinion().subtract(o).toPureBayesian();
	}

	/**
	 * Returns the complement (negation) of this Opinion
	 * 
	 * @return the complement as a PureBayesian opinion
	 */
	public final PureBayesian not() {
		PureBayesian opinion = new PureBayesian();

		opinion.positive = this.negative;
		opinion.negative = this.positive;
		opinion.baseRate = (1.0D - this.baseRate);

		return opinion;
	}

	/**
	 * Multiplies this opinion by the given opinion
	 * 
	 * @param o the factor opinion
	 * @return the multiplication result as a PureBayesian opinion
	 */
	public PureBayesian and(Opinion o) {
		return this.toSubjectiveOpinion().and(o).toPureBayesian();
	}

	/**
	 * Multiplies all the given opinions
	 * 
	 * @param opinions the opinions to multiply
	 * @return the multiplication result as a PureBayesian opinion
	 */
	public static final PureBayesian and(Collection<? extends Opinion> opinions) {
		return SubjectiveOpinion.and(opinions).toPureBayesian();
	}

	/**
	 * Comultiplies this Opinion with the given opinion
	 * 
	 * @param o the factor opinion
	 * @return the comultiplication result as a PureBayesian opinion
	 */
	public PureBayesian or(Opinion o) {
		return toSubjectiveOpinion().or(o).toPureBayesian();
	}

	/**
	 * Comultiplies all the given opinions
	 * 
	 * @param opinions the opinions to comultiply
	 * @return the comultiplication result as a PureBayesian opinion
	 */
	public static final PureBayesian or(Collection<? extends Opinion> opinions) {
		return SubjectiveOpinion.or(opinions).toPureBayesian();
	}

	/**
	 * Divides this Opinion by the given opinion
	 * 
	 * @param o the divisor opinion
	 * @return the division result as a PureBayesian opinion
	 */
	public PureBayesian unAnd(Opinion o) {
		return this.toSubjectiveOpinion().unAnd(o).toPureBayesian();
	}

	/**
	 * Codivides this Opinion by the given opinion
	 * 
	 * @param o the divisor opinion
	 * @return the codivision result as a PureBauesian opinion
	 */
	public PureBayesian unOr(Opinion o) {
		return this.toSubjectiveOpinion().unOr(o).toPureBayesian();
	}

	/**
	 * Deducts (Modues Ponens) this opinion using the given Conditionals
	 * 
	 * @param conditionals the conditionals to use
	 * @return the deduction result as a PureBayesian opinion
	 */
	public PureBayesian deduce(Conditionals conditionals) {
		return this.toSubjectiveOpinion().deduce(conditionals).toPureBayesian();
	}

	/**
	 * Deducts (Modus Ponens) the given opinions
	 * 
	 * @param yTx
	 * @param yFx
	 * @return the deduction result as a PureBayesian opinion
	 */
	public PureBayesian deduce(Opinion yTx, Opinion yFx) {
		return this.toSubjectiveOpinion().deduce(yTx, yFx).toPureBayesian();
	}

	/**
	 * Abducts (Modus Tollens) this opinion using the given Conditionals
	 * 
	 * @param conditionals the conditionals to use
	 * @param baseRateX
	 * @return the abduction result as a PureBayesian opinion
	 */
	public PureBayesian abduce(Conditionals conditionals, double baseRateX) {
		return this.toSubjectiveOpinion().abduce(conditionals, baseRateX).toPureBayesian();
	}

	/**
	 * Abducts (Modus Tollens) the given opinions
	 * 
	 * @param xTy
	 * @param xFy
	 * @param baseRateX
	 * @return the abduction result as a PureBayesian opinion
	 */
	public PureBayesian abduce(Opinion xTy, Opinion xFy, double baseRateX) {
		return this.toSubjectiveOpinion().abduce(xTy, xFy, baseRateX).toPureBayesian();
	}

	////////////////////
	/// SL Operators ///
	////////////////////

	/**
	 * Fuses this opinion with the given opinion. The fusion of PureBayesian
	 * opinions is implemented as a consensus operation
	 * 
	 * @param o the opinion to fuse
	 * @return the fusion result as a PureBayesian opinion
	 */
	public final PureBayesian fuse(Opinion o) {
		return consensus(new PureBayesian(this), new PureBayesian(o));
	}

	/**
	 * Fuses all the given opinions
	 * 
	 * @param opinions the opinions to fuse
	 * @return the fusion result as a PureBayesian opinion
	 */
	public static final PureBayesian fuse(Collection<? extends Opinion> opinions) {
		PureBayesian x = null;

		if (opinions == null) {
			throw new NullPointerException("Opinion must not be null");
		}

		if (opinions.isEmpty()) {
			throw new OpinionArithmeticException("Opinions must not be empty");
		}

		for (Opinion opinion : opinions) {
			x = x == null ? new PureBayesian(opinion) : x.fuse(opinion.toPureBayesian());
		}

		return x;
	}

	/**
	 * Fuses and averages this instance with the given opinion
	 * 
	 * @param o the second opinion
	 * @return the averaging fusion result as a PureBayesian opinion
	 */
	public final PureBayesian average(Opinion o) {
		Collection<Opinion> opinions = new ArrayList<>();

		opinions.add(new SubjectiveOpinion(this));
		opinions.add(new SubjectiveOpinion(o));

		return SubjectiveOpinion.smoothAverage(opinions).toPureBayesian();
	}

	/**
	 * Fuses and averages all the given opinions
	 * 
	 * @param opinions the opinions to use
	 * @return the averaging fusion result as a PureBayesian opinion
	 */
	public static final PureBayesian average(Collection<? extends Opinion> opinions) {
		return SubjectiveOpinion.smoothAverage(opinions).toPureBayesian();
	}

}
