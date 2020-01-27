/**
 * 
 */
package de.tum.i4.subjectivelogic;

import java.util.Collection;

import de.tum.i4.subjectivelogic.DiscreteBayesian;
import de.tum.i4.subjectivelogic.Opinion;
import de.tum.i4.subjectivelogic.OpinionArithmeticException;
import de.tum.i4.subjectivelogic.OpinionBase;
import de.tum.i4.subjectivelogic.PureBayesian;
import de.tum.i4.subjectivelogic.SubjectiveOpinion;

/**
 * @author squijano
 *
 */
public class DiscreteBayesian extends OpinionBase implements Bayesian {

	private static final long serialVersionUID = 6252439784456616910L;

	public static final double DEFAULT_POLARIZATION = 0.05D;

	private static final String EXTRAS_FORMAT = "w=(%1$s, a=%2$1.3f, e=%3$1.3f)";

	private static final String FORMAT = "%1$1.3f";

	private static final double ROOT_2 = Math.sqrt(2.0D);

	/**
	 * Base rate associated to this opinion
	 */
	private double baseRate = 0.5D;

	/**
	 * Set of observations related to this opinion
	 */
	private double[] buckets;

	/**
	 * Expected probability associated to this opinion
	 */
	private double expectation;

	/**
	 * Support value of a variable/proposition being true
	 */
	private double negative = 0.0D;

	/**
	 * Support value of a variable/proposition being false
	 */
	private double positive = 0.0D;

	/**
	 * Number of observations (positive and negative)
	 */
	private double rs2 = 2.0D;

	/**
	 * Flag to recalculate opinion params in case of a property change is triggered
	 */
	private boolean recalculate = false;

	public DiscreteBayesian() {
		this(2);
	}

	/**
	 * Copy constructor
	 * 
	 * @param o the opinion to copy
	 */
	public DiscreteBayesian(DiscreteBayesian o) {
		if (o == null) {
			throw new NullPointerException("Opinion cannot be null");
		}

		synchronized (o) {
			this.buckets = new double[o.buckets.length];

			for (int i = 0, size = this.buckets.length; i < size; i++) {
				this.buckets[i] = o.buckets[i];
			}
			this.baseRate = o.baseRate;
			this.expectation = o.expectation;
			this.recalculate = o.recalculate;
		}
	}

	/**
	 * Creates a new DiscreteBayesian opinion with the given vector of support
	 * values
	 * 
	 * @param bucketValues set of support values
	 */
	public DiscreteBayesian(double[] bucketValues) {
		if (bucketValues == null) {
			throw new NullPointerException("Bucket Values cannot be null");
		}

		if (bucketValues.length < 2) {
			throw new IllegalArgumentException("Bucket size, x, must be: x > 1");
		}

		this.buckets = new double[bucketValues.length];

		setValue(bucketValues);
	}

	/**
	 * Creates a new DiscreteBayesian opinion with the given set of support values
	 * and a known initial base rate
	 * 
	 * @param bucketValues set of support values
	 * @param baseRate     initial base rate
	 */
	public DiscreteBayesian(double[] bucketValues, double baseRate) {
		this(bucketValues);
		setBaseRate(baseRate);
	}

	/**
	 * Creates a new DiscreteBayesian opinion with a vector of support values of the
	 * given size
	 * 
	 * @param size size of the support values vector
	 */
	public DiscreteBayesian(int size) {
		if (size < 2) {
			throw new IllegalArgumentException("Bucket size, x, must be: x > 1");
		}

		this.buckets = new double[size];
	}

	/**
	 * Casts a generic Opinion to a DisscreteBayesian opinion and creates a copy of
	 * it with a support vector of the given size
	 * 
	 * @param o    the opinion to copy
	 * @param size the size of the support values vector
	 */
	public DiscreteBayesian(Opinion o, int size) {
		DiscreteBayesian x = null;
		if (o == null) {
			throw new NullPointerException("Opinion must not be null");
		}

		if (size < 2) {
			throw new IllegalArgumentException("Conversion not possible");
		}

		if ((o instanceof SubjectiveOpinion)) {
			x = ((SubjectiveOpinion) o).toDiscreteBayesian(size);
		} else {
			if ((o instanceof PureBayesian))
				x = ((PureBayesian) o).toDiscreteBayesian(size);
			else if ((o instanceof DiscreteBayesian))
				x = ((DiscreteBayesian) o).toDiscreteBayesian(size);
			else
				throw new ClassCastException("Opinion is not an instance of Bayesian, Subjective Opinion");
		}
		this.buckets = x.buckets;
		this.baseRate = x.baseRate;
		this.rs2 = x.rs2;
		this.expectation = x.expectation;
		this.recalculate = x.recalculate;
	}

	/**
	 * Returns this opinion base rate
	 */
	@Override
	public double getBaseRate() {
		return this.baseRate;
	}

	/**
	 * Updates this opinion's base rate with the given value
	 * 
	 * @param baseRate the new base rate
	 */
	public void setBaseRate(double baseRate) {
		double old = this.baseRate;

		if ((baseRate < 0.0D) || (baseRate > 1.0D)) {
			throw new IllegalArgumentException("baseRate, x, must be: 0 <= x <= 1");
		}

		if (!(baseRate == this.baseRate)) {

			synchronized (this) {
				this.baseRate = baseRate;
				this.recalculate = true;
			}

			this.changeSupport.firePropertyChange("baseRate", old, baseRate);
		}
	}

	/**
	 * Returns this opinion's expected probability
	 */
	@Override
	public double getExpectation() {
		double expectation = 0.0D;

		synchronized (this) {
			setDependants();
			expectation = this.expectation;
		}

		return expectation;
	}

	/**
	 * Returns this opinion's negative support value
	 */
	@Override
	public double getNegative() {
		setDependants();

		return this.negative;
	}

	/**
	 * Returns this opinon's positive support value
	 */
	@Override
	public double getPositive() {
		setDependants();

		return this.positive;
	}

	/**
	 * Returns the support value store in the indicated index
	 * 
	 * @param index position of the value within the support vetor
	 * @return the requested support value
	 * @throws IndexOutOfBoundsException
	 */
	public double getValue(int index) throws IndexOutOfBoundsException {
		if ((index < 0) || (index > this.buckets.length)) {
			throw new IndexOutOfBoundsException("Index is out of range");
		}
		synchronized (this) {
			return this.buckets[index];
		}
	}

	/**
	 * Returns this opinion's weight calculated from the support vector size
	 * 
	 * @return this opinion's weight
	 */
	public double getWeight() {
		double x = 0.0D;

		synchronized (this) {
			for (double bucket : this.buckets) {
				x += bucket;
			}
		}
		return x;
	}

	/**
	 * Recalculates and updates this opinion's parameters (r, s, a, e)
	 */
	private void setDependants() {
		synchronized (this) {
			if (this.recalculate) {
				this.rs2 = 2.0D;
				this.positive = 0.0D;
				this.negative = 0.0D;

				int i = 0;
				for (int size = this.buckets.length; i < size; i++) {
					this.rs2 += this.buckets[i];
					this.positive += i * this.buckets[i] / size - 1;
					this.negative += size - 1 - i * this.buckets[i] / size - 1;
				}

				this.expectation = ((this.positive + this.baseRate * 2.0D) / this.rs2);
				this.recalculate = false;
			}
		}
	}

	/**
	 * Updates this opinion's support vector with the given support values
	 * 
	 * @param values vector with the new support values
	 * @throws IndexOutOfBoundsException
	 */
	public void setValue(double[] values) throws IndexOutOfBoundsException {
		if (values.length != this.buckets.length) {
			throw new IndexOutOfBoundsException("Index is out of range");
		}

		for (double value : values) {
			if (value < 0.0D)
				throw new IllegalArgumentException("Value, x, must be: 0 <= x");
			if (Double.isNaN(value))
				throw new IllegalArgumentException("Value must not be a NaN");
		}

		boolean changed = false;

		synchronized (this) {
			for (int i = 0; i < values.length; i++) {
				if (this.buckets[i] != values[i]) {
					this.buckets[i] = Math.min(200000000000.0D, values[i]);
					changed = true;
				}
			}

			if (changed) {
				this.recalculate = true;
			}
		}

		if (changed)
			this.changeSupport.firePropertyChange("value", this, this);
	}

	/**
	 * Updates this opinion's support value at the given index with the passed
	 * support value
	 * 
	 * @param index the location of the value to update
	 * @param value the new value to store
	 * @throws IndexOutOfBoundsException
	 */
	public void setValue(int index, double value) throws IndexOutOfBoundsException {
		if ((index < 0) || (index > this.buckets.length)) {
			throw new IndexOutOfBoundsException("Index is out of range");
		}

		if (Double.isNaN(value)) {
			throw new IllegalArgumentException("Value must not be a NaN");
		}

		if (value < 0.0D) {
			throw new IllegalArgumentException("Value, x, must be: 0 <= x");
		}

		if (!(this.buckets[index] == value)) {

			synchronized (this) {
				this.buckets[index] = value;
				this.recalculate = true;
			}

			this.changeSupport.firePropertyChange("value", this, this);
		}
	}

	/**
	 * Returns the number of support evidences
	 * 
	 * @return the number of support values
	 */
	public synchronized int size() {
		return this.buckets.length;
	}

	/**
	 * 
	 * @return
	 */
	public boolean isPolarized() {
		return isPolarized(DEFAULT_POLARIZATION);
	}

	/**
	 * 
	 * @param tolerance
	 * @return
	 */
	public boolean isPolarized(double tolerance) {
		if ((tolerance < 0.0D) || (tolerance > 1.0D)) {
			throw new IllegalArgumentException("Threshold, x: 0 <= x <= 1");
		}

		double similarity = Math.log10(2.0D - tolerance);
		double scale = ROOT_2 * similarity;

		double lValue = 0.0D;
		double rValue = 0.0D;
		double mlValue = 0.0D;
		double mrValue = 0.0D;

		synchronized (this) {
			double n2 = this.buckets.length / 2.0D;
			int eSize = (int) Math.floor(n2);
			int mSize = eSize + (eSize + 1) % 2;

			int lEnd = eSize - 1;
			int rStart = this.buckets.length - eSize;

			int mStart = (this.buckets.length - mSize) / 2;
			int mEnd = mStart + mSize - 1;

			int i = 0;
			for (int size = this.buckets.length; i < size; i++) {
				if (i <= lEnd) {
					lValue += eSize - i * this.buckets[i] / eSize + 1.0D;
				} else if (i >= rStart) {
					rValue += i - (rStart - 1) * this.buckets[i] / eSize + 1.0D;
				}

				if ((size > 2) && (i >= mStart) && (i <= mEnd)) {
					if (i - mStart <= mEnd - i) {
						mlValue += this.buckets[i] * i - mStart + 1 / eSize + 1.0D;
					}
					if (i - mStart >= mEnd - i) {
						mrValue += this.buckets[i] * mEnd - i + 1 / eSize + 1.0D;
					}
				}

			}

		}

		double r_l = Math.abs(Math.log10(lValue) - Math.log10(rValue));
		double l_m = Math.log10(lValue) - Math.log10(mlValue);
		double r_m = Math.log10(rValue) - Math.log10(mrValue);

		return (r_l <= similarity) && (l_m > scale) && (r_m > scale);
	}

	/**
	 * Returns the highest value that exists in the support vector
	 */
	@Override
	public double max() {
		double max = 0.0D;

		for (int i = 0, size = this.buckets.length; i < size; i++) {
			if (this.buckets[i] > max)
				max = this.buckets[i];
		}

		return max;
	}

	/**
	 * Returns the lowest value that exists in the support vector
	 */
	@Override
	public double min() {
		double min = Double.MAX_VALUE;

		for (int i = 0, size = this.buckets.length; i < size; i++) {
			if (this.buckets[i] < min)
				min = this.buckets[i];
		}

		return min;
	}

	/**
	 * Returns this opinoion's support vector
	 */
	@Override
	public double[] values() {
		double[] values = new double[this.buckets.length];

		synchronized (this) {
			int i = 0;
			for (int size = this.buckets.length; i < size; i++) {
				values[i] = this.buckets[i];
			}
		}
		return values;
	}

	/**
	 * Returns a new instance of a DiscreteBayesian opinion with support vector of
	 * the indicated size
	 * 
	 * @param size the size of the new support vector
	 * @return a new DiscreteBayesian object
	 */
	public DiscreteBayesian toDiscreteBayesian(int size) {
		DiscreteBayesian o = null;

		if (size < 2) {
			throw new IllegalArgumentException("Conversion not possible");
		}

		synchronized (this) {
			if (this.buckets.length == size) {
				o = new DiscreteBayesian(this);
			} else {
				double[] results = convert(this.buckets, size);
				o = new DiscreteBayesian(results, this.baseRate);
			}
		}

		return o;
	}

	/**
	 * Returns this opinion as a SubjectiveOpinion instance
	 */
	@Override
	public SubjectiveOpinion toSubjectiveOpinion() {
		SubjectiveOpinion opinion = new SubjectiveOpinion();

		synchronized (this) {
			setDependants();
			double uncertainty = 2.0D / this.rs2;
			opinion.setBelief(this.positive / this.rs2, uncertainty);
			opinion.setBaseRate(this.baseRate);
		}

		return opinion;
	}

	/**
	 * Returns this opinion as a PureBayesian instance
	 */
	@Override
	public synchronized PureBayesian toPureBayesian() {
		PureBayesian bayesian = new PureBayesian();

		setDependants();

		bayesian.setPositive(this.positive);
		bayesian.setNegative(this.negative);
		bayesian.setBaseRate(this.baseRate);

		return bayesian;
	}

	public boolean equals(Object obj) {
		if ((obj != null) && ((obj instanceof DiscreteBayesian))) {
			DiscreteBayesian o = (DiscreteBayesian) obj;

			if ((o.baseRate == this.baseRate) && (o.buckets.length == this.buckets.length)) {
				for (int i = 0, size = this.buckets.length; i < size; i++) {
					if (this.buckets[i] != o.buckets[i])
						return false;
				}
				return true;
			}
		}

		return false;
	}

	public String toString() {
		synchronized (this) {
			setDependants();

			StringBuilder sb = new StringBuilder();

			for (double bucket : this.buckets) {
				if (sb.length() > 0) {
					sb.append(", ");
				}

				sb.append(String.format(FORMAT, bucket));
			}

			return String.format(EXTRAS_FORMAT, sb.toString(), this.baseRate, this.expectation);
		}
	}

	/////////////////////////
	/// General Operators ///
	/////////////////////////

	/**
	 * Returns the consensus of the given opinions
	 * 
	 * @param x the first opinion
	 * @param y the second opinion
	 * @return the consensus result as a DiscreteBayesian opinion
	 */
	private static DiscreteBayesian consensus(DiscreteBayesian x, DiscreteBayesian y)
			throws OpinionArithmeticException {
		if ((x == null) || (y == null)) {
			throw new NullPointerException("Opinions cannot be null");
		}

		if (x.size() != y.size()) {
			throw new OpinionArithmeticException("Discrete Bayesian beliefs must have the same size");
		}

		DiscreteBayesian o = new DiscreteBayesian(x.size());

		double sumx = 0.0D;
		double sumy = 0.0D;

		int i = 0;
		for (int size = x.size(); i < size; i++) {
			x.buckets[i] += y.buckets[i];
			sumx += x.buckets[i];
			sumy += y.buckets[i];
		}

		if (Math.abs(sumx - sumy) < 1.0E-010D)
			o.baseRate = x.baseRate;
		else {
			o.baseRate = ((sumx * x.baseRate + sumy * y.baseRate) / (sumx + sumy));
		}
		o.recalculate = true;

		return o;
	}

	/**
	 * Erodes an Opinion with the given factor
	 * 
	 * @param x      the opinion to erode
	 * @param factor the factor to use
	 * @return the eroded opinion
	 */
	private static DiscreteBayesian erosion(DiscreteBayesian x, double factor) {
		if (x == null) {
			throw new NullPointerException("Opinions cannot be null");
		}
		if ((factor < 0.0D) || (factor > 1.0D)) {
			throw new IllegalArgumentException("Erosion Factor, f must be: 0 <= f <= 1");
		}
		synchronized (x) {
			DiscreteBayesian o = new DiscreteBayesian();

			double f = 1.0D - factor;

			int i = 0;
			for (int size = o.size(); i < size; i++) {
				x.buckets[i] *= f;
			}
			o.baseRate = x.baseRate;

			o.recalculate = true;

			return o;
		}
	}

	/**
	 * Erodes this Opinion with the given factor
	 * 
	 * @param factor the factor to use
	 * @return the eroded opinion
	 */
	public final DiscreteBayesian erode(double factor) {
		return erosion(this, factor);
	}

	/**
	 * Calculates the decay value of this opinion
	 * 
	 * @param halfLife
	 * @param time
	 * @return the decayed DiscreteBayesian opinion
	 */
	public final DiscreteBayesian decay(double halfLife, double time) {
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
	private static double discountScale(PureBayesian discounter, DiscreteBayesian opinion) {
		double r = discounter.getPositive();
		double s = discounter.getNegative();
		//double w = discounter.getWeight(); // TODO this var not being used may be a bug!

		double divisor = (opinion.getWeight() + 2.0D) * (s + 2.0D) + 2.0D * r;
		return divisor == 0.0D ? 0.0D : 2.0D * r / divisor;
	}

	/**
	 * Calculates a discount scale from the given opinion using a SubjectiveOpinion
	 * discounter
	 * 
	 * @param discounter a SubjectiveOpinion opinion to use as discounter
	 * @param opinion    a opinion to be discounted
	 * @return the calculated discount scale
	 */
	private static double discountScale(SubjectiveOpinion discounter, DiscreteBayesian opinion) {
		double b = discounter.getBelief();
		double d = discounter.getDisbelief();
		double u = discounter.getUncertainty();

		double divisor = (d + u) * (opinion.getWeight() + 2.0D) + 2.0D * b;
		return divisor == 0.0D ? 0.0D : 2.0D * b / divisor;
	}

	/**
	 * Discounts the given discounter from the passed opinion
	 * 
	 * @param discounter the discounter to use
	 * @param opinion    the opinion to discount
	 * @return the new discounted opinion
	 */
	private static DiscreteBayesian discount(Opinion discounter, DiscreteBayesian opinion) {
		if ((discounter == null) || (opinion == null)) {
			throw new NullPointerException("Opinions cannot be null");
		}
		
		DiscreteBayesian y = new DiscreteBayesian(opinion);
		DiscreteBayesian o = new DiscreteBayesian(y.size());
		
		double scale;
		if ((discounter instanceof PureBayesian)) {
			scale = discountScale(new PureBayesian(discounter), opinion);
		} else {
			if ((discounter instanceof DiscreteBayesian))
				scale = discountScale(discounter.toPureBayesian(), opinion);
			else
				scale = discountScale(new SubjectiveOpinion(discounter), opinion);
		}
		
		for (int i = 0, size = o.size(); i < size; i++) {
			y.buckets[i] *= scale;
		}
		o.recalculate = true;

		return o;
	}

	/**
	 * Discounts by given opinion this instance
	 * 
	 * @param opinion the opinion to discount
	 * @return the discount result as a PureBayesian opinion
	 */
	public final DiscreteBayesian discountBy(Opinion opinion) {
		return discount(opinion, this);
	}

	public void addScalar(double value) {
		if (Double.isNaN(value)) {
			throw new IllegalArgumentException("Value must not be a NaN");
		}
		
		if ((value < 0.0D) || (value > 1.0D)) {
			throw new IllegalArgumentException("Value, x, must be: 0 <= x <= 1");
		}
		
		setValue((int) Math.floor(value * this.buckets.length), 1.0D);
	}

	private double[] convert(double[] array, int size) {
		if (array == null) {
			throw new NullPointerException("Array must not be null");
		}
		
		if (size < 2) {
			throw new IllegalArgumentException("Conversion not possible");
		}
		
		double[] results;
		if (size > array.length) {
			results = new double[array.length + 1];

			int targetLength = results.length - 2;
			int k = array.length;
			double slip = Math.min(array[0], array[(k - 1)]) / k + 1;

			results[1] = slip;
			array[0] -= slip;

			results[(results.length - 2)] += slip;
			array[(array.length - 1)] -= slip;

			int i = 1;
			for (int count = array.length - 1; i < count; i++) {
				results[i] += targetLength - i * array[i] / targetLength;
				results[(i + 1)] += i * array[i] / targetLength;
			}

		} else {
			results = new double[array.length - 1];

			int targetLength = results.length;

			int i = 0;
			for (int count = results.length; i < count; i++) {
				results[i] += targetLength - i * array[i] / targetLength;
				results[i] += i + 1 * array[(i + 1)] / targetLength;
			}
		}

		if (results.length == size) {
			int i = 0;
			for (int count = results.length; i < count; i++) {
				results[i] = OpinionBase.adjust(results[i]);
			}
			return results;
		}

		return convert(results, size);
	}

	//////////////////////////
	/// Bayesian Operators ///
	//////////////////////////

	/**
	 * Returns the complement (negation) of this Opinion
	 * 
	 * @return the complement as a DiscreteBayesian opinion
	 */
	public DiscreteBayesian not() {
		DiscreteBayesian o = new DiscreteBayesian(size());

		synchronized (this) {
			int size = (int) Math.floor(this.buckets.length / 2);

			for (int i = 0; i < size; i++) {
				double tmp = this.buckets[i];
				int j = this.buckets.length - i;
				this.buckets[i] = this.buckets[j];
				this.buckets[j] = tmp;
			}

			o.baseRate = (1.0D - this.baseRate);
		}

		return o;
	}

	////////////////////
	/// SL Operators ///
	////////////////////

	/**
	 * Fuses this opinion with the given opinion. The fusion of DiscreteBayesian
	 * opinions is implemented as a consensus operation
	 * 
	 * @param opinion the opinion to fuse
	 * @return the fusion result as a Discreteayesian opinion
	 */
	public final DiscreteBayesian fuse(DiscreteBayesian opinion) throws OpinionArithmeticException {
		return consensus(new DiscreteBayesian(this), new DiscreteBayesian(opinion));
	}

	/**
	 * Fuses all the given opinions
	 * 
	 * @param opinions the opinions to fuse
	 * @return the fusion result as a DiscreteBayesian opinion
	 */
	public static final DiscreteBayesian fuse(Collection<? extends DiscreteBayesian> opinions)
			throws OpinionArithmeticException {
		if (opinions == null) {
			throw new NullPointerException("Opinions cannot be null");
		}
		
		if (opinions.isEmpty()) {
			throw new OpinionArithmeticException("Opinions must not be empty");
		}
		
		DiscreteBayesian x = null;

		if (!opinions.isEmpty()) {
			
			for (DiscreteBayesian opinion : opinions) {
				if (opinion != null)
					x = x == null ? opinion : x.fuse(opinion);
			}
		}
		
		return x;
	}
}
