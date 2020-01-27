/**
 * 
 */
package de.tum.i4.subjectivelogic;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Set;

import de.tum.i4.subjectivelogic.Conditionals;
import de.tum.i4.subjectivelogic.Opinion;
import de.tum.i4.subjectivelogic.OpinionArithmeticException;
import de.tum.i4.subjectivelogic.OpinionBase;
import de.tum.i4.subjectivelogic.OpinionOperator;
import de.tum.i4.subjectivelogic.SubjectiveOpinion;

/**
 * @author squijano
 *
 */
public class SubjectiveOpinion extends OpinionBase {

	private static final long serialVersionUID = 320108885350778185L;

	/**
	 * String format representation of a Subjective Opinion
	 */
	private static final String TO_STRING_FORMAT = "w=(belief=%1$1.3f, disbelief=%2$1.3f, uncertainty=%3$1.3f, base rate=%4$1.3f, e=%5$1.3f, rd=%6$1.3f)";

	/**
	 * Totally uncertain opinion, that is, the amount of uncertainty is 1.0
	 */
	public static final SubjectiveOpinion UNCERTAIN = new SubjectiveOpinion(0.0D, 0.0D, 1.0D, 0.5D);

	/**
	 * Base rate distribution
	 */
	private double baseRate = 0.5D;

	/**
	 * Belief mass in support of variables/propositions being true
	 */
	private double belief = 0.0D;

	/**
	 * Belief mass in support of variables/propositions being false
	 */
	private double disbelief = 0.0D;

	/**
	 * Uncertainty mass, that is amount of confidence over an observation
	 * 
	 */
	private double uncertainty = 1.0D;

	/**
	 * Expected (projected) probability associated to this opinion. Cached value
	 */
	private double cachedExpectation = 0.05D;

	/**
	 * 
	 */
	private double relativeWeight = 1.0D;

	/**
	 * Flag to recalculate opinion params in case of a property change is triggered
	 */
	private boolean recalculate = false;

	/**
	 * Last operator applied to this SubjectiveOpinion
	 */
	private OpinionOperator lastOp = null;

	public SubjectiveOpinion() {

	}

	/**
	 * Creates a new SubjectiveOpinion with an initial base rate distribution
	 * 
	 * @param baseRate the base rate distribution
	 */
	public SubjectiveOpinion(double baseRate) {
		this.baseRate = baseRate;
	}

	/**
	 * Creates a new SubjectiveOpinion with an initial belief mass
	 * 
	 * @param belief   the initial belief mass
	 * @param dogmatic flag to create a dogmatic opinion
	 */
	public SubjectiveOpinion(double belief, boolean dogmatic) {
		setBelief(belief, dogmatic);
	}

	/**
	 * Creates a new SubjectiveOpinion with an initial belief mass and base rate
	 * distribution
	 * 
	 * @param belief   the initial belief mass
	 * @param dogmatic flag to create a dogmatic opinion
	 * @param baseRate the initial base rate distribution
	 */
	public SubjectiveOpinion(double belief, boolean dogmatic, double baseRate) {
		setBelief(belief, dogmatic);
		setBaseRate(baseRate);
	}

	/**
	 * Creates a new SubjectiveOpinion with an initial belief mass and uncertainty
	 * mass
	 * 
	 * @param belief      the belief mass
	 * @param uncertainty the uncertainty mass
	 */
	public SubjectiveOpinion(double belief, double uncertainty) {
		setBelief(belief, uncertainty);
	}

	/**
	 * Creates a new SubjectiveOpinion with an initial belief, disbelief and
	 * uncertainty masses
	 * 
	 * @param belief      the belief mass
	 * @param disbelief   the disbelief mass
	 * @param uncertainty the uncertainty mass
	 */
	public SubjectiveOpinion(double belief, double disbelief, double uncertainty) {
		set(belief, disbelief, uncertainty);
	}

	/**
	 * Creates a new SubjectiveOpinion with an initial belief, disbelief,
	 * uncertainty masses and base rate distribution
	 * 
	 * @param belief      the belief mass
	 * @param disbelief   the disbelief mass
	 * @param uncertainty the uncertainty mass
	 */
	public SubjectiveOpinion(double belief, double disbelief, double uncertainty, double baseRate) {
		this(belief, disbelief, uncertainty);
		setBaseRate(baseRate);
	}

	/**
	 * Creates a new SubjectiveOpinion from the given Opinion
	 * 
	 * @param o reference opinion
	 */
	public SubjectiveOpinion(Opinion o) {
		SubjectiveOpinion x = null;

		if (o == null) {
			throw new NullPointerException("Opinion cannot be null");
		}

		x = o.toSubjectiveOpinion();

		this.belief = x.getBelief();
		this.disbelief = x.getDisbelief();
		this.uncertainty = x.getUncertainty();
		this.baseRate = x.getBaseRate();
		this.cachedExpectation = x.getExpectation();
		this.relativeWeight = x.relativeWeight;
		this.lastOp = x.lastOp;
		this.recalculate = x.recalculate;
	}

	/**
	 * Copy constructor
	 * 
	 * @param o SubjectiveOpinion to copy
	 */
	public SubjectiveOpinion(SubjectiveOpinion o) {
		if (o == null) {
			throw new NullPointerException("Opinion cannot be null");
		}

		synchronized (o) {
			o.setDependants();

			this.belief = o.getBelief();
			this.disbelief = o.getDisbelief();
			this.uncertainty = o.getUncertainty();
			this.baseRate = o.getBaseRate();
			this.cachedExpectation = o.getExpectation();
			this.relativeWeight = o.relativeWeight;
			this.lastOp = o.lastOp;
			this.recalculate = o.recalculate;
		}
	}

	/**
	 * Creates a new SubjectiveOpinion with the given projected probability and base
	 * rate distribution. The uncertainty mass is maximized
	 * 
	 * @param P        the opinion's projected probability
	 * @param baseRate the base rate distribution
	 * @return a new SubjectiveOpinion instance
	 */
	public static SubjectiveOpinion fromProjection(double P, double baseRate) {
		if (P < 0 || P > 1) {
			throw new IllegalArgumentException("Projected probability, P, must be: 0<=P<=1");
		}

		if (baseRate < 0 || baseRate > 1) {
			throw new IllegalArgumentException("Projected probability, a, must be: 0<=a<=1");
		}

		SubjectiveOpinion so = new SubjectiveOpinion(P, 1 - P, 0.0D, baseRate);
		so.maximizeUncertainty();

		return so;
	}

	/**
	 * Creates a new SubjectiveOpinion with the given projected probability. The
	 * base rate is assumed to be 0.5 and the uncertainty mass is maximized
	 * 
	 * @param P the opinion's projected probability
	 * @return the new SubjectiveOpinon instance
	 */
	public static SubjectiveOpinion fromProjection(double P) {
		if (P < 0 || P > 1) {
			throw new IllegalArgumentException("Projected probability, P must be: 0<=P<=1");
		}

		return fromProjection(P, 0.5D);
	}

	/**
	 * Returns a completely uncertain opinion. An uncertain opinion has a maximized
	 * uncertainty mass
	 * 
	 * @return an uncertain SubjectiveOpinion
	 */
	public SubjectiveOpinion uncertainOpinion() {
		SubjectiveOpinion so = new SubjectiveOpinion(this);

		so.maximizeUncertainty();

		return so;
	}

	/**
	 * Returns a dogmatic opinion
	 * 
	 * @return a dogmatic SubjectiveOpinion
	 */
	public SubjectiveOpinion dogmaticOpinion() {
		SubjectiveOpinion so = new SubjectiveOpinion(this);

		so.setBelief(so.getExpectation(), true);

		return so;
	}

	/**
	 * Returns a new dogmatic opinion with the given projected probability and known
	 * base rate
	 * 
	 * @param expectation the projected probability
	 * @param baseRate    the known base rate
	 * @return a new dogmatic SubjectiveOpinion
	 */
	public static SubjectiveOpinion newDogmaticOpinion(double expectation, double baseRate) {
		if ((expectation < 0.0D || (expectation > 1.0D))) {
			throw new IllegalArgumentException("Expectation e, must be 0<=e<=1");
		}

		if ((baseRate < 0.0D || (baseRate > 1.0D))) {
			throw new IllegalArgumentException("Base rate a, must be 0<=a<=1");
		}

		return new SubjectiveOpinion(expectation, 1.0D - expectation, 0.0D, baseRate);
	}

	/**
	 * Returns a new vacuous opinion with the expectation value as base rate
	 * 
	 * @param expectation expectation value as base rate
	 * @return a new vacuous SubjectiveOpinion
	 */
	public static SubjectiveOpinion newVacuousOpinion(double expectation) {
		if ((expectation < 0.0D) || (expectation > 1.0D)) {
			throw new IllegalArgumentException("Expectation e, must be 0<=e<=1");
		}

		return new SubjectiveOpinion(0.0D, 0.0D, 1.0D, expectation);
	}

	/**
	 * Recalculates and updates this opinion's parameters (b, d, u, a, p)
	 */
	private synchronized void setDependants() {
		if (this.recalculate) {
			this.belief = OpinionBase.constrain(OpinionBase.adjust(this.getBelief()));
			this.disbelief = OpinionBase.constrain(OpinionBase.adjust(this.getDisbelief()));
			this.uncertainty = OpinionBase.constrain(OpinionBase.adjust(this.getUncertainty()));
			this.baseRate = OpinionBase.constrain(OpinionBase.adjust(this.getBaseRate()));
			this.cachedExpectation = OpinionBase
					.constrain(OpinionBase.adjust(this.getBelief() + this.getBaseRate() * this.getUncertainty()));
			this.recalculate = false;
		}
	}
	
	/**
	 * Recalculates and updates this opinion's parameters (b, d, u, a, p)
	 * 
	 * @param force flag to force the params update
	 */
	private synchronized void setDependants(boolean force) {
		if (force) {
			this.recalculate = true;
		}
		setDependants();
	}

	/**
	 * Updates this instance with the given parameters
	 * 
	 * @param belief      the new belief mass distribution
	 * @param disbelief   the new disbelief mass
	 * @param uncertainty the new uncertainty mass
	 */
	public final void set(double belief, double disbelief, double uncertainty) {
		if ((this.getBelief() < 0.0D) || (this.getDisbelief() < 0.0D) || (this.getUncertainty() < 0.0D)) {
			throw new IllegalArgumentException("Belief, Disbelief and Uncertainty, x,  must be: 0 <= x");
		}

		double bdu = belief + disbelief + uncertainty;
		setBelief(belief / bdu, uncertainty / bdu);
	}

	/**
	 * Updates this instance with the given opinion values
	 * 
	 * @param o the reference opinion
	 */
	public synchronized void set(Opinion o) {
		Opinion old = null;

		if (o == null) {
			throw new NullPointerException("Opinion cannot be null");
		}

		if (!o.equals(this)) {
			old = new SubjectiveOpinion(this);

			SubjectiveOpinion so = o.toSubjectiveOpinion();

			synchronized (so) {
				this.belief = so.getBelief();
				this.disbelief = so.getDisbelief();
				this.uncertainty = so.getUncertainty();
				this.baseRate = so.getBaseRate();
				this.cachedExpectation = so.getExpectation();
				this.recalculate = so.recalculate;
				this.lastOp = so.lastOp;
				this.relativeWeight = so.relativeWeight;
			}

			this.changeSupport.firePropertyChange("opinion", old, this);
		}
	}

	/**
	 * Returns this base rate distribution
	 */
	@Override
	public final double getBaseRate() {
		return this.baseRate;
	}

	/**
	 * Updates this opinion base rate distribution
	 * 
	 * @param baseRate the new base rate distribution
	 */
	public final void setBaseRate(double baseRate) {
		double old = this.baseRate;

		if ((baseRate < 0.0D) || (baseRate > 1.0D)) {
			throw new IllegalArgumentException("Base rate a, must be 0<=a<=1");
		}

		if (old != baseRate) {
			synchronized (this) {
				this.baseRate = baseRate;
				this.recalculate = true;
			}

			this.changeSupport.firePropertyChange("base rate", old, baseRate);
		}
	}

	/**
	 * Returns this belief mass
	 */
	public final double getBelief() {
		return this.belief;
	}

	/**
	 * Sets this instance belief and uncertainty masses
	 * 
	 * @param belief      the belief mass to use
	 * @param uncertainty the uncertainty mass to use
	 */
	protected void setBelief(double belief, double uncertainty) {
		Opinion old = null;

		if ((belief < 0.0D) || (belief > 1.0D)) {
			throw new IllegalArgumentException("Belief mass b, must be 0<=b<=1");
		}

		if ((uncertainty < 0.0D) || (uncertainty > 1.0D)) {
			throw new IllegalArgumentException("Uncertainty mass u, must be 0<=u<=1");
		}

		if (belief + uncertainty - 1.0D > 1.0E-010D) {
			throw new IllegalArgumentException("Belief b, Uncertainty u, must be: (b + u)<=1");
		}

		if (!((belief == this.getBelief()) && (uncertainty == this.getUncertainty()))) {
			old = new SubjectiveOpinion(this);

			synchronized (this) {
				this.belief = belief;
				this.uncertainty = uncertainty;
				this.disbelief = (1.0D - (belief + uncertainty));
				this.recalculate = true;
			}

			this.changeSupport.firePropertyChange("opinion", old, this);
		}
	}

	/**
	 * Sets the belief mass
	 * 
	 * @param belief   belief mass value
	 * @param dogmatic flag to set this instance as a dogmatic opinion
	 */
	private void setBelief(double belief, boolean dogmatic) {
		if (dogmatic) {
			setBelief(belief, 0.0D);
		} else {
			setBelief(belief, 1.0D - belief);
		}
	}

	/**
	 * Updates this belief mass
	 * 
	 * @param belief the new belief mass
	 */
	@SuppressWarnings("unused")
	private void setBelief(double belief) {
		setBelief(belief, false);
	}

	/**
	 * Returns this certainty mass value
	 */
	public final double getCertainty() {
		double certainty = 0.0D;

		if (this.getUncertainty() == 0.0D) {
			certainty = 0.0D;
		}

		certainty = OpinionBase.adjust(1.0D - this.getUncertainty());

		return certainty;
	}

	/**
	 * Returns this disbelief mass
	 */
	public final double getDisbelief() {
		return this.disbelief;
	}

	/**
	 * Sets this instance disbelief and uncertainty masses
	 * 
	 * @param disbelief   the disbelief mass to use
	 * @param uncertainty the uncertainty mass to use
	 */
	private void setDisbelief(double disbelief, double uncertainty) {
		Opinion old = null;

		if ((disbelief < 0.0D) || (disbelief > 1.0D)) {
			throw new IllegalArgumentException("Disbelief mass d, must be 0<=d<=1");
		}

		if ((uncertainty < 0.0D) || (uncertainty > 1.0D)) {
			throw new IllegalArgumentException("Uncertainty mass u, must be 0<=u<=1");
		}

		if (disbelief + uncertainty - 1.0D > 1.0E-010D) {
			throw new IllegalArgumentException("Disbelief d, Uncertainty u, must be: (d + u)<=1");
		}

		if (!((disbelief == this.getDisbelief()) && (uncertainty == this.getUncertainty()))) {
			old = new SubjectiveOpinion(this);

			synchronized (this) {
				this.disbelief = disbelief;
				this.uncertainty = uncertainty;
				this.belief = (1.0D - (disbelief + uncertainty));
				this.recalculate = true;
			}

			this.changeSupport.firePropertyChange("opinion", old, this);
		}
	}

	/**
	 * Sets the disbelief mass
	 * 
	 * @param disbelief disbelief mass value
	 * @param dogmatic  flag to set this instance as a dogmatic opinion
	 */
	private void setDisbelief(double disbelief, boolean dogmatic) {
		if (dogmatic) {
			setDisbelief(disbelief, 0.0D);
		} else {
			setDisbelief(disbelief, 1.0D - disbelief);
		}
	}

	/**
	 * Updates this disbelief mass
	 * 
	 * @param belief the new disbelief mass
	 */
	private void setDisbelief(double disbelief) {
		setDisbelief(disbelief, false);
	}

	/**
	 * Returns this expected probability
	 */
	@Override
	public final double getExpectation() {
		double expectation = 0.5D;

		synchronized (this) {
			setDependants();
			expectation = this.cachedExpectation;
		}

		return expectation;
	}

	/**
	 * Adjusts the expected probability within the given opinion
	 * 
	 * @param x           a SubjectiveOpinion
	 * @param expectation the expected probability to adjust
	 */
	private static void adjustExpectation(SubjectiveOpinion x, double expectation) {
		synchronized (x) {
			x.setDependants();

			double new_e = OpinionBase.constrain(OpinionBase.adjust(expectation));

			if (Math.abs(new_e - x.getExpectation()) <= 1.0E-010D) {
				return;
			}
			if ((new_e == 0.0D) || (new_e == 1.0D)) {
				x.setBelief(new_e, true);
			} else if (new_e < x.getExpectation()) {
				double new_d = OpinionBase.adjust(((1.0D - new_e) * (x.getBelief() + x.getUncertainty())
						- (1.0D - x.getBaseRate()) * x.getUncertainty()) / x.getExpectation());
				double new_u = OpinionBase.adjust(new_e * x.getUncertainty() / x.getExpectation());

				if (new_d + new_u > 1.0D) {
					new_u = 1.0D - new_d;
				}
				x.setDisbelief(new_d, new_u);
			} else {
				double divisor = x.getDisbelief() + x.getUncertainty() - x.getBaseRate() * x.getUncertainty();
				double new_b = OpinionBase
						.adjust((new_e * (x.getDisbelief() + x.getUncertainty()) - x.getBaseRate() * x.getUncertainty())
								/ divisor);
				double new_u = OpinionBase.adjust((1.0D - new_e) * x.getUncertainty() / divisor);

				if (x.getBelief() + new_u > 1.0D) {
					new_u = 1.0D - new_b;
				}
				x.setBelief(new_b, new_u);
			}
		}
	}

	/**
	 * Adjusts the given expected probability
	 * 
	 * @param expectation the expected probability to adjust
	 * @return a new SubjectiveOpinion instance with adjusted expectation
	 */
	public SubjectiveOpinion adjustExpectation(double expectation) {
		SubjectiveOpinion so = new SubjectiveOpinion(this);

		adjustExpectation(so, expectation);

		return so;
	}

	/**
	 * Adjusts the expected probability of the given opinion
	 * 
	 * @param opinion opinion to adjust
	 * @return a new SubjectiveOpinion instance with adjusted expectation
	 */
	public SubjectiveOpinion adjustExpectation(Opinion opinion) {
		if (opinion == null) {
			throw new NullPointerException("Opinion cannot be null");
		}

		return adjustExpectation(opinion.getExpectation());
	}

	/**
	 * Returns this opinion's relative weight
	 */
	public double getRelativeWeight() {
		double relativeWeight = 0.0D;

		if (this.isDogmatic()) {
			relativeWeight = this.relativeWeight;
		}

		return relativeWeight;
	}

	/**
	 * Returns the relative weight of this opinion in respect to the given
	 * SubjectiveOpinon
	 * 
	 * @param so the SubjectiveOpinion to use
	 * @param op the operator to take into account
	 * @return the resulting relative weight
	 */
	public double getRelativeWeight(SubjectiveOpinion so, OpinionOperator op) {
		double relativeWeight = 1.0D;

		if (so == null) {
			throw new NullPointerException("Opinion cannot be null");
		}

		if ((op != null) && (this.lastOp == op) && (op.isAssociative())) {
			relativeWeight = this.relativeWeight / so.relativeWeight;
		}

		return relativeWeight;
	}

	/**
	 * Updates this relative weight
	 * 
	 * @param weight the new relative weight value
	 */
	private void setRelativeWeight(double weight) {
		double old = this.relativeWeight;

		if (!(weight == this.relativeWeight)) {
			this.relativeWeight = weight;

			this.changeSupport.firePropertyChange("relativeWeight", old, this.relativeWeight);
		}
	}

	/**
	 * Returns this uncertainty mass
	 */
	public final double getUncertainty() {
		return this.uncertainty;
	}

	/**
	 * Updates this uncertainty mass
	 * 
	 * @param u the new uncertainty mass
	 */
	public void setUncertainty(double u) {
		this.uncertainty = u;
	}

	/**
	 * Checks if this opinion is Absolute.
	 * <p>
	 * An opinion is absolute if the belief mass or disbelief mass is equal to 1.0
	 * 
	 * @return true if the opinion is absolute, false otherwise
	 */
	public boolean isAbsolute() {
		return (this.getBelief() == 1.0D) || (this.getDisbelief() == 1.0D);
	}

	/**
	 * Checks if this opinion is Vacuous.
	 * <p>
	 * An opinion is vacuous or completely uncertain, if the uncertainty mass is
	 * equal to 1.0
	 * 
	 * @return true if the opinion is vacuous, false otherwise
	 */
	public boolean isVacuous() {
		return this.getUncertainty() == 1.0D;
	}

	/**
	 * Checks if this opinion is Certain
	 * 
	 * @param threshold limit up to which the opinion can be considered certain
	 * @return true if the opinion is certain, false otherwise
	 */
	public boolean isCertain(double threshold) {
		return !isUncertain(threshold);
	}

	/**
	 * Checks if this opinion is Consistent
	 * 
	 * @return true if the opinion is consistent, false otherwise
	 */
	public boolean isConsistent() {
		boolean isConsistent = true;

		try {
			checkConsistency();
			isConsistent = true;
		} catch (OpinionArithmeticException oae) {
			isConsistent = false;
		}

		return isConsistent;
	}

	/**
	 * Checks if this opinions is Dogmatic.
	 * <p>
	 * An opinion is Dogmatic if the uncertainty mass is equal to 0.0
	 * 
	 * @return true if the opinion is dogmatic, false otherwise
	 */
	public boolean isDogmatic() {
		return this.getUncertainty() == 0.0D;
	}

	/**
	 * Checks if this opinion is Uncertain
	 * 
	 * @param threshold limit up to which the opinion can be considered as uncertain
	 * @return true if the opinion is uncertain, false otherwise
	 */
	public boolean isUncertain(double threshold) {
		return 1.0D - this.getUncertainty() < threshold;
	}

	/**
	 * Checks if this opinion's uncertainty mass is maximized
	 * 
	 * @return true if the uncertainty mass is maximized, false otherwise
	 */
	public boolean isUncertaintyMaximized() {
		return (this.getDisbelief() == 0.0D) || (this.getBelief() == 0.0D);
	}

	/**
	 * 
	 * @return
	 */
	public SubjectiveOpinion increasedUncertainty() {
		SubjectiveOpinion br = null;

		synchronized (this) {
			double sqrt_u = OpinionBase.adjust(Math.sqrt(this.getUncertainty()));
			double k = 1.0D - (sqrt_u - this.getUncertainty()) / (this.getBelief() + this.getDisbelief());

			double brBelief = OpinionBase.adjust(this.getBelief() * k);
			double brUncertainty = sqrt_u;
			double brDisbelief = OpinionBase.adjust(this.getDisbelief() * k);

			br = new SubjectiveOpinion(brBelief, brDisbelief, brUncertainty);
		}

		return br;
	}

	/**
	 * Maximizes the uncertainty mass of this opinion
	 */
	private void maximizeUncertainty() {
		synchronized (this) {
			
			this.setDependants();
			
			double b, d, u, a;
			
			if (this.getExpectation() <= this.getBaseRate()) {
				b = 0.0D;
				a = this.getBaseRate();
				if (this.getBaseRate() > 0.0D) {
					d = OpinionBase.adjust(1.0D - this.getUncertainty() - this.getBelief() / this.getBaseRate());
					u = OpinionBase.adjust(this.getUncertainty() + this.getBelief() / this.getBaseRate());
				} else {
					d = 0.0D;
					u = OpinionBase.adjust(1.0D - b);
				}
			} else {
				d = 0.0D;
				a = this.getBaseRate();
				if (this.getBaseRate() < 1.0D) {
					b = OpinionBase
							.adjust(1.0D - this.getUncertainty() - this.getDisbelief() / (1.0D - this.getBaseRate()));
					u = OpinionBase.adjust(this.getUncertainty() + this.getDisbelief() / (1.0D - this.getBaseRate()));
				} else {
					b = 0.0D;
					u = b;
				}
			}

			this.belief = b;
			this.disbelief = d;
			this.uncertainty = u;
			this.baseRate = a;

			this.checkConsistency(true);
		}
	}

	/**
	 * Normalizes expected probabilities of the given opinions
	 * 
	 * @param opinions the opinions to normalize
	 * 
	 * @deprecated
	 */
	public static void normalize(Collection<? extends SubjectiveOpinion> opinions) {
		double sum = 0.0D;

		if (opinions == null) {
			throw new NullPointerException("Opinions cannot be null");
		}
		if (opinions.isEmpty()) {
			throw new OpinionArithmeticException("Opinions must not be empty");
		}

		for (SubjectiveOpinion o : opinions) {
			sum += o.getExpectation();
		}
		for (SubjectiveOpinion o : opinions) {
			if (sum == 0.0D)
				o.setDisbelief(1.0D);
			else
				o.set(new SubjectiveOpinion(o.adjustExpectation(o.getExpectation() / sum)));
		}
	}

	/**
	 * Checks consistency of this opinion components (b, d, u, a)
	 * 
	 * @param recalculate flag to ask for this opinion's parameters recalculation
	 */
	private void checkConsistency(boolean recalculate) {
		synchronized (this) {
			if ((this.getBaseRate() < 0.0D) || (this.getBaseRate() > 1.0D)) {
				throw new OpinionArithmeticException("Base rate a, must be in range: 0<=a<=1");
			}

			if (recalculate) {
				this.belief = OpinionBase.constrain(OpinionBase.adjust(this.getBelief()));
				this.disbelief = OpinionBase.constrain(OpinionBase.adjust(this.getDisbelief()));
				this.uncertainty = OpinionBase.constrain(OpinionBase.adjust(this.getUncertainty()));

				double bdu = this.getBelief() + this.getDisbelief() + this.getUncertainty();
				if (Math.abs((bdu - 1.0D)) > 1.0E-010D) {
					this.belief = OpinionBase.constrain(OpinionBase.adjust(this.getBelief() / bdu));
					this.uncertainty = OpinionBase.constrain(OpinionBase.adjust(this.getUncertainty() / bdu));
					this.disbelief = 1.0D - (this.getBelief() + this.getUncertainty());
				}

				this.recalculate = true;
			} else {
				if ((this.getBelief() < 0.0D) || (this.getBelief() > 1.0D)) {
					throw new OpinionArithmeticException("Belief mass b, must be in range: 0<=b<=1");
				}

				if ((this.getDisbelief() < 0.0D) || (this.getDisbelief() > 1.0D)) {
					throw new OpinionArithmeticException("Disbelief mass d, must be in range: 0<=d<=1");
				}

				if ((this.getUncertainty() < 0.0D) || (this.getUncertainty() > 1.0D)) {
					throw new OpinionArithmeticException("Uncertainty mass u, must be in range: 0<=u<=1");
				}

				double bdu = this.getBelief() + this.getDisbelief() + this.getUncertainty();
				if (Math.abs(bdu - 1.0D) > 1.0E-010D) {
					throw new OpinionArithmeticException(
							"Belief b, disbelief d and uncertainty u do not add up to 1: b + d + u != 1");
				}
			}
		}
	}

	/**
	 * Checks consistency of this opinion components (b, d, u, a)
	 */
	private void checkConsistency() {
		checkConsistency(false);
	}

	/**
	 * Checks if the given instance is equal to this Opinion.
	 * 
	 */
	@Override
	public boolean equals(Object obj) {
		boolean eq = false;

		if (obj == null) {
			eq = false;
		} else {
			if ((obj instanceof SubjectiveOpinion)) {
				SubjectiveOpinion so = (SubjectiveOpinion) obj;

				eq = Math.abs(so.getBelief() - this.getBelief()) < 1.0E-010D;
				eq = eq && Math.abs(so.getDisbelief() - this.getDisbelief()) < 1.0E-010D;
				eq = eq && Math.abs(so.getUncertainty() - this.getUncertainty()) < 1.0E-010D;
				eq = eq && Math.abs(so.getBaseRate() - this.getBaseRate()) < 1.0E-010D;

			}
		}

		return eq;
	}

	@Override
	public int hashCode() {
		int hash = 7;
		hash = 71 * hash + (int) (Double.doubleToLongBits(this.getBaseRate())
				^ (Double.doubleToLongBits(this.getBaseRate()) >>> 32));
		hash = 71 * hash + (int) (Double.doubleToLongBits(this.getBelief())
				^ (Double.doubleToLongBits(this.getBelief()) >>> 32));
		hash = 71 * hash + (int) (Double.doubleToLongBits(this.getDisbelief())
				^ (Double.doubleToLongBits(this.getDisbelief()) >>> 32));
		hash = 71 * hash + (int) (Double.doubleToLongBits(this.getUncertainty())
				^ (Double.doubleToLongBits(this.getUncertainty()) >>> 32));
		return hash;
	}

	@Override
	public String toString() {
		return String.format(TO_STRING_FORMAT, this.getBelief(), this.getDisbelief(), this.getUncertainty(),
				this.getBaseRate(), this.getExpectation(), this.relativeWeight);
	}

	@Override
	public int compareTo(Opinion o) {
		return super.compareTo(o);
	}

	/**
	 * Returns this instance
	 */
	@Override
	public SubjectiveOpinion toSubjectiveOpinion() {
		return this;
	}

	/**
	 * Returns this opinion as a PureBayesian instance
	 */
	@Override
	public PureBayesian toPureBayesian() {
		PureBayesian opinion = new PureBayesian();

		synchronized (this) {
			if (this.getUncertainty() == 0.0D) {
				opinion.setPositive(Double.MAX_VALUE);
				opinion.setNegative(Double.MAX_VALUE);
			} else {
				double r = 2.0D * this.getBelief() / this.getUncertainty();
				double s = 2.0D * this.getDisbelief() / this.getUncertainty();

				opinion.setPositive(Double.isInfinite(r) ? Double.MAX_VALUE : r);
				opinion.setNegative(Double.isInfinite(s) ? Double.MAX_VALUE : s);
			}

			opinion.setBaseRate(this.getBaseRate());
		}

		return opinion;
	}

	/**
	 * Converts this opinion to a DiscreteBayesian opinion of the given size
	 * 
	 * @param size size of the new DiscreteBayesian opinion
	 * @return a DiscreteBayesian opinion
	 */
	public DiscreteBayesian toDiscreteBayesian(int size) {
		if (size < 2) {
			throw new IllegalArgumentException("Conversion not possible");
		}

		return this.toPureBayesian().toDiscreteBayesian(size);
	}

	//////////////////////////
	/// General Operations ///
	//////////////////////////

	/**
	 * Adjusts this opinion's belief, desbelief and uncertainty mass values
	 */
	private void adjust() {
		this.belief = OpinionBase.constrain(OpinionBase.adjust(this.getBelief()));
		this.disbelief = OpinionBase.constrain(OpinionBase.adjust(this.getDisbelief()));
		this.uncertainty = OpinionBase.constrain(OpinionBase.adjust(this.getUncertainty()));
	}

	/**
	 * 
	 * @param b
	 * @param u
	 * @param a
	 * @return
	 */
	private static SubjectiveOpinion clippedOpinion(double b, double u, double a) {
		if ((a < 0.0D) || (a > 1.0D)) {
			throw new OpinionArithmeticException("Base rate a, must be in range : 0 <= a <= 1");
		}

		double resBelief, resDisbelief, resUncertainty, resBaseRate = a;
		double e = OpinionBase.constrain(b + a * u);
		double sum = u + b;

		if (u < 0.0D) {
			resUncertainty = 0.0D;
			resBelief = e;
			resDisbelief = 1.0D - e;
		} else if (b < 0.0D) {
			resBelief = 0.0D;
			resUncertainty = e / a;
			resDisbelief = 1 - resUncertainty;
		} else if (sum > 1.0D) {
			if (a == 1.0D) {
				resDisbelief = 0.0D;
				resBelief = b / sum;
				resUncertainty = u / sum;
			} else {
				resDisbelief = 0.0D;
				if (a < 1.0D)
					resBelief = (e - a) / (1.0D - a);
				else
					resBelief = e;
				resUncertainty = 1.0D - resBelief;
			}
		} else {
			resBelief = b;
			resUncertainty = u;
			resDisbelief = 1.0D - b - u;
		}

		SubjectiveOpinion o = new SubjectiveOpinion(resBelief, resDisbelief, resUncertainty, resBaseRate);
		o.adjust();

		o.checkConsistency();
		o.recalculate = true;

		return o;
	}

	/**
	 * Returns the lowest expectation of this instance and the given opinion
	 * 
	 * @param other the reference opinion
	 * @return the opinion with the lowest expected probability
	 */
	public SubjectiveOpinion minimum(SubjectiveOpinion other) {
		SubjectiveOpinion so = this;

		if (this.getExpectation() < other.getExpectation())
			so = this;
		else
			so = other;

		return so;
	}

	/**
	 * Erodes an opinion with the given factor
	 * 
	 * @param x      the SubjectiveOpinion to erode
	 * @param factor the erosion factor
	 * @return the erosion result as a SubjectiveOpion
	 */
	private static SubjectiveOpinion erosion(SubjectiveOpinion x, double factor) {
		if (x == null) {
			throw new NullPointerException("Opinion cannot be null");
		}

		if ((factor < 0.0D) || (factor > 1.0D)) {
			throw new IllegalArgumentException("Erosion Factor, f must be: 0 <= f <= 1");
		}

		synchronized (x) {
			double f = 1.0D - factor;

			double oBelief = OpinionBase.constrain(OpinionBase.adjust(x.getBelief() * f));
			double oDisbelief = OpinionBase.constrain(OpinionBase.adjust(x.getDisbelief() * f));
			double oUncertainty = OpinionBase.constrain(OpinionBase.adjust(1.0D - oBelief - oDisbelief));
			double oAtomicity = x.getBaseRate();

			SubjectiveOpinion o = new SubjectiveOpinion(oBelief, oDisbelief, oUncertainty, oAtomicity);

			o.checkConsistency(true);

			return o;
		}
	}

	/**
	 * Erodes this SubjectiveOpinion with the given factor
	 * 
	 * @param factor the factor to use
	 * @return the eroded opinion
	 */
	public final SubjectiveOpinion erode(double factor) {
		return erosion(this, factor);
	}

	/**
	 * Calculates the decay value of this opinion
	 * 
	 * @param halfLife
	 * @param time
	 * @return the decayed SubjectiveOpinion
	 */
	public final SubjectiveOpinion decay(double halfLife, double time) {
		return erosion(this, OpinionBase.erosionFactorFromHalfLife(halfLife, time));
	}

	private static SubjectiveOpinion transitivity(SubjectiveOpinion x, SubjectiveOpinion y) {
		if ((x == null) || (y == null)) {
			throw new NullPointerException();
		}
		// TODO submit this code upstream: the old code does not correspond to the
		// book's description of transitivity.
		double e = x.getExpectation();
		double newBelief = e * y.getBelief();
		double newDisbelief = e * y.getDisbelief();
		double newUncertainty = 1 - e * (y.getDisbelief() + y.getBelief());
		double newAtomicity = y.getBaseRate();

		SubjectiveOpinion o = new SubjectiveOpinion(newBelief, newDisbelief, newUncertainty, newAtomicity);

		o.checkConsistency(true);

		o.lastOp = OpinionOperator.Discount;

		return o;
	}

	/** @deprecated */
	public final SubjectiveOpinion discount(Opinion opinion) {
		if (opinion == null) {
			throw new NullPointerException("Opinion must not be null");
		}
		return transitivity(new SubjectiveOpinion(this), new SubjectiveOpinion(opinion));
	}

	public final SubjectiveOpinion discountBy(Opinion opinion) {
		if (opinion == null) {
			throw new NullPointerException("Opinion must not be null");
		}
		return transitivity(new SubjectiveOpinion(opinion), new SubjectiveOpinion(this));
	}

	//////////////////////////
	/// Bayesian Operators ///
	//////////////////////////

	// TODO change this
	@SuppressWarnings("unused")
	private static SubjectiveOpinion simpleAnd(SubjectiveOpinion x, SubjectiveOpinion y) {
		if ((x == null) || (y == null)) {
			throw new NullPointerException("Opinion must not be null");
		}
		
		SubjectiveOpinion o = new SubjectiveOpinion();

		x.setDependants();
		y.setDependants();

		double divisor;
		o.belief = x.getBelief() * y.getBelief();
		
		o.disbelief = x.getDisbelief() + y.getDisbelief() - x.getDisbelief() * y.getDisbelief();
		
		o.uncertainty = x.getBelief() * y.getUncertainty() + y.getBelief() * x.getUncertainty()
				+ x.getUncertainty() * y.getUncertainty();
		
		divisor = x.getBelief() * y.getUncertainty() + y.getBelief() * x.getUncertainty()
				+ x.getUncertainty() * y.getUncertainty();
		
		if (divisor != 0.0D) {
			
			o.setBaseRate(((y.getBaseRate() * x.getBelief() * y.getUncertainty()
					+ x.getBaseRate() * y.getBelief() * x.getUncertainty()
					+ x.getBaseRate() * y.getBaseRate() * x.getUncertainty() * y.getUncertainty())
					/ (x.getBelief() * y.getUncertainty() + y.getBelief() * x.getUncertainty()
							+ x.getUncertainty() * y.getUncertainty())));
		
		} else if ((y.getUncertainty() == 0.0D) && (x.getUncertainty() == 0.0D) && (x.getDisbelief() != 1.0D)
				&& (y.getDisbelief() != 1.0D)) {
			
			o.setBaseRate(((y.getBaseRate() * x.getBelief() + x.relativeWeight * x.getBaseRate() * y.getBelief())
					/ (x.getBelief() + x.relativeWeight * y.getBelief())));
		
		} else if ((x.getDisbelief() == 1.0D) && (y.getUncertainty() != 0.0D)) {
			
			o.setBaseRate(((y.getBaseRate() * y.getUncertainty() + x.relativeWeight * x.getBaseRate() * y.getBelief()
					+ x.relativeWeight * x.getBaseRate() * y.getBaseRate() * y.getUncertainty())
					/ (y.getUncertainty() + x.relativeWeight - x.relativeWeight * y.getDisbelief())));
		
		} else if ((y.getDisbelief() == 1.0D) && (x.getUncertainty() != 0.0D)) {
			
			o.setBaseRate(((x.relativeWeight * y.getBaseRate() * x.getBelief() + x.getBaseRate() * x.getUncertainty()
					+ x.relativeWeight * x.getBaseRate() * y.getBaseRate() * x.getUncertainty())
					/ (x.relativeWeight + x.getUncertainty() - x.relativeWeight * x.getDisbelief())));
		
		} else if ((x.getDisbelief() == 1.0D) && (y.getUncertainty() == 0.0D)) {
			
			o.setBaseRate(((x.relativeWeight * y.getBaseRate() + x.getBaseRate() * y.getBelief())
					/ (x.relativeWeight + y.getBelief())));
		
		} else if ((y.getDisbelief() == 1.0D) && (x.getUncertainty() == 0.0D)) {
			
			o.setBaseRate(((y.getBaseRate() * x.getBelief() + x.relativeWeight * x.getBaseRate())
					/ (x.getBelief() + x.relativeWeight)));
		
		} else if ((x.getDisbelief() == 1.0D) && (y.getDisbelief() == 1.0D)) {
			
			o.setBaseRate(((y.relativeWeight * y.getBaseRate() + x.relativeWeight * x.getBaseRate()
					+ x.relativeWeight * y.relativeWeight * x.getBaseRate() * y.getBaseRate())
					/ (y.relativeWeight + x.relativeWeight + x.relativeWeight * y.relativeWeight)));
		
		} else {
			o.setBaseRate(0.5D);
		}

		o.checkConsistency(true);
		o.lastOp = OpinionOperator.SimpleAnd;
		o.setRelativeWeight(x.getRelativeWeight() + y.getRelativeWeight());

		return o;
	}

	// TODO change this
	@SuppressWarnings("unused")
	private static SubjectiveOpinion simpleOr(SubjectiveOpinion x, SubjectiveOpinion y) {
		if ((x == null) || (y == null)) {
			throw new NullPointerException("Opinion must not be null");
		}
		
		SubjectiveOpinion o = new SubjectiveOpinion();

		x.setDependants();
		y.setDependants();

		double divisor;
		o.belief = x.getBelief() + y.getBelief() - x.getBelief() * y.getBelief();
		
		o.disbelief = x.getDisbelief() * y.getDisbelief();
		
		o.uncertainty = x.getDisbelief() * y.getUncertainty() + y.getDisbelief() * x.getUncertainty()
				+ x.getUncertainty() * y.getUncertainty();
		
		divisor = x.getUncertainty() + y.getUncertainty() - x.getBelief() * y.getUncertainty()
				- y.getBelief() * x.getUncertainty() - x.getUncertainty() * y.getUncertainty();
		
		if (divisor != 0.0D) {
			
			o.setBaseRate(((x.getUncertainty() * x.getBaseRate() + y.getUncertainty() * y.getBaseRate()
					- y.getBaseRate() * x.getBelief() * y.getUncertainty()
					- x.getBaseRate() * y.getBelief() * x.getUncertainty()
					- x.getBaseRate() * y.getBaseRate() * x.getUncertainty() * y.getUncertainty())
					/ (x.getUncertainty() + y.getUncertainty() - x.getBelief() * y.getUncertainty()
							- y.getBelief() * x.getUncertainty() - x.getUncertainty() * y.getUncertainty())));
		
		} else if ((y.getUncertainty() == 0.0D) && (x.getUncertainty() == 0.0D) && (x.getDisbelief() != 0.0D)
				&& (y.getDisbelief() != 0.0D)) {
			
			o.setBaseRate(((x.relativeWeight * x.getBaseRate() * y.getDisbelief() + y.getBaseRate() * x.getDisbelief())
					/ (x.relativeWeight * y.getDisbelief() + x.getDisbelief())));
		
		} else if ((x.getBelief() == 1.0D) && (y.getUncertainty() != 0.0D)) {
			
			o.setBaseRate(((x.relativeWeight * x.getBaseRate() * y.getDisbelief()
					+ x.relativeWeight * x.getBaseRate() * y.getUncertainty()
					+ x.relativeWeight * y.getUncertainty() * y.getBaseRate() + y.getUncertainty() * y.getBaseRate()
					- x.relativeWeight * x.getBaseRate() * y.getBaseRate() * y.getUncertainty())
					/ (x.relativeWeight + y.getUncertainty() - x.relativeWeight * y.getBelief())));
		
		} else if ((y.getBelief() == 1.0D) && (x.getUncertainty() != 0.0D)) {
			
			o.setBaseRate(((x.relativeWeight * x.getUncertainty() * x.getBaseRate()
					+ x.getUncertainty() * x.getBaseRate() + x.relativeWeight * y.getBaseRate() * x.getDisbelief()
					+ x.relativeWeight * y.getBaseRate() * x.getUncertainty()
					- x.relativeWeight * x.getBaseRate() * y.getBaseRate() * x.getUncertainty())
					/ (x.getUncertainty() + x.relativeWeight - x.relativeWeight * x.getBelief())));
		
		} else if ((x.getBelief() == 1.0D) && (y.getUncertainty() == 0.0D)) {
			
			o.setBaseRate(((x.getBaseRate() * y.getDisbelief() + x.relativeWeight * y.getBaseRate())
					/ (y.getDisbelief() + x.relativeWeight)));
		
		} else if ((y.getBelief() == 1.0D) && (x.getUncertainty() == 0.0D)) {
			
			o.setBaseRate(((x.relativeWeight * x.getBaseRate() + y.getBaseRate() * x.getDisbelief())
					/ (x.relativeWeight + x.getDisbelief())));
		
		} else if ((x.getBelief() == 1.0D) && (y.getBelief() == 1.0D)) {
			
			o.setBaseRate(((x.relativeWeight * y.relativeWeight * x.getBaseRate() + x.relativeWeight * x.getBaseRate()
					+ x.relativeWeight * y.relativeWeight * y.getBaseRate() + y.relativeWeight * y.getBaseRate()
					- x.relativeWeight * y.relativeWeight * x.getBaseRate() * y.getBaseRate())
					/ (x.relativeWeight + y.relativeWeight + x.relativeWeight * y.relativeWeight)));
		
		} else {
			o.setBaseRate(0.5D);
		}

		o.checkConsistency(true);
		o.lastOp = OpinionOperator.SimpleOr;
		o.setRelativeWeight(x.getRelativeWeight() + y.getRelativeWeight());

		return o;
	}

	private static SubjectiveOpinion sum(Collection<? extends Opinion> opinions) throws OpinionArithmeticException {
		if (opinions == null) {
			throw new NullPointerException("Opinions must not be null");
		}
		
		if (opinions.isEmpty()) {
			throw new OpinionArithmeticException("Opinions must not be empty");
		}
		
		double b = 0.0D;
		double u = 0.0D;
		double a = 0.0D;

		for (Opinion o : opinions) {
			SubjectiveOpinion so = o.toSubjectiveOpinion();

			b += so.getBelief();
			u += so.getUncertainty() * so.getBaseRate();
			a += so.getBaseRate();
		}

		if (a > 1.0D) {
			throw new OpinionArithmeticException("Illegal operation, Sum of base rates is greater than 1.0");
		}
		
		if (a > 0.0D) {
			u /= a;
		}
		
		SubjectiveOpinion o = clippedOpinion(b, u, a);

		o.lastOp = OpinionOperator.Add;

		return o;
	}

	private static SubjectiveOpinion sum(SubjectiveOpinion x, SubjectiveOpinion y) throws OpinionArithmeticException {
		if ((x == null) || (y == null)) {
			throw new NullPointerException("Opinions must not be null");
		}
		
		if (y.getBaseRate() + x.getBaseRate() > 1.0D) {
			throw new OpinionArithmeticException("Illegal operation, Sum of base rates is greater than 1.0");
		}
		
		double b = x.getBelief() + y.getBelief();
		double a = x.getBaseRate() + y.getBaseRate();
		double u = x.getBaseRate() * x.getUncertainty() + y.getBaseRate() * y.getUncertainty();

		if (a > 0.0D) {
			u /= a;
		}
		
		SubjectiveOpinion o = clippedOpinion(b, u, a);

		o.lastOp = OpinionOperator.Add;

		return o;
	}

	public final SubjectiveOpinion add(Opinion opinion) throws OpinionArithmeticException {
		if (opinion == null) {
			throw new NullPointerException("The Opinion must not be null");
		}
		
		return sum(new SubjectiveOpinion(this), new SubjectiveOpinion(opinion));
	}

	public static SubjectiveOpinion add(Collection<? extends Opinion> opinions) throws OpinionArithmeticException {
		return sum(opinions);
	}

	private static SubjectiveOpinion subtraction(SubjectiveOpinion x, SubjectiveOpinion y) {
		if ((x == null) || (y == null)) {
			throw new NullPointerException("Opinions must not be null");
		}
		
		if (x.getBaseRate() - y.getBaseRate() < 0.0D) {
			throw new OpinionArithmeticException("Illegal operation, Difference of atomicities is less than 0.0");
		}
		
		double b = x.getBelief() - y.getBelief();
		double a = OpinionBase.constrain(x.getBaseRate() - y.getBaseRate());
		double u = x.getBaseRate() * x.getUncertainty() - y.getBaseRate() * y.getUncertainty();

		if (a != 0.0D) {
			u /= a;
		}
		
		SubjectiveOpinion o = clippedOpinion(b, u, a);

		o.lastOp = OpinionOperator.Subtract;

		return o;
	}

	public final SubjectiveOpinion subtract(Opinion opinion) {
		if (opinion == null) {
			throw new NullPointerException("Opinion must not be null");
		}
		
		return subtraction(new SubjectiveOpinion(this), new SubjectiveOpinion(opinion));
	}

	private static SubjectiveOpinion complement(SubjectiveOpinion x) {
		if (x == null) {
			throw new NullPointerException("Opinion must not be null");
		}
		
		synchronized (x) {
			SubjectiveOpinion o = new SubjectiveOpinion(x.getDisbelief(), x.getBelief(), x.getUncertainty(),
					1.0D - x.getBaseRate());

			o.checkConsistency(true);

			o.lastOp = OpinionOperator.Not;

			return o;
		}
	}

	public final SubjectiveOpinion not() {
		return complement(this);
	}

	private static SubjectiveOpinion multiply(SubjectiveOpinion x, SubjectiveOpinion y) {
		if ((x == null) || (y == null)) {
			throw new NullPointerException("Opinions must not be null");
		}

		x.setDependants();
		y.setDependants();

		double divisor, expec;
		double r = x.getRelativeWeight(y, OpinionOperator.Or);

		double oDisbelief = x.getDisbelief() + y.getDisbelief() - x.getDisbelief() * y.getDisbelief();
		double oAtomicity = x.getBaseRate() * y.getBaseRate();
		expec = x.getExpectation() * (y.getBelief() + y.getBaseRate() * y.getUncertainty());
		divisor = 1.0D - oAtomicity;

		double oBelief, oUncertainty;
		if (divisor != 0.0D) {
			oBelief = ((oDisbelief - 1.0D) * oAtomicity + expec) / divisor;
			oUncertainty = -(oDisbelief - 1.0D + expec) / divisor;
		} else {
			oBelief = x.getBelief() * y.getBelief()
					+ (r * x.getBelief() * y.getUncertainty() + x.getUncertainty() * y.getBelief()) / (r + 1.0D);
			
			oUncertainty = (x.getBelief() * y.getUncertainty() + r * y.getBelief() * x.getUncertainty()) / (r + 1.0D)
					+ x.getUncertainty() * y.getUncertainty();
		}

		SubjectiveOpinion o = new SubjectiveOpinion(oBelief, oDisbelief, oUncertainty, oAtomicity);

		o.adjust();
		o.checkConsistency(true);

		o.lastOp = OpinionOperator.And;
		o.setRelativeWeight(x.getRelativeWeight() + y.getRelativeWeight());

		return o;
	}

	public final SubjectiveOpinion and(Opinion opinion) {
		if (opinion == null) {
			throw new NullPointerException("Opinion must not be null");
		}
		
		return multiply(new SubjectiveOpinion(this), new SubjectiveOpinion(opinion));
	}

	public static SubjectiveOpinion and(Collection<? extends Opinion> opinions) throws OpinionArithmeticException {
		if (opinions == null) {
			throw new NullPointerException("Opinions must not be null");
		}
		
		if (opinions.isEmpty()) {
			throw new OpinionArithmeticException("Opinions must not be empty");
		}
		
		SubjectiveOpinion x = null;

		for (Opinion opinion : opinions) {
			if (opinion != null)
				x = x == null ? new SubjectiveOpinion(opinion) : x.and(opinion);
		}
		return x;
	}

	public static SubjectiveOpinion or(Collection<? extends Opinion> opinions) throws OpinionArithmeticException {
		if (opinions == null) {
			throw new NullPointerException("Opinion must not be null");
		}
		
		if (opinions.isEmpty()) {
			throw new OpinionArithmeticException("Opinions must not be empty");
		}
		SubjectiveOpinion x = null;

		for (Opinion opinion : opinions) {
			if (opinion != null)
				x = x == null ? new SubjectiveOpinion(opinion) : x.or(opinion);
		}
		return x;
	}

	private static SubjectiveOpinion coMultiplication(SubjectiveOpinion x, SubjectiveOpinion y) {
		if ((x == null) || (y == null)) {
			throw new NullPointerException("Opinions must not be null");
		}

		double r = x.getRelativeWeight(y, OpinionOperator.Or);

		x.setDependants();
		y.setDependants();

		double oBelief = x.getBelief() + y.getBelief() - x.getBelief() * y.getBelief();
		double oAtomicity = x.getBaseRate() + y.getBaseRate() - x.getBaseRate() * y.getBaseRate();

		double oUncertainty;
		if (oAtomicity != 0.0D) {
			oUncertainty = x.getUncertainty() * y.getUncertainty()
					+ (y.getBaseRate() * x.getDisbelief() * y.getUncertainty()
							+ x.getBaseRate() * x.getUncertainty() * y.getDisbelief()) / oAtomicity;
		} else {
			oUncertainty = x.getUncertainty() * y.getUncertainty()
					+ (x.getDisbelief() * y.getUncertainty() + r * x.getUncertainty() * y.getDisbelief()) / (r + 1.0D);
		}

		double oDisbelief = 1.0D - oBelief - oUncertainty;

		SubjectiveOpinion o = new SubjectiveOpinion(oBelief, oDisbelief, oUncertainty, oAtomicity);
		o.checkConsistency();
		o.recalculate = true;

		o.lastOp = OpinionOperator.Or;
		o.setRelativeWeight(x.relativeWeight + y.relativeWeight);

		return o;
	}

	public final SubjectiveOpinion or(Opinion opinion) {
		if (opinion == null) {
			throw new NullPointerException("Opinion must not be null");
		}
		
		return coMultiplication(new SubjectiveOpinion(this), new SubjectiveOpinion(opinion));
	}

	// TODO change this
	@SuppressWarnings("unused")
	private static SubjectiveOpinion simpleMultiplication(SubjectiveOpinion x, SubjectiveOpinion y, double r,
			double s) {
		if (y == null) {
			throw new NullPointerException();
		}
		SubjectiveOpinion o = new SubjectiveOpinion();

		x.setDependants();
		y.setDependants();

		o.belief = x.getBelief() * y.getBelief();
		
		o.disbelief = x.getDisbelief() + y.getDisbelief() - x.getDisbelief() * y.getDisbelief();
		
		o.uncertainty = x.getBelief() * y.getUncertainty() + y.getBelief() * x.getUncertainty()
				+ x.getUncertainty() * y.getUncertainty();

		double divisor = x.getBelief() * y.getUncertainty() + y.getBelief() * x.getUncertainty()
				+ x.getUncertainty() * y.getUncertainty();

		if (divisor != 0.0D) {
			
			o.setBaseRate(((y.getBaseRate() * x.getBelief() * y.getUncertainty()
					+ x.getBaseRate() * y.getBelief() * x.getUncertainty()
					+ x.getBaseRate() * y.getBaseRate() * x.getUncertainty() * y.getUncertainty())
					/ (x.getBelief() * y.getUncertainty() + y.getBelief() * x.getUncertainty()
							+ x.getUncertainty() * y.getUncertainty())));
		
		} else if ((y.getUncertainty() == 0.0D) && (x.getUncertainty() == 0.0D) && (x.getDisbelief() != 1.0D)
				&& (y.getDisbelief() != 1.0D)) {
			
			o.setBaseRate(((y.getBaseRate() * x.getBelief() + r * x.getBaseRate() * y.getBelief())
					/ (x.getBelief() + r * y.getBelief())));
		
		} else if ((x.getDisbelief() == 1.0D) && (y.getUncertainty() != 0.0D)) {
			
			o.setBaseRate(((y.getBaseRate() * y.getUncertainty() + r * x.getBaseRate() * y.getBelief()
					+ r * x.getBaseRate() * y.getBaseRate() * y.getUncertainty())
					/ (y.getUncertainty() + r - r * y.getDisbelief())));
		
		} else if ((y.getDisbelief() == 1.0D) && (x.getUncertainty() != 0.0D)) {
			
			o.setBaseRate(((r * y.getBaseRate() * x.getBelief() + x.getBaseRate() * x.getUncertainty()
					+ r * x.getBaseRate() * y.getBaseRate() * x.getUncertainty())
					/ (r + x.getUncertainty() - r * x.getDisbelief())));
		
		} else if ((x.getDisbelief() == 1.0D) && (y.getUncertainty() == 0.0D)) {
			
			o.setBaseRate(((r * y.getBaseRate() + x.getBaseRate() * y.getBelief()) / (r + y.getBelief())));
		
		} else if ((y.getDisbelief() == 1.0D) && (x.getUncertainty() == 0.0D)) {
			
			o.setBaseRate(((y.getBaseRate() * x.getBelief() + r * x.getBaseRate()) / (x.getBelief() + r)));
		
		} else if ((x.getDisbelief() == 1.0D) && (y.getDisbelief() == 1.0D)) {
			
			o.setBaseRate(((s * y.getBaseRate() + r * x.getBaseRate() + r * s * x.getBaseRate() * y.getBaseRate())
					/ (s + r + r * s)));
		
		} else {
			o.setBaseRate(0.5D);
		}

		o.checkConsistency(true);

		return o;
	}

	// TODO change this
	@SuppressWarnings("unused")
	private static SubjectiveOpinion simpleCoMultiplication(SubjectiveOpinion x, SubjectiveOpinion y, double r,
			double s) {
		if ((x == null) || (y == null)) {
			throw new NullPointerException("Opinions must not be null");
		}
		
		SubjectiveOpinion o = new SubjectiveOpinion();

		x.setDependants();
		y.setDependants();

		double divisor;
		o.belief = x.getBelief() + y.getBelief() - x.getBelief() * y.getBelief();
		
		o.disbelief = x.getDisbelief() * y.getDisbelief();
		
		o.uncertainty = x.getDisbelief() * y.getUncertainty() + y.getDisbelief() * x.getUncertainty()
				+ x.getUncertainty() * y.getUncertainty();
		
		divisor = x.getUncertainty() + y.getUncertainty() - x.getBelief() * y.getUncertainty()
				- y.getBelief() * x.getUncertainty() - x.getUncertainty() * y.getUncertainty();

		if (divisor != 0.0D) {
			
			o.setBaseRate(((x.getUncertainty() * x.getBaseRate() + y.getUncertainty() * y.getBaseRate()
					- y.getBaseRate() * x.getBelief() * y.getUncertainty()
					- x.getBaseRate() * y.getBelief() * x.getUncertainty()
					- x.getBaseRate() * y.getBaseRate() * x.getUncertainty() * y.getUncertainty())
					/ (x.getUncertainty() + y.getUncertainty() - x.getBelief() * y.getUncertainty()
							- y.getBelief() * x.getUncertainty() - x.getUncertainty() * y.getUncertainty())));
		
		} else if ((y.getUncertainty() == 0.0D) && (x.getUncertainty() == 0.0D) && (x.getDisbelief() != 0.0D)
				&& (y.getDisbelief() != 0.0D)) {
			
			o.setBaseRate(((r * x.getBaseRate() * y.getDisbelief() + y.getBaseRate() * x.getDisbelief())
					/ (r * y.getDisbelief() + x.getDisbelief())));
		
		} else if ((x.getBelief() == 1.0D) && (y.getUncertainty() != 0.0D)) {
			
			o.setBaseRate(((r * x.getBaseRate() * y.getDisbelief() + r * x.getBaseRate() * y.getUncertainty()
					+ r * y.getUncertainty() * y.getBaseRate() + y.getUncertainty() * y.getBaseRate()
					- r * x.getBaseRate() * y.getBaseRate() * y.getUncertainty())
					/ (r + y.getUncertainty() - r * y.getBelief())));
		
		} else if ((y.getBelief() == 1.0D) && (x.getUncertainty() != 0.0D)) {
			
			o.setBaseRate(((r * x.getUncertainty() * x.getBaseRate() + x.getUncertainty() * x.getBaseRate()
					+ r * y.getBaseRate() * x.getDisbelief() + r * y.getBaseRate() * x.getUncertainty()
					- r * x.getBaseRate() * y.getBaseRate() * x.getUncertainty())
					/ (x.getUncertainty() + r - r * x.getBelief())));
		
		} else if ((x.getBelief() == 1.0D) && (y.getUncertainty() == 0.0D)) {
			
			o.setBaseRate(((x.getBaseRate() * y.getDisbelief() + r * y.getBaseRate()) / (y.getDisbelief() + r)));
		
		} else if ((y.getBelief() == 1.0D) && (x.getUncertainty() == 0.0D)) {
			
			o.setBaseRate(((r * x.getBaseRate() + y.getBaseRate() * x.getDisbelief()) / (r + x.getDisbelief())));
		
		} else if ((x.getBelief() == 1.0D) && (y.getBelief() == 1.0D)) {
			
			o.setBaseRate(((r * s * x.getBaseRate() + r * x.getBaseRate() + r * s * y.getBaseRate()
					+ s * y.getBaseRate() - r * s * x.getBaseRate() * y.getBaseRate()) / (r + s + r * s)));
		
		} else {
			o.setBaseRate(0.5D);
		}

		o.checkConsistency(true);

		return o;
	}

	private static SubjectiveOpinion division(SubjectiveOpinion x, SubjectiveOpinion y) {
		if ((x == null) || (y == null)) {
			throw new NullPointerException("Opinions cannot be null");
		}

		if (y.getBaseRate() == 0.0D) {
			throw new OpinionArithmeticException("Atomicity of divisor is zero");
		}

		x.setDependants();
		y.setDependants();

		if (y.getExpectation() - x.getExpectation() < -1.0E-010D) {
			throw new OpinionArithmeticException("Expectation of divisor cannot be less than of numerator");
		}
		try {
			double a = x.getBaseRate() / y.getBaseRate();
			SubjectiveOpinion o;
			if (x.getExpectation() == 0.0D) {
				o = new SubjectiveOpinion(0.0D, 1.0D, 0.0D, a);
			} else {
				if (a == 1.0D) {
					o = new SubjectiveOpinion(1.0D, 0.0D, 0.0D, a);
				} else {
					double e = x.getExpectation() / y.getExpectation();

					double d = OpinionBase.constrain((x.getDisbelief() - y.getDisbelief()) / (1.0D - y.getDisbelief()));
					double u = (1.0D - d - e) / (1.0D - a);
					double b = 1.0D - d - u;

					o = clippedOpinion(b, u, a);
				}
			}
			o.checkConsistency();
			o.recalculate = true;

			o.lastOp = OpinionOperator.UnAnd;

			return o;
		} catch (ArithmeticException ae) {
			throw new OpinionArithmeticException(ae.getMessage());
		}
	}

	private static SubjectiveOpinion coDivision(SubjectiveOpinion x, SubjectiveOpinion y, double r) {
		if ((x == null) || (y == null)) {
			throw new NullPointerException("Opinions cannot be null");
		}
		if ((r < 0.0D) || (r > 1.0D)) {
			throw new IllegalArgumentException("Limiting value, r, must be: 0<= r <=1");
		}

		x.setDependants();
		y.setDependants();
		try {
			double oBelief = (x.getBelief() - y.getBelief()) / (1.0D - y.getBelief());
			double oAtomicity = (x.getBaseRate() - y.getBaseRate()) / (1.0D - y.getBaseRate());

			double oUncertainty, oDisbelief;
			if (x.getBaseRate() > y.getBaseRate()) {
				
				oUncertainty = ((1.0D - x.getBelief()) / (1.0D - y.getBelief())
						- (x.getDisbelief() + (1.0D - x.getBaseRate()) * x.getUncertainty())
								/ (y.getDisbelief() + (1.0D - y.getBaseRate()) * y.getUncertainty()))
						* (1.0D - y.getBaseRate()) / (x.getBaseRate() - y.getBaseRate());
				
				oDisbelief = ((1.0D - y.getBaseRate())
						* (x.getDisbelief() + (1.0D - x.getBaseRate()) * x.getUncertainty())
						/ (y.getDisbelief() + (1.0D - y.getBaseRate()) * y.getUncertainty())
						- (1.0D - x.getBaseRate()) * (1.0D - x.getBelief()) / (1.0D - y.getBelief()))
						/ (x.getBaseRate() - y.getBaseRate());
			} else {
				oDisbelief = r * (1.0D - x.getBelief()) / (1.0D - y.getBelief());
				oUncertainty = (1.0D - r) * (1.0D - x.getBelief()) / (1.0D - y.getBelief());
			}

			SubjectiveOpinion o = new SubjectiveOpinion(oBelief, oDisbelief, oUncertainty, oAtomicity);
			o.checkConsistency();
			o.recalculate = true;

			o.lastOp = OpinionOperator.UnOr;

			return o;
		} catch (ArithmeticException e) {
			return null;
		}
	}

	public final SubjectiveOpinion unAnd(Opinion opinion) {
		if (opinion == null) {
			throw new NullPointerException("Opinions cannot be null");
		}
		
		return division(new SubjectiveOpinion(this), new SubjectiveOpinion(opinion));
	}

	public final SubjectiveOpinion unOr(Opinion opinion) {
		if (opinion == null) {
			throw new NullPointerException("Opinions cannot be null");
		}
		
		return coDivision(new SubjectiveOpinion(this), new SubjectiveOpinion(opinion), 0.0D);
	}

	private static SubjectiveOpinion deduction(SubjectiveOpinion x, SubjectiveOpinion yTx, SubjectiveOpinion yFx) {
		if ((x == null) || (yTx == null) || (yFx == null)) {
			throw new NullPointerException("Opinion and sub-conditionals cannot be null");
		}

		if (Math.abs(yTx.getBaseRate() - yFx.getBaseRate()) > 1.0E-010D) {
			throw new OpinionArithmeticException("The atomicities of both sub-conditionals must be equal");
		}
		x.setDependants();

		final double IBelief, IDisbelief, IUncertainty, IBaseRate;
		IBaseRate = yTx.getBaseRate();
		
		IBelief = x.getBelief() * yTx.getBelief() + x.getDisbelief() * yFx.getBelief()
				+ x.getUncertainty() * (yTx.getBelief() * x.getBaseRate() + yFx.getBelief() * (1.0D - x.getBaseRate()));
		
		IDisbelief = x.getBelief() * yTx.getDisbelief() + x.getDisbelief() * yFx.getDisbelief() + x.getUncertainty()
				* (yTx.getDisbelief() * x.getBaseRate() + yFx.getDisbelief() * (1.0D - x.getBaseRate()));
		
		IUncertainty = x.getBelief() * yTx.getUncertainty() + x.getDisbelief() * yFx.getUncertainty()
				+ x.getUncertainty()
						* (yTx.getUncertainty() * x.getBaseRate() + yFx.getUncertainty() * (1.0D - x.getBaseRate()));

		final SubjectiveOpinion I = new SubjectiveOpinion(IBelief, IDisbelief, IUncertainty, IBaseRate);

		I.setDependants(true);
		SubjectiveOpinion y;
		
		if (((yTx.getBelief() >= yFx.getBelief()) && (yTx.getDisbelief() >= yFx.getDisbelief()))
				|| ((yTx.getBelief() <= yFx.getBelief()) && (yTx.getDisbelief() <= yFx.getDisbelief()))) {
			y = I;
		} else {
			double expec = yTx.getBelief() * x.getBaseRate() + yFx.getBelief() * (1.0D - x.getBaseRate()) + yTx
					.getBaseRate()
					* (yTx.getUncertainty() * x.getBaseRate() + yFx.getUncertainty() * (1.0D - x.getBaseRate()));

			boolean case_II = (yTx.getBelief() > yFx.getBelief()) && (yTx.getDisbelief() < yFx.getDisbelief());

			boolean case_1 = x.getExpectation() <= x.getBaseRate();
			double k;
			if (case_II) {
				boolean case_A = expec <= yFx.getBelief()
						+ yTx.getBaseRate() * (1.0D - yFx.getBelief() - yTx.getDisbelief());
				if (case_A) {
					if (case_1) {
						double divisor;
						if ((divisor = x.getExpectation() * yTx.getBaseRate()) > 0.0D)
							k = x.getBaseRate() * x.getUncertainty() * (I.getBelief() - yFx.getBelief()) / divisor;
						else
							k = I.getBelief() - yFx.getBelief();
					} else {
						double divisor;
						if ((divisor = (x.getDisbelief() + (1.0D - x.getBaseRate()) * x.getUncertainty())
								* yTx.getBaseRate() * (yFx.getDisbelief() - yTx.getDisbelief())) > 0.0D)
							
							k = x.getBaseRate() * x.getUncertainty() * (I.getDisbelief() - yTx.getDisbelief())
									* (yTx.getBelief() - yFx.getBelief()) / divisor;
						else
							k = (I.getDisbelief() - yTx.getDisbelief()) * (yTx.getBelief() - yFx.getBelief());
					}
				} else {
					if (case_1) {
						double divisor;
						if ((divisor = x.getExpectation() * (1.0D - yTx.getBaseRate())
								* (yTx.getBelief() - yFx.getBelief())) > 0.0D)
							
							k = (1.0D - x.getBaseRate()) * x.getUncertainty() * (I.getBelief() - yFx.getBelief())
									* (yFx.getDisbelief() - yTx.getDisbelief()) / divisor;
						else
							k = (I.getBelief() - yFx.getBelief()) * (yFx.getDisbelief() - yTx.getDisbelief());
					} else {
						double divisor;
						if ((divisor = (x.getDisbelief() + (1.0D - x.getBaseRate()) * x.getUncertainty())
								* (1.0D - yTx.getBaseRate())) > 0.0D)
							
							k = (1.0D - x.getBaseRate()) * x.getUncertainty() * (I.getDisbelief() - yTx.getDisbelief())
									/ divisor;
						else {
							k = I.getDisbelief() - yTx.getDisbelief();
						}
					}
				}
			} else {
				boolean case_A = expec <= yTx.getBelief()
						+ yTx.getBaseRate() * (1.0D - yTx.getBelief() - yFx.getDisbelief());
				if (case_A) {
					if (case_1) {
						double divisor;
						if ((divisor = x.getExpectation() * yTx.getBaseRate()
								* (yTx.getDisbelief() - yFx.getDisbelief())) > 0.0D)
							
							k = (1.0D - x.getBaseRate()) * x.getUncertainty() * (I.getDisbelief() - yFx.getDisbelief())
									* (yFx.getBelief() - yTx.getBelief()) / divisor;
						else
							k = (I.getDisbelief() - yFx.getDisbelief()) * (yFx.getBelief() - yTx.getBelief());
					} else {
						double divisor;
						if ((divisor = (x.getDisbelief() + (1.0D - x.getBaseRate()) * x.getUncertainty())
								* yTx.getBaseRate()) > 0.0D)
							
							k = (1.0D - x.getBaseRate()) * x.getUncertainty() * (I.getBelief() - yTx.getBelief())
									/ divisor;
						else
							k = I.getBelief() - yTx.getBelief();
					}
				} else {
					if (case_1) {
						double divisor;
						if ((divisor = x.getExpectation() * (1.0D - yTx.getBaseRate())) > 0.0D)
							
							k = x.getBaseRate() * x.getUncertainty() * (I.getDisbelief() - yFx.getDisbelief())
									/ divisor;
						else
							k = I.getDisbelief() - yFx.getDisbelief();
					} else {
						double divisor;
						if ((divisor = (x.getDisbelief() + (1.0D - x.getBaseRate()) * x.getUncertainty())
								* (1.0D - yTx.getBaseRate()) * (yFx.getBelief() - yTx.getBelief())) > 0.0D)
							
							k = x.getBaseRate() * x.getUncertainty() * (I.getBelief() - yTx.getBelief())
									* (yTx.getDisbelief() - yFx.getDisbelief()) / divisor;
						else {
							k = (I.getBelief() - yTx.getBelief()) * (yTx.getDisbelief() - yFx.getDisbelief());
						}
					}
				}
			}
			double yAtomicity, yBelief, yDisbelief, yUncertainty;
			yAtomicity = yTx.getBaseRate();

			yBelief = OpinionBase.adjust(I.getBelief() - k * yAtomicity);
			yDisbelief = OpinionBase.adjust(I.getDisbelief() - k * (1.0D - yAtomicity));
			yUncertainty = OpinionBase.adjust(I.getUncertainty() + k);

			y = new SubjectiveOpinion(yAtomicity, yBelief, yDisbelief, yUncertainty);
			y.checkConsistency(true);
		}

		y.lastOp = OpinionOperator.Deduce;

		return y;
	}

	public final SubjectiveOpinion deduce(Conditionals conditionals) throws OpinionArithmeticException {
		if (conditionals == null) {
			throw new NullPointerException("The conditionals must not be null");
		}
		
		return deduction(new SubjectiveOpinion(this), new SubjectiveOpinion(conditionals.getPositive()),
				new SubjectiveOpinion(conditionals.getNegative()));
	}

	public final SubjectiveOpinion deduce(Opinion yTx, Opinion yFx) throws OpinionArithmeticException {
		if ((yTx == null) || (yFx == null)) {
			throw new NullPointerException("The conditionals must not be null");
		}
		
		return deduction(new SubjectiveOpinion(this), new SubjectiveOpinion(yTx), new SubjectiveOpinion(yFx));
	}

	private static SubjectiveOpinion abduction(SubjectiveOpinion y, SubjectiveOpinion yTx, SubjectiveOpinion yFx,
			double baseRateX) throws OpinionArithmeticException {
		if ((baseRateX < 0.0D) || (baseRateX > 1.0D)) {
			throw new IllegalArgumentException("Base Rate x, must be: 0 <= x <= 1");
		}
		
		if ((y == null) || (yTx == null) || (yFx == null)) {
			throw new NullPointerException("Opinions cannot be null");
		}
		
		SubjectiveOpinion o;

		if (y.getBaseRate() == 0.0D) {
			o = newVacuousOpinion(baseRateX);
		} else {
			Conditionals conditionals = reverseConditionals(yTx, yFx, baseRateX);
			o = deduction(y, conditionals.getPositive().toSubjectiveOpinion(),
					conditionals.getNegative().toSubjectiveOpinion());
		}

		o.lastOp = OpinionOperator.Abduce;

		return o;
	}

	public final SubjectiveOpinion abduce(Conditionals conditionals, double baseRateX)
			throws OpinionArithmeticException {
		if (conditionals == null) {
			throw new NullPointerException("Conditionals cannot be null");
		}
		
		return abduction(new SubjectiveOpinion(this), new SubjectiveOpinion(conditionals.getPositive()),
				new SubjectiveOpinion(conditionals.getNegative()), baseRateX);
	}

	public final SubjectiveOpinion abduce(Opinion xTy, Opinion xFy, double baseRateX)
			throws OpinionArithmeticException {
		if ((xTy == null) || (xFy == null)) {
			throw new NullPointerException("Opinions cannot be null");
		}
		
		return abduction(new SubjectiveOpinion(this), new SubjectiveOpinion(xTy), new SubjectiveOpinion(xFy),
				baseRateX);
	}

	public static Conditionals reverseConditionals(Conditionals conditionals, double baseRateX)
			throws OpinionArithmeticException {
		if (conditionals == null) {
			throw new NullPointerException("Opinions cannot be null");
		}
		
		return reverseConditionals(conditionals.getPositive().toSubjectiveOpinion(),
				conditionals.getNegative().toSubjectiveOpinion(), baseRateX);
	}

	public static Conditionals reverseConditionals(SubjectiveOpinion yTx, SubjectiveOpinion yFx, double baseRateX)
			throws OpinionArithmeticException {
		if ((baseRateX < 0.0D) || (baseRateX > 1.0D)) {
			throw new IllegalArgumentException("Base Rate x, must be: 0 <= x <= 1");
		}
		
		if ((yTx == null) || (yFx == null)) {
			throw new NullPointerException("Opinions cannot be null");
		}
		SubjectiveOpinion x_br = newVacuousOpinion(baseRateX);

		double atom_y = yTx.getBaseRate();
		SubjectiveOpinion xFy;
		SubjectiveOpinion xTy;
		
		if (baseRateX == 0.0D) {
			xTy = newDogmaticOpinion(0.0D, 0.0D);
			xFy = newDogmaticOpinion(0.0D, 0.0D);
		} else {
			if (baseRateX == 1.0D) {
				xTy = newDogmaticOpinion(1.0D, 1.0D);
				xFy = newDogmaticOpinion(1.0D, 1.0D);
			} else {
				if ((atom_y == 0.0D) || (atom_y == 1.0D)) {
					xTy = new SubjectiveOpinion(0.0D, 0.0D, 1.0D, baseRateX);
					xFy = new SubjectiveOpinion(0.0D, 0.0D, 1.0D, baseRateX);
				} else {
					SubjectiveOpinion not_yTx = complement(yTx);
					SubjectiveOpinion y_br = deduction(x_br, yTx, yFx);
					SubjectiveOpinion not_y_br = complement(y_br);
					SubjectiveOpinion y_and_x = multiply(x_br, yTx);
					SubjectiveOpinion not_y_and_x = multiply(x_br, not_yTx);

					xTy = division(y_and_x, y_br);
					xFy = division(not_y_and_x, not_y_br);
				}
			}
		}
		return new Conditionals(xTy, xFy);
	}

	////////////////////
	/// SL Operators ///
	////////////////////

	/**
	 * Returns the opinion with the lowest projected probability, that is, the
	 * lowest probability of being true.
	 * 
	 * @param opinions a collection of opinions from different sources
	 * @return a new SubjectiveOpinion thta represents the fused evidence
	 */
	public static SubjectiveOpinion minimumCollectionFuse(Collection<? extends SubjectiveOpinion> opinions) {
		if (opinions.contains(null) || opinions == null) {
			throw new NullPointerException("Opinions cannot be null");
		}

		if (opinions.size() < 2) {
			throw new IllegalArgumentException("At least two opinions must be provided");
		}

		SubjectiveOpinion min = null;
		for (SubjectiveOpinion so : opinions) {
			if (min == null) {
				min = so;
			}
			min = min.minimum(so);
		}

		return min;
	}

	/**
	 * This method implements MAJORITY fusion. This returns a new dogmatic opinion
	 * that specifies the decision of the majority. If the majority is tied, a
	 * vacuous opinion is returned. It is assumed that the base rates of all
	 * opinions are equal. For this operation, opinions that are undecided
	 * (expectation equals base rate) are ignored.
	 *
	 * @param opinions a collection of opinions from different sources.
	 * @return a new SubjectiveOpinion that represents the fused evidence.
	 * @throws OpinionArithmeticException
	 */
	public static SubjectiveOpinion majorityCollectionFuse(Collection<SubjectiveOpinion> opinions)
			throws OpinionArithmeticException {
		if (opinions.contains(null) || opinions.size() < 2)
			throw new NullPointerException("Cannot fuse null opinions, or only one opinion was passed");
		
		int pos = 0, neg = 0;
		for (SubjectiveOpinion so : opinions) {
			if (so.getExpectation() < so.getBaseRate())
				neg++;
			else if (so.getExpectation() > so.getBaseRate())
				pos++;
		}
		
		if (pos > neg)
			return new SubjectiveOpinion(1, 0, 0, 0.5);
		else if (pos < neg)
			return new SubjectiveOpinion(0, 1, 0, 0.5);
		else
			return new SubjectiveOpinion(0, 0, 1, 0.5);
	}
	
    /**
     * An exponential weighted average update method.
     *
     * This basically computes a new opinion o, such that the projection is:
     * P(o) = (1-alpha) P(avg) + alpha P(new)
     *
     * Where avg is the previous average, new is newOpinion, the next element in the time series, and alpha is the weight of the new data.
     * Alpha should be from [0,1] and the opinions should be non-null and valid.
     *
     * The atomicity of the original average is carried over to the output;
     * the opinion o is generated from the projection given above and then uncertainty maximized.
     *
     * Note; for alpha=1, o will be an uncertainty-maximized copy of newOpinion, and for alpha=0, o will be an uncertainty-maximized copy of average.
     *
     * @param average the previous average
     * @param newOpinion the new opinion generated by a detector
     * @param alpha the weight given to the new opinion
     * @return
     */
    public static SubjectiveOpinion exponentialWeightedAveraging(SubjectiveOpinion average, SubjectiveOpinion newOpinion, double alpha){
        if(average == null || newOpinion == null || alpha < 0 || alpha > 1 || ! average.isConsistent() || ! newOpinion.isConsistent())
            throw new IllegalArgumentException(); //TODO proper error messages

        double beliefProjection =  0;
        beliefProjection = (1-alpha) * average.getExpectation() + alpha * newOpinion.getExpectation();

        return fromProjection(beliefProjection, average.getBaseRate());
    }

	/**
	 * @deprecated
	 * @param x
	 * @param y
	 * @return
	 * @throws OpinionArithmeticException
	 */
	private static SubjectiveOpinion cumulativeFusion(SubjectiveOpinion x, SubjectiveOpinion y)
			throws OpinionArithmeticException {
		if ((x == null) || (y == null)) {
			throw new NullPointerException();
		}

		double totalWeight = x.getRelativeWeight() + y.getRelativeWeight();
		double weightX = x.getRelativeWeight(), weightY = y.getRelativeWeight();
		double k = x.getUncertainty() + y.getUncertainty() - x.getUncertainty() * y.getUncertainty();
		double l = x.getUncertainty() + y.getUncertainty() - 2.0D * x.getUncertainty() * y.getUncertainty();

		double resultBelief, resultDisbelief, resultUncertainty, resultAtomicity;

		if (k != 0.0D) {
			if (l != 0.0D) {
				resultBelief = (x.getBelief() * y.getUncertainty() + y.getBelief() * x.getUncertainty()) / k;
				
				resultDisbelief = (x.getDisbelief() * y.getUncertainty() + y.getDisbelief() * x.getUncertainty()) / k;
				
				resultUncertainty = x.getUncertainty() * y.getUncertainty() / k;
				
				resultAtomicity = (y.getBaseRate() * x.getUncertainty() + x.getBaseRate() * y.getUncertainty()
						- (x.getBaseRate() + y.getBaseRate()) * x.getUncertainty() * y.getUncertainty()) / l;
			
			} else if (Math.abs(x.getBaseRate() - y.getBaseRate()) <= 1.0E-010D) {
				
				resultBelief = 0.0D;
				
				resultDisbelief = 0.0D;
				
				resultUncertainty = 1.0D;
				
				resultAtomicity = x.getBaseRate();
			} else {
				throw new OpinionArithmeticException("Relative atomicities are not equal");
			}

		} else {
			resultBelief = (x.getBelief() * weightX + y.getBelief() * weightY) / totalWeight;
			
			resultDisbelief = (x.getDisbelief() * weightX + y.getDisbelief() * weightY) / totalWeight;
			
			resultUncertainty = 0.0D;
			
			resultAtomicity = (x.getBaseRate() * weightX + y.getBaseRate() * weightY) / totalWeight;
		}

		SubjectiveOpinion o = new SubjectiveOpinion(resultBelief, resultDisbelief, resultUncertainty, resultAtomicity);

		o.checkConsistency(true);

		o.lastOp = OpinionOperator.Fuse;
		o.setRelativeWeight(totalWeight); // relative weight represents how many opinions were fused in

		return o;
	}

	public final SubjectiveOpinion cumulativeFuse(Opinion opinion) throws OpinionArithmeticException {
		if (opinion == null) {
			throw new NullPointerException("Opinion must not be null");
		}
		
		SubjectiveOpinion copy1 = new SubjectiveOpinion(this);
		SubjectiveOpinion copy2 = new SubjectiveOpinion(opinion);
		
		return cumulativeFusion(copy1, copy2);
	}

	/**
	 * @deprecated
	 * @param opinions
	 * @return
	 * @throws OpinionArithmeticException
	 */
	public static SubjectiveOpinion cumulativeFuse(Collection<? extends Opinion> opinions)
			throws OpinionArithmeticException {
		if (opinions == null) {
			throw new NullPointerException("Opinions cannot be null");
		}
		
		if (opinions.isEmpty()) {
			throw new OpinionArithmeticException("Opinions must not be empty");
		}
		
		SubjectiveOpinion x = null;

		for (Opinion opinion : opinions) {
			if (opinion != null)
				x = x == null ? new SubjectiveOpinion(opinion) : x.cumulativeFuse(opinion);
		}
		
		return x;
	}

	/**
	 * This method implements cumulative belief fusion (CBF) for multiple sources,
	 * as discussed in the corrected version of
	 * <a href="https://folk.uio.no/josang/papers/JWZ2017-FUSION.pdf">a FUSION 2017
	 * paper by Jøsang et al.</a>
	 *
	 * As discussed in the book, cumulative fusion is useful in scenarios where
	 * opinions from multiple sources is combined, where each source is relying on
	 * independent (in the statistical sense) evidence. For more details, refer to
	 * Chapter 12 of the Subjective Logic book by Jøsang, specifically Section 12.3,
	 * which defines cumulative fusion.
	 *
	 * @param opinions a collection of opinions from different sources.
	 * @return a new SubjectiveOpinion that represents the fused evidence based on
	 *         evidence accumulation.
	 * @throws OpinionArithmeticException
	 */
	public static SubjectiveOpinion cumulativeCollectionFuse(Collection<SubjectiveOpinion> opinions)
			throws OpinionArithmeticException {
		// handle edge cases
		if (opinions == null) {
			throw new NullPointerException("Opinions cannot be null");
		}
		
		if (opinions.isEmpty()) {
			throw new OpinionArithmeticException("Opinions must not be empty");
		}
		
		if (opinions.size() == 1) {
			return new SubjectiveOpinion(opinions.iterator().next());
		}

		// fusion as defined by Jøsang
		double resultBelief, resultDisbelief, resultUncertainty, resultRelativeWeight = 0, resultAtomicity = -1;

		Collection<SubjectiveOpinion> dogmatic = new ArrayList<>(opinions.size());
		Iterator<SubjectiveOpinion> it = opinions.iterator();
		boolean first = true;
		while (it.hasNext()) {
			SubjectiveOpinion o = it.next();
			if (first) {
				resultAtomicity = o.getBaseRate();
				first = false;
			}
			// dogmatic iff uncertainty is zero.
			if (o.getUncertainty() == 0)
				dogmatic.add(o);
		}

		if (dogmatic.isEmpty()) {
			// there are no dogmatic opinions -- case I/Eq16 of 10.23919/ICIF.2017.8009820
			double productOfUncertainties = opinions.stream().mapToDouble(o -> o.getUncertainty()).reduce(1.0D,
					(acc, u) -> acc * u);

			double numerator = 0.0D;
			double beliefAccumulator = 0.0D;
			double disbeliefAccumulator = 0.0D;

			// this computes the top and bottom sums in Eq16, but ignores the - (N-1) *
			// productOfUncertainties in the numerator (see below)
			for (SubjectiveOpinion o : opinions) {
				// productWithoutO = product of uncertainties without o's uncertainty
				// this can be rewritten:
				// prod {C_j != C } u^{C_j} = (u^C)^-1 * prod{C_j} u^{C_j} = 1/(u^C) * prod{C_j}
				// u^{C_j}
				// so instead of n-1 multiplications, we only need a division
				double productWithoutO = productOfUncertainties / o.getUncertainty();

				beliefAccumulator = beliefAccumulator + productWithoutO * o.getBelief();
				disbeliefAccumulator = disbeliefAccumulator + productWithoutO * o.getDisbelief();
				numerator = numerator + productWithoutO;
			}

			// this completes the numerator:
			numerator = numerator - (opinions.size() - 1) * productOfUncertainties;

			resultBelief = beliefAccumulator / numerator;
			resultDisbelief = disbeliefAccumulator / numerator;
			resultUncertainty = productOfUncertainties / numerator;

			resultRelativeWeight = 0;
		} else {
			// at least 1 dogmatic opinion
			// note: this computation assumes that the relative weight represents how many
			// opinions have been fused into that opinion.
			// for a normal multi-source fusion operation, this should be 1, in which case
			// the gamma's in Eq17 are 1/N as noted in the text (i.e., all opinions count
			// equally)
			// however, this formulation also allows partial fusion beforehand, by
			// "remembering" the amount of dogmatic (!) opinions in o.relativeWeight.

			double totalWeight = dogmatic.stream().mapToDouble(o -> o.getRelativeWeight()).sum();

			resultBelief = dogmatic.stream().mapToDouble(o -> o.getRelativeWeight() / totalWeight * (o).getBelief())
					.sum();

			resultDisbelief = dogmatic.stream()
					.mapToDouble(o -> o.getRelativeWeight() / totalWeight * (o).getDisbelief()).sum();

			resultUncertainty = 0.0D;

			resultRelativeWeight = totalWeight;
		}

		SubjectiveOpinion result = new SubjectiveOpinion(resultBelief, resultDisbelief, resultUncertainty,
				resultAtomicity);
		result.setRelativeWeight(resultRelativeWeight);
		result.lastOp = OpinionOperator.Fuse;
		
		return result;
	}

	protected static SubjectiveOpinion smoothAverage(Collection<? extends Opinion> opinions) {
		if (opinions == null) {
			throw new NullPointerException("Opinions cannot be null");
		}
		
		if (opinions.isEmpty()) {
			throw new IllegalArgumentException("Opinions must not be empty");
		}

		int count = 0;

		double b = 0.0D;
		double a = 0.0D;
		double e = 0.0D;

		for (Opinion opinion : opinions) {
			if (opinion != null) {
				SubjectiveOpinion x = new SubjectiveOpinion(opinion);

				count++;
				b += x.getBelief();
				a += x.getBaseRate();
				e += x.getBelief() + x.getBaseRate() * x.getUncertainty();
			}
		}
		if (count == 0) {
			throw new IllegalArgumentException("Opinions must not be empty");
		}
		double oBelief = b / count;
		double oAtomicity = a / count;
		double oUncertainty = (e / count - oBelief) / oAtomicity;
		double oDisbelief = 1.0D - oBelief - oUncertainty;

		SubjectiveOpinion o = new SubjectiveOpinion(oBelief, oDisbelief, oUncertainty, oAtomicity);

		o.adjust();
		o.checkConsistency(true);

		return o;
	}

	public final SubjectiveOpinion average(Opinion opinion) {
		Collection<Opinion> opinions = new ArrayList<>();

		opinions.add(new SubjectiveOpinion(this));
		opinions.add(new SubjectiveOpinion(opinion));

		return smoothAverage(opinions).toSubjectiveOpinion();
	}

	public static SubjectiveOpinion average(Collection<? extends Opinion> opinions) {
		return smoothAverage(opinions);
	}

	/**
	 * Weighted Belief Fusion (See Josang SL book pp. 231)
	 * 
	 * @details "WBF is commutative, idempotent and has the vacuous opinion as
	 *          neutral element. Semi-associativity requires that three or more
	 *          argument must first be combined together in the same operation."
	 *
	 *          "WBF produces averaging beliefs weighted by the opinion confidences.
	 *          WBF is suitable for fusing (expert) agent opinions in situations
	 *          where the source agent’s confidence should determine the opinion
	 *          weight in the fusion process."
	 * @param x Opinion of source A
	 * @param y Opinion of source B
	 * @return WBF fused opinion
	 * @throws OpinionArithmeticException
	 * @deprecated
	 */
	private static SubjectiveOpinion wbFusion(SubjectiveOpinion x, SubjectiveOpinion y)
			throws OpinionArithmeticException {
		if ((x == null) || (y == null)) {
			throw new NullPointerException("Opinions cannot be null");
		}

		double b = 0.0D;
		double u = 0.0D;
		double a = 0.5D;

		// case 1: (u_A != 0 || u_B != 0) && (u_A != 1 || u_B != 1)
		if ((x.getUncertainty() != 0 || y.getUncertainty() != 0)
				&& (x.getUncertainty() != 1 || y.getUncertainty() != 1)) {
			
			b = (x.getBelief() * (1 - x.getUncertainty()) * y.getUncertainty()
					+ y.getBelief() * (1 - y.getUncertainty()) * x.getUncertainty())
					/ (x.getUncertainty() + y.getUncertainty() - 2 * x.getUncertainty() * y.getUncertainty());
			
			u = ((2 - x.getUncertainty() - y.getUncertainty()) * x.getUncertainty() * y.getUncertainty())
					/ (x.getUncertainty() + y.getUncertainty() - 2 * x.getUncertainty() * y.getUncertainty());
			
			a = (x.getBaseRate() * (1 - x.getUncertainty()) + y.getBaseRate() * (1 - y.getUncertainty()))
					/ (2 - x.getUncertainty() - y.getUncertainty());
		}

		// case 2: (u_A == 0 && u_B == 0)
		if (x.getUncertainty() == 0 && y.getUncertainty() == 0) {
			// In case of dogmatic arguments assume the limits in Eq.(12.24) to be γ_X^A =
			// γ_X^B = 0.5 (p. 232)
			double gammaA = 0.5;
			double gammaB = 0.5;

			b = gammaA * x.getBelief() + gammaB * y.getBelief();
			u = 0.0D;
			a = gammaA * x.getBaseRate() + gammaB * y.getBaseRate();
		}

		// case 3: (u_A == 1 && u_B == 1)
		if (x.getUncertainty() == 1 && y.getUncertainty() == 1) {
			b = 0.0D;
			u = 1.0D;
			a = (x.getBaseRate() + y.getBaseRate()) / 2;
		}

		SubjectiveOpinion res = new SubjectiveOpinion(b, u);
		a = OpinionBase.adjust(OpinionBase.constrain(a));
		res.setBaseRate(a);
		res.checkConsistency(true);
		
		return res;
	}

	/**
	 * @deprecated
	 * @param opinion
	 * @return
	 * @throws OpinionArithmeticException
	 */
	public final SubjectiveOpinion wbFuse(Opinion opinion) throws OpinionArithmeticException {
		if (opinion == null) {
			throw new NullPointerException("Opinion must not be null");
		}
		
		return wbFusion(new SubjectiveOpinion(this), new SubjectiveOpinion(opinion));
	}

	/**
	 * This method implements weighted belief fusion (WBF) for multiple sources, as
	 * discussed in a FUSION 2018 paper by van der Heijden et al. that is currently
	 * under review.
	 *
	 * As discussed in the book, WBF is intended to represent the
	 * confidence-weighted averaging of evidence. For more details, refer to Chapter
	 * 12 of the Subjective Logic book by Jøsang, specifically Section 12.5, which
	 * defines weighted belief fusion.
	 *
	 * @param opinions a collection of opinions from different sources.
	 * @return a new SubjectiveOpinion that represents the fused evidence based on
	 *         confidence-weighted averaging of evidence.
	 * @throws OpinionArithmeticException
	 */
	public static SubjectiveOpinion weightedCollectionFuse(Collection<SubjectiveOpinion> opinions)
			throws OpinionArithmeticException {
		if (opinions == null) {
			throw new NullPointerException("Opinions cannot be null");
		}
		
		if (opinions.isEmpty()) {
			throw new OpinionArithmeticException("Opinions must not be empty");
		}
		
		if (opinions.size() == 1) {
			return new SubjectiveOpinion(opinions.iterator().next());
		}

		double resultBelief, resultDisbelief, resultUncertainty, resultRelativeWeight = 0, resultAtomicity;

		Collection<SubjectiveOpinion> dogmatic = new ArrayList<>(opinions.size());
		Iterator<SubjectiveOpinion> it = opinions.iterator();
		while (it.hasNext()) {
			SubjectiveOpinion o = it.next();
			// dogmatic iff uncertainty is zero.
			if (o.getUncertainty() == 0)
				dogmatic.add(o);
		}

		if (dogmatic.isEmpty() && opinions.stream().anyMatch(o -> o.getCertainty() > 0)) {
			// Case 1: no dogmatic opinions, at least one non-vacuous opinion
			double productOfUncertainties = opinions.stream().mapToDouble(o -> o.getUncertainty()).reduce(1.0D,
					(acc, u) -> acc * u);
			double sumOfUncertainties = opinions.stream().mapToDouble(o -> o.getUncertainty()).sum();

			double numerator = 0.0D;
			double beliefAccumulator = 0.0D;
			double disbeliefAccumulator = 0.0D;
			double atomicityAccumulator = 0.0D;

			for (SubjectiveOpinion o : opinions) {
				// prod = product of uncertainties without o's uncertainty
				double prod = productOfUncertainties / o.getUncertainty();

				// recall certainty = 1 - uncertainty
				beliefAccumulator = beliefAccumulator + prod * o.getBelief() * o.getCertainty();
				disbeliefAccumulator = disbeliefAccumulator + prod * o.getDisbelief() * o.getCertainty();
				atomicityAccumulator = atomicityAccumulator + o.getBaseRate() * o.getCertainty();
				numerator = numerator + prod;
			}

			numerator = numerator - opinions.size() * productOfUncertainties;

			resultBelief = beliefAccumulator / numerator;
			resultDisbelief = disbeliefAccumulator / numerator;
			resultUncertainty = (opinions.size() - sumOfUncertainties) * productOfUncertainties / numerator;
			resultAtomicity = atomicityAccumulator / (opinions.size() - sumOfUncertainties);
		} else if (opinions.stream().allMatch(o -> o.getUncertainty() == 1)) {
			// Case 3 -- everything is vacuous
			resultBelief = 0;
			resultDisbelief = 0;
			resultUncertainty = 1;
			boolean first = true;

			// all confidences are zero, so the weight for each opinion is the same -> use a
			// plain average for the resultAtomicity
			resultAtomicity = 0;
			for (Opinion o : opinions) {
				if (first) {
					resultAtomicity = resultAtomicity + o.getBaseRate();
					first = false;
				}
			}
			resultAtomicity = resultAtomicity / ((double) opinions.size());

		} else {
			// Case 2 -- dogmatic opinions are involved
			double totalWeight = dogmatic.stream().mapToDouble(o -> o.getRelativeWeight()).sum();

			resultBelief = dogmatic.stream().mapToDouble(o -> o.getRelativeWeight() / totalWeight * o.getBelief())
					.sum();

			resultDisbelief = dogmatic.stream().mapToDouble(o -> o.getRelativeWeight() / totalWeight * o.getDisbelief())
					.sum();

			resultUncertainty = 0.0D;

			resultRelativeWeight = totalWeight;

			// note: the for loop below will always set resultAtomicity correctly.
			resultAtomicity = -1;
			boolean first = true;
			for (Opinion o : opinions) {
				if (first) {
					resultAtomicity = o.getBaseRate();
					first = false;
				}
			}
		}

		SubjectiveOpinion result = new SubjectiveOpinion(resultBelief, resultDisbelief, resultUncertainty,
				resultAtomicity);

		result.setRelativeWeight(resultRelativeWeight);
		result.lastOp = OpinionOperator.Fuse;
		
		return result;
	}

	// see Section 12.6 of the Subjective Logic book: 10.1007/978-3-319-42337-1_12

	/**
	 * @deprecated
	 * @param x
	 * @param y
	 * @return
	 * @throws OpinionArithmeticException
	 */
	private static SubjectiveOpinion ccFusion(SubjectiveOpinion x, SubjectiveOpinion y)
			throws OpinionArithmeticException {
		if (x == null || y == null)
			throw new NullPointerException("Cannot fuse null opinions");

		if (x.getBaseRate() != y.getBaseRate())
			throw new OpinionArithmeticException("Base rates for CC Fusion must be the same");

		// consensus
		double consensusBelief = Double.min(x.getBelief(), y.getBelief());
		double consensusDisbelief = Double.min(x.getDisbelief(), y.getDisbelief());
		double consensusMass = consensusBelief + consensusDisbelief;
		
		// note: residue belief must be at least zero, by the definition of belief mass
		// (see chapter 2), although this is not explicitly stated in equation 12.31
		double XResidueBelief = Math.max(x.getBelief() - consensusBelief, 0);
		double YResidueBelief = Math.max(y.getBelief() - consensusBelief, 0);
		double XResidueDisbelief = Math.max(x.getDisbelief() - consensusDisbelief, 0);
		double YResidueDisbelief = Math.max(y.getDisbelief() - consensusDisbelief, 0);

		// compromise
		double compromiseBelief = XResidueBelief * y.getUncertainty() + YResidueBelief * x.getUncertainty()
				+ 1 * 1 * XResidueBelief * YResidueBelief + // first sum; y=z=x for x=true
				0 * 0 * XResidueBelief * YResidueBelief + // second sum; y=z=x for x=true
				0; // third sum; x=true means that the intersection of y and z must be non-empty
		double compromiseDisbelief = XResidueDisbelief * y.getUncertainty() + YResidueDisbelief * x.getUncertainty()
				+ 1 * 1 * XResidueDisbelief * YResidueDisbelief + // first sum; y=z=x for x=false
				0 * 0 * XResidueDisbelief * YResidueDisbelief + // second sum; y=z=x for x=false
				0; // third sum; x=false means that the intersection of y and z must be non-empty

		// this variable contains the belief mass for the entire domain, which is in
		// this case
		// {true, false}. For subjective opinions, belief({true,false})=0 by definition,
		// however
		// the compromise process introduces some belief to the entire domain. This is
		// later used in
		// the normalization process to compute the fused uncertainty, because belief
		// over the entire
		// domain is basically the same thing as uncertainty.
		// residual belief mass over {T,F} is 0, so the computation using eq12.32 is a
		// lot easier,
		// since only the third sum is non-zero:
		double compromiseUncertainty = XResidueBelief * YResidueDisbelief + YResidueBelief * XResidueDisbelief;

		double preliminaryUncertainty = x.getUncertainty() * y.getUncertainty();
		double compromiseMass = compromiseBelief + compromiseDisbelief + compromiseUncertainty;

		// FIXME: compromise mass is 0 for equal arguments and two contradicting
		// dogmatic opinions
		double normalizationFactor = (1 - consensusMass - preliminaryUncertainty) / (compromiseMass);

		double fusedUncertainty = preliminaryUncertainty + normalizationFactor * compromiseUncertainty;

		// merger
		double fusedBelief = consensusBelief + normalizationFactor * compromiseBelief;
		double fusedDisbelief = consensusDisbelief + normalizationFactor * compromiseDisbelief;

		SubjectiveOpinion res = new SubjectiveOpinion(fusedBelief, fusedDisbelief, fusedUncertainty, x.getBaseRate());
		res.checkConsistency(true);
		
		return res;
	}

	/**
	 * @deprecated
	 * @param opinion
	 * @return
	 * @throws OpinionArithmeticException
	 */
	public final SubjectiveOpinion ccFuse(Opinion opinion) throws OpinionArithmeticException {
		if (opinion == null) {
			throw new NullPointerException("Opinion must not be null");
		}
		
		return ccFusion(new SubjectiveOpinion(this), new SubjectiveOpinion(opinion));
	}

	/**
	 * <b>WARNING:</b> This is currently *not* implemented properly, and may produce
	 * unexpected results! c.f. semi-associativity Josang 2016, p. 235:
	 * 
	 * "Semi-associativity requires that three or more arguments must first
	 *  be combined together in the consensus step, and then combined
	 *  together again in the compromise step before the merging step."
	 *          
	 * @param opinions Collection of opinions to be fused
	 * @return Fused opinion
	 * @throws OpinionArithmeticException
	 * @deprecated
	 */
	public static SubjectiveOpinion ccFuse(Collection<? extends Opinion> opinions) throws OpinionArithmeticException {
		if (opinions == null) {
			throw new NullPointerException("Opinions cannot be null");
		}
		
		if (opinions.isEmpty()) {
			throw new OpinionArithmeticException("Opinions must not be empty");
		}
		
		SubjectiveOpinion x = null;

		for (Opinion opinion : opinions) {
			if (opinion != null)
				x = x == null ? new SubjectiveOpinion(opinion) : x.ccFuse(opinion);
		}
		
		return x;
	}

	public enum Domain {
		NIL, TRUE, FALSE, DOMAIN;

		public Domain intersect(Domain d) {
			switch (this) {
			case NIL:
				return NIL;
			case TRUE:
				switch (d) {
				case NIL:
				case FALSE:
					return NIL;
				case TRUE:
				case DOMAIN:
					return TRUE;
				default:
					throw new RuntimeException("unidentified domain");
				}
			case FALSE:
				switch (d) {
				case NIL:
				case TRUE:
					return NIL;
				case FALSE:
				case DOMAIN:
					return FALSE;
				default:
					throw new RuntimeException("unidentified domain");
				}
			case DOMAIN:
				return d;
			default:
				throw new RuntimeException("unidentified domain");
			}
		}

		public Domain union(Domain d) {
			switch (this) {
			case DOMAIN:
				return DOMAIN;
			case TRUE:
				switch (d) {
				case TRUE:
				case NIL:
					return TRUE;
				case FALSE:
				case DOMAIN:
					return DOMAIN;
				default:
					throw new RuntimeException("unidentified domain");
				}
			case FALSE:
				switch (d) {
				case FALSE:
				case NIL:
					return FALSE;
				case TRUE:
				case DOMAIN:
					return DOMAIN;
				default:
					throw new RuntimeException("unidentified domain");
				}
			case NIL:
				return d;
			default:
				throw new RuntimeException("unidentified domain");
			}
		}
	}

	private static Set<List<Domain>> tabulateOptions(int size) {
		if (size == 1) {
			Set<List<Domain>> result = new HashSet<List<Domain>>();
			for (Domain item : Domain.values()) {
				List<Domain> l = new ArrayList<Domain>();
				l.add(item);
				result.add(l);
			}
			return result;
		}
		Set<List<Domain>> result = new HashSet<List<Domain>>();
		for (List<Domain> tuple : tabulateOptions(size - 1)) {
			for (Domain d : Domain.values()) {
				List<Domain> newList = new ArrayList<Domain>(tuple);
				newList.add(d);
				result.add(newList);
			}
		}
		return result;
	}

	/**
	 * This method implements consensus and compromise fusion (CCF) for multiple
	 * sources, as discussed in a FUSION 2018 paper by van der Heijden et al. that
	 * is currently under review.
	 *
	 * For more details, refer to Chapter 12 of the Subjective Logic book by Jøsang,
	 * specifically Section 12.6, which defines CC fusion for the case of two
	 * sources.
	 *
	 * @param opinions a collection of opinions from different sources.
	 * @return a new SubjectiveOpinion that represents the fused evidence.
	 * @throws OpinionArithmeticException
	 */
	public static SubjectiveOpinion ccCollectionFuse(Collection<SubjectiveOpinion> opinions)
			throws OpinionArithmeticException {
		if (opinions.contains(null) || opinions.size() < 2)
			throw new NullPointerException("Cannot fuse null opinions, or only one opinion was passed");

		double baseRate = -1;
		boolean first = true;
		for (SubjectiveOpinion so : opinions) {
			if (first) {
				baseRate = so.getBaseRate();
				first = false;
			} else if (baseRate != so.getBaseRate()) {
				throw new OpinionArithmeticException("Base rates for CC Fusion must be the same");
			}
		}

		// Step 1: consensus phase
		final double consensusBelief = opinions.stream().mapToDouble(o -> o.getBelief()).min().getAsDouble();
		final double consensusDisbelief = opinions.stream().mapToDouble(o -> o.getDisbelief()).min().getAsDouble();

		final double consensusMass = consensusBelief + consensusDisbelief;

		List<Double> residueBeliefs = new ArrayList<>(opinions.size());
		List<Double> residueDisbeliefs = new ArrayList<>(opinions.size());
		List<Double> uncertainties = new ArrayList<>(opinions.size());
		for (SubjectiveOpinion so : opinions) {
			// note: this max should not be necessary..
			residueBeliefs.add(Math.max(so.getBelief() - consensusBelief, 0));
			residueDisbeliefs.add(Math.max(so.getDisbelief() - consensusDisbelief, 0));
			uncertainties.add(so.getUncertainty());
		}

		// Step 2: Compromise phase

		double productOfUncertainties = opinions.stream().mapToDouble(o -> o.getUncertainty()).reduce(1.0D,
				(acc, u) -> acc * u);

		double compromiseBeliefAccumulator = 0;
		double compromiseDisbeliefAccumulator = 0;
		double compromiseXAccumulator = 0; // this is what will later become uncertainty

		// this computation consists of 4 sub-sums that will be added independently.
		for (int i = 0; i < opinions.size(); i++) {
			double bResI = residueBeliefs.get(i);
			double dResI = residueDisbeliefs.get(i);
			double uI = uncertainties.get(i);
			double uWithoutI = productOfUncertainties / uI;

			// sub-sum 1:
			compromiseBeliefAccumulator = compromiseBeliefAccumulator + bResI * uWithoutI;
			compromiseDisbeliefAccumulator = compromiseDisbeliefAccumulator + dResI * uWithoutI;
			// note: compromiseXAccumulator is unchanged, since b^{ResI}_X() of the entire
			// domain is 0
		}
		// sub-sums 2,3,4 are all related to different permutations of possible values
		for (List<Domain> permutation : tabulateOptions(opinions.size())) {
			Domain intersection = permutation.stream().reduce(Domain.DOMAIN, (acc, p) -> acc.intersect(p));
			Domain union = permutation.stream().reduce(Domain.NIL, (acc, p) -> acc.union(p));

			// sub-sum 2: intersection of elements in permutation is x
			if (intersection.equals(Domain.TRUE)) {
				// --> add to belief
				double prod = 1;
				if (permutation.contains(Domain.DOMAIN))
					prod = 0;
				else
					for (int j = 0; j < permutation.size(); j++)
						switch (permutation.get(j)) {
						case DOMAIN:
							prod = 0; // multiplication by 0
							break;
						case TRUE:
							prod = prod * residueBeliefs.get(j) * 1;
							break;
						case FALSE:
						case NIL:
						default:
							break;
						}
				compromiseBeliefAccumulator = compromiseBeliefAccumulator + prod;
			} else if (intersection.equals(Domain.FALSE)) {
				// --> add to disbelief
				double prod = 1;
				if (permutation.contains(Domain.DOMAIN))
					prod = 0;
				else
					for (int j = 0; j < permutation.size(); j++)
						switch (permutation.get(j)) {
						case DOMAIN:
							prod = 0; // multiplication by 0
							break;
						case FALSE:
							prod = prod * residueDisbeliefs.get(j) * 1;
							break;
						case NIL:
						case TRUE:
						default:
							break;
						}
				compromiseDisbeliefAccumulator = compromiseDisbeliefAccumulator + prod;
			}

			switch (union) {
			case DOMAIN:
				if (!intersection.equals(Domain.NIL)) {
					// sub-sum 3: union of elements in permutation is x, and intersection of
					// elements in permutation is not NIL

					// Note: this is always zero for binary domains, as explained by the following:
					// double prod = 1;
					// for (int j=0; j<permutation.size(); j++) {
					// switch (permutation.get(j)) {
					// case NIL:
					// case DOMAIN:
					// prod = 0; //because residue belief over NIL/DOMAIN is zero here
					// break;
					// case TRUE:
					// case FALSE:
					// prod = 0; //because 1-a(y|x) is zero here, since a(y|x)=1 when x=y, and this
					// must occur, since a(x|!x) occurring implies the intersection is NIL
					// break;
					// }
					// }

				} else {
					// sub-sum 4: union of elements in permutation is x, and intersection of
					// elements in permutation is NIL
					double prod = 1;
					for (int j = 0; j < permutation.size(); j++) {
						switch (permutation.get(j)) {
						case NIL:
						case DOMAIN:
							prod = 0; // because residue belief over NIL/DOMAIN is zero here
							break;
						case TRUE:
							prod = prod * residueBeliefs.get(j);
							break;
						case FALSE:
							prod = prod * residueDisbeliefs.get(j);
							break;
						}
					}
					compromiseXAccumulator = compromiseXAccumulator + prod;
				}
				break;
			case NIL:
				// union of NIL means we have nothing to add
				// sub-sum 3: union of elements in permutation is x, and intersection of
				// elements in permutation is not NIL
				// sub-sum 4: union of elements in permutation is x, and intersection of
				// elements in permutation is NIL
				break;
			case TRUE:
				// sub-sum 3: this is always zero for TRUE and FALSE, since 1-a(y_i|y_j)=0 in
				// binary domains, where the relative base rate is either 1 if the union is x

				// sub-sum 4: union of elements in permutation is x, and intersection of
				// elements in permutation is NIL
				if (intersection.equals(Domain.NIL)) {
					// union is true, intersection is nil --> compute the product
					double prod = 1;
					for (int j = 0; j < permutation.size(); j++) {
						switch (permutation.get(j)) { // other cases will not occur
						case TRUE:
							prod = prod * residueBeliefs.get(j);
							break;
						case FALSE:
							prod = prod * residueDisbeliefs.get(j);
							break;
						case NIL:
							prod = 0;
							break;
						default:
							throw new RuntimeException();
						}
					}
					compromiseBeliefAccumulator = compromiseBeliefAccumulator + prod;
				}
				break;
			case FALSE:
				// sub-sum 3: this is always zero for TRUE and FALSE, since 1-a(y_i|y_j)=0 in
				// binary domains, where the relative base rate is either 1 if the union is x
				// sub-sum 4: union of elements in permutation is x, and intersection of
				// elements in permutation is NIL
				if (intersection.equals(Domain.NIL)) {
					// union is true, intersection is nil --> compute the product
					double prod = 1;
					for (int j = 0; j < permutation.size(); j++) {
						switch (permutation.get(j)) { // other cases will not occur
						case TRUE:
							prod = prod * residueBeliefs.get(j);
							break;
						case FALSE:
							prod = prod * residueDisbeliefs.get(j);
							break;
						case NIL:
							prod = 0;
							break;
						default:
							throw new RuntimeException();
						}
					}
					compromiseDisbeliefAccumulator = compromiseDisbeliefAccumulator + prod;
				}
				break;
			default:
				break;

			}
		}

		double compromiseBelief = compromiseBeliefAccumulator;
		double compromiseDisbelief = compromiseDisbeliefAccumulator;
		double compromiseUncertainty = compromiseXAccumulator;

		double preliminaryUncertainty = productOfUncertainties;
		double compromiseMass = compromiseBelief + compromiseDisbelief + compromiseUncertainty;

		// Step 3: Normalization phase
		double normalizationFactor = (1 - consensusMass - preliminaryUncertainty) / (compromiseMass);

		double fusedUncertainty = preliminaryUncertainty + normalizationFactor * compromiseUncertainty;
		// compromiseUncertainty = 0; --> but this variable is never used again anyway.

		double fusedBelief = consensusBelief + normalizationFactor * compromiseBelief;
		double fusedDisbelief = consensusDisbelief + normalizationFactor * compromiseDisbelief;

		SubjectiveOpinion res = new SubjectiveOpinion(fusedBelief, fusedDisbelief, fusedUncertainty, baseRate);
		res.checkConsistency(true);
		
		return res;
	}
}
