/**
 * 
 */
package de.tum.i4.subjectivelogic;

import java.util.HashSet;
import java.util.Set;

/**
 * @author squijano
 *
 */
public enum OpinionOperator {
	
	Discount(2, "%1$s:%2$s"),
	
	Fuse(2, Integer.MAX_VALUE, "%1$s¤%2$s"),
	
	Add(2, Integer.MAX_VALUE, "%1$s+%2$s"),
	
	Subtract(2, "%1$s-%2$s"),
	
	Or(2, Integer.MAX_VALUE, "%1$s|%2$s"),
	
	And(2, Integer.MAX_VALUE, "%1$s&%2$s"),
	
	SimpleOr(2, "%1$s||%2$s"),
	
	SimpleAnd(2, "%1$s&&%2$s"),
	
	UnOr(2, "%1$s~|%2$s"),
	
	UnAnd(2, "%1$s~&%2$s"),
	
	Not(1, "¬%1$s"),
	
	Deduce(3, "Deduce(%1$s,%2$s,%3$s)"),
	
	Abduce(4, "Abduce(%1$s,%2$s,%3$s,%4$s)");
	
	private String format;
	private String tag;
	private boolean commutative = false;
	private boolean associative = false;
	private int minArgs = 0;
	private int maxArgs = 0;
	private Set<OpinionOperator> distributesOver = null;
	
	static {
		Or.commutative = true;
		Or.associative = true;
		
		And.commutative = true;
		And.associative = true;
		
		Add.commutative = true;
		Add.associative = true;
		
		Fuse.commutative = true;
		Fuse.associative = true;
		
		Discount.associative = true;
		
		Not.commutative = true;
		Not.associative = true;
	}
	
	OpinionOperator(int minArgs, int maxArgs, String fmt){
		int min = 0;
		int max = 0;
		
		if ((minArgs < 0) || (maxArgs < 0)) {
			throw new IllegalArgumentException("Arguments must be greater than or equal zero");
		}
		
		if (fmt == null) {
			throw new NullPointerException("String format must not be null");
		}
		
		min = Math.min(minArgs, maxArgs);
		max = Math.max(minArgs, maxArgs);
		
		this.minArgs = min;
		this.maxArgs = max;
		this.format = fmt;
	}
	
	OpinionOperator(int minArgs, String fmt){
		this(minArgs, minArgs, fmt);
	}
	
	public static OpinionOperator get(String tag) {
		return Enum.valueOf(OpinionOperator.class, tag);
	}
	
	public String format(Object[] args) {
		return String.format(this.format, args);
	}
	
	public int getMinArgs() {
		return this.minArgs;
	}
	
	public int getMaxArgs() {
		return this.maxArgs;
	}
	
	public boolean checkArgCount(int args) {
		return (args >= this.minArgs) && (args <= this.maxArgs);
	}
	
	public boolean isAssociative() {
		return this.associative;
	}
	
	public boolean isCommutative() {
		return this.commutative;
	}
	
	public void distributesOver(OpinionOperator operator) {
		if (operator == null) {
			throw new NullPointerException("Operator must not be null");
		}
		
		synchronized(this) {
			if (this.distributesOver == null) {
				this.distributesOver = new HashSet<>();
			}
			this.distributesOver.add(operator);
		}
	}
	
	public boolean isDistributiveOver(OpinionOperator operator) {
		boolean isDistributiveOver = false;
		
		synchronized(this) {
			if (this.distributesOver != null) {
				isDistributiveOver = this.distributesOver.contains(operator);
			}
		}
		
		return isDistributiveOver;
	}
	
	public String toString() {
		return this.tag;
	}
}
