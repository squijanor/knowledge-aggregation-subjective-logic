package de.uulm.vs.subjectivelogic.test;

import static org.junit.Assert.fail;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import de.tum.i4.subjectivelogic.SubjectiveOpinion;

public class TestSubjectiveOpinion {
    private final Logger l = LogManager.getLogger(getClass());

    @Before
    public void setUp() {
        l.info("Setting up opinions...");
    }

    @Test
    public void testUncertaintyMaximization(){
        l.info("Testing uncertainty maximization.");

        SubjectiveOpinion uncertain = new SubjectiveOpinion(0.5, 0.5, 0, 0.5);

        Assert.assertFalse(uncertain.isUncertaintyMaximized());
        SubjectiveOpinion uMax = uncertain.uncertainOpinion();
        Assert.assertTrue(uMax.isUncertaintyMaximized());
        Assert.assertEquals(uMax, new SubjectiveOpinion(0,0,1,0.5));

        SubjectiveOpinion believe = new SubjectiveOpinion(0.7, 0.3, 0, 0.5);
        Assert.assertFalse(believe.isUncertaintyMaximized());
        uMax = believe.uncertainOpinion();
        Assert.assertTrue(uMax.isUncertaintyMaximized());
        Assert.assertEquals(uMax, new SubjectiveOpinion(0.4,0,0.6,0.5));

    }

    @Test
    public void testFromProjection(){
        l.info("Testing Opinion construction from projection.");

        SubjectiveOpinion uncertain = new SubjectiveOpinion(0.5, 0.5, 0, 0.5);

        SubjectiveOpinion uMax = uncertain.uncertainOpinion();
        Assert.assertEquals(uMax, SubjectiveOpinion.fromProjection(0.5));

        SubjectiveOpinion believe = new SubjectiveOpinion(0.7, 0.3, 0, 0.5);
        uMax = believe.uncertainOpinion();

        Assert.assertEquals(uMax, SubjectiveOpinion.fromProjection(0.7));

        //test exceptional behavior
        try {
            SubjectiveOpinion fail = SubjectiveOpinion.fromProjection(-0.1);
            fail("Created opinion from invalid projection: " + fail);
        }catch (IllegalArgumentException e){
            l.info("Correct exception with value -0.1");
        }
        try {
            SubjectiveOpinion fail = SubjectiveOpinion.fromProjection(1.1);
            fail("Created opinion from invalid projection: " + fail);
        }catch (IllegalArgumentException e){
            l.info("Correct exception with value 1.1");
        }
        try {
            SubjectiveOpinion fail = SubjectiveOpinion.fromProjection(0.1, -0.1);
            fail("Created opinion from projection with invalid base rate: " + fail);
        }catch (IllegalArgumentException e){
            l.info("Correct exception with base rate -0.1");
        }
        try {
            SubjectiveOpinion fail = SubjectiveOpinion.fromProjection(0.1, 1.1);
            fail("Created opinion from projection with invalid base rate: " + fail);
        }catch (IllegalArgumentException e){
            l.info("Correct exception with base rate " + 1.1);
        }
    }

    @Test
    public void testEWA(){
        l.info("Testing exponential weighted average operation.");

        SubjectiveOpinion uncertain = new SubjectiveOpinion(0.5, 0.5, 0, 0.5);
        SubjectiveOpinion believe = new SubjectiveOpinion(0.7, 0.3, 0, 0.5);
        SubjectiveOpinion distrust = new SubjectiveOpinion(0.3, 0.7, 0, 0.5);

        SubjectiveOpinion result;

        result = SubjectiveOpinion.exponentialWeightedAveraging(uncertain, believe, 1);
        Assert.assertEquals(result.getExpectation(), believe.getExpectation(), SubjectiveOpinion.TOLERANCE);
        result = SubjectiveOpinion.exponentialWeightedAveraging(uncertain, believe, 0);
        Assert.assertEquals(result.getExpectation(), uncertain.getExpectation(), SubjectiveOpinion.TOLERANCE);

        result = SubjectiveOpinion.exponentialWeightedAveraging(uncertain, believe, 0.5);
        Assert.assertEquals(result.getExpectation(), 0.5*uncertain.getExpectation() + 0.5*believe.getExpectation(), SubjectiveOpinion.TOLERANCE);
        result = SubjectiveOpinion.exponentialWeightedAveraging(result, believe, 0.5);
        Assert.assertEquals(result.getExpectation(), 0.5*(0.5*uncertain.getExpectation() + 0.5*believe.getExpectation()) + 0.5*believe.getExpectation(), SubjectiveOpinion.TOLERANCE);

        result = SubjectiveOpinion.exponentialWeightedAveraging(uncertain, distrust, 0.5);
        Assert.assertEquals(result.getExpectation(), 0.5*uncertain.getExpectation() + 0.5*distrust.getExpectation(), SubjectiveOpinion.TOLERANCE);
        result = SubjectiveOpinion.exponentialWeightedAveraging(result, distrust, 0.5);
        Assert.assertEquals(result.getExpectation(), 0.5*(0.5*uncertain.getExpectation() + 0.5*distrust.getExpectation()) + 0.5*distrust.getExpectation(), SubjectiveOpinion.TOLERANCE);

        result = SubjectiveOpinion.exponentialWeightedAveraging(believe, distrust, 0.5);
        Assert.assertEquals(result.getExpectation(), 0.5*believe.getExpectation() + 0.5*distrust.getExpectation(), SubjectiveOpinion.TOLERANCE);
        result = SubjectiveOpinion.exponentialWeightedAveraging(result, distrust, 0.5);
        Assert.assertEquals(result.getExpectation(), 0.5*(0.5*believe.getExpectation() + 0.5*distrust.getExpectation()) + 0.5*distrust.getExpectation(), SubjectiveOpinion.TOLERANCE);
    }
}
