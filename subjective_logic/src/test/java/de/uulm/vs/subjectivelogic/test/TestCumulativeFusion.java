package de.uulm.vs.subjectivelogic.test;

import de.tum.i4.subjectivelogic.OpinionBase;
import de.tum.i4.subjectivelogic.SubjectiveOpinion;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.junit.Assert;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

public class TestCumulativeFusion extends TestFusionSetup {
    private final Logger l = LogManager.getLogger(getClass());

    @Test
    public void testCumulativeFusion_2_opinions() {
        l.info("Testing binary operator SubjectiveLogic.cumulativeFuse(o)");

        SubjectiveOpinion nonZero1 = new SubjectiveOpinion(soP);
        SubjectiveOpinion nonZero2 = new SubjectiveOpinion(soN);

        SubjectiveOpinion vacuous1 = new SubjectiveOpinion(soVacuous);
        SubjectiveOpinion vacuous2 = new SubjectiveOpinion(soVacuous);

        SubjectiveOpinion zero1 = new SubjectiveOpinion(soPD);
        SubjectiveOpinion zero2 = new SubjectiveOpinion(soND);

        //Case 1: u^A_X != 0 or u^B_X != 0

        // b = (b^A u^B + b^B u^A) / (u^A + u^B - u^au^B)
        // u = u^A u^B / (u^A + u^B - u^au^B)

        //Case 1A: u^A != 1 or u^B != 1
        // a = (a^Au^B + a^Bu^A - (a^A + a^B)u^Au^B)/(u^A + u^B - u^au^B)
        SubjectiveOpinion res = nonZero1.cumulativeFuse(nonZero2);

        Assert.assertTrue(res.isConsistent());
        double bA = nonZero1.getBelief();
        double dA = nonZero1.getDisbelief();
        double uA = nonZero1.getUncertainty();
        double aA = nonZero1.getBaseRate();

        double bB = nonZero2.getBelief();
        double dB = nonZero2.getDisbelief();
        double uB = nonZero2.getUncertainty();
        double aB = nonZero2.getBaseRate();

        double expectedBelief = (bA*uB + bB*uA)/(uA+uB-uA*uB);
        Assert.assertEquals(expectedBelief, res.getBelief(), OpinionBase.TOLERANCE);

        double expectedDisbelief = (dA*uB + dB*uA)/(uA+uB-uA*uB);
        Assert.assertEquals(expectedDisbelief, res.getDisbelief(), OpinionBase.TOLERANCE);

        double expectedUncertainty = uA*uB/(uA+uB-uA*uB);
        Assert.assertEquals(expectedUncertainty, res.getUncertainty(), OpinionBase.TOLERANCE);

        double expectedBaseRate = (aA*uB+aB*uA-(aA+aB)*uA*uB)/(uA+uB-2*uA*uB);
        Assert.assertEquals(expectedBaseRate, res.getBaseRate(), OpinionBase.TOLERANCE);

        //Case 1B: u^A = u^B = 1
        // a = (a^A + a^B)/2
        SubjectiveOpinion vac = vacuous1.cumulativeFuse(vacuous2);

        Assert.assertTrue(vac.isConsistent());
        bA = vacuous1.getBelief();
        dA = vacuous1.getDisbelief();
        uA = vacuous1.getUncertainty();
        aA = vacuous1.getBaseRate();

        dB = vacuous2.getDisbelief();
        bB = vacuous2.getBelief();
        uB = vacuous2.getUncertainty();
        aB = vacuous2.getBaseRate();

        expectedBelief = (bA*uB + bB*uA)/(uA+uB-uA*uB);
        Assert.assertEquals(expectedBelief, vac.getBelief(), OpinionBase.TOLERANCE);

        expectedDisbelief = (dA*uB + dB*uA)/(uA+uB-uA*uB);
        Assert.assertEquals(expectedDisbelief, vac.getDisbelief(), OpinionBase.TOLERANCE);

        expectedUncertainty = uA*uB/(uA+uB-uA*uB);
        Assert.assertEquals(expectedUncertainty, vac.getUncertainty(), OpinionBase.TOLERANCE);

        expectedBaseRate = (aA + aB)/2.0D;
        Assert.assertEquals(expectedBaseRate, vac.getBaseRate(), OpinionBase.TOLERANCE);

        //Case 2: u^A_X = u^B_X = 0
        SubjectiveOpinion dogmatic = zero1.cumulativeFuse(zero2);

        Assert.assertTrue(dogmatic.isConsistent());
        bA = zero1.getBelief();
        dA = zero1.getDisbelief();
        uA = zero1.getUncertainty();
        aA = zero1.getBaseRate();

        dB = zero2.getDisbelief();
        bB = zero2.getBelief();
        uB = zero2.getUncertainty();
        aB = zero2.getBaseRate();

        //note, book says: In case of dogmatic arguments it can be assumed that the limits in Eq.(12.15) are defined as γ X A = γ X B = 0.5.
        double gammaA = 0.5D;
        double gammaB = 0.5D;

        expectedBelief = gammaA*bA + gammaB*bB;
        Assert.assertEquals(expectedBelief, dogmatic.getBelief(), OpinionBase.TOLERANCE);

        expectedDisbelief = gammaA*dA + gammaB*dB;
        Assert.assertEquals(expectedDisbelief, dogmatic.getDisbelief(), OpinionBase.TOLERANCE);

        expectedUncertainty = 0.0D;
        Assert.assertEquals(expectedUncertainty, dogmatic.getUncertainty(), OpinionBase.TOLERANCE);

        expectedBaseRate = gammaA*aA + gammaB*aB;
        Assert.assertEquals(expectedBaseRate, dogmatic.getBaseRate(), OpinionBase.TOLERANCE);
    }

    @Test
    public void testCumulativeFusion_tri_associativity_nonDogmatic() {
        l.info("Testing associativity of SubjectiveLogic.cumulativeFuse(o) for non-dogmatic inputs (tri-source)");

        SubjectiveOpinion batchFuse = SubjectiveOpinion.cumulativeFuse(soPPsoPsoN);

        SubjectiveOpinion seqFuse123 = soPP.cumulativeFuse(soP).cumulativeFuse(soN);
        Assert.assertEquals(seqFuse123, batchFuse);
        SubjectiveOpinion seqFuse132 = soPP.cumulativeFuse(soN).cumulativeFuse(soP);
        Assert.assertEquals(seqFuse132, batchFuse);
        SubjectiveOpinion seqFuse213 = soP.cumulativeFuse(soPP).cumulativeFuse(soN);
        Assert.assertEquals(seqFuse213, batchFuse);
        SubjectiveOpinion seqFuse231 = soP.cumulativeFuse(soN).cumulativeFuse(soPP);
        Assert.assertEquals(seqFuse231, batchFuse);
        SubjectiveOpinion seqFuse321 = soN.cumulativeFuse(soP).cumulativeFuse(soPP);
        Assert.assertEquals(seqFuse321, batchFuse);
        SubjectiveOpinion seqFuse312 = soN.cumulativeFuse(soPP).cumulativeFuse(soP);
        Assert.assertEquals(seqFuse312, batchFuse);
    }

    @Test
    public void testCumulativeBatchFusion_nonDogmatic() {
        l.info("Testing correctness of SubjectiveLogic.cumulativeCollectionFuse(o) with example from FUSION 2017 paper..");

        SubjectiveOpinion batchFuse = SubjectiveOpinion.cumulativeCollectionFuse(triSourceExample);

        //test that batch fusion is correct
        Assert.assertEquals(0.651, batchFuse.getBelief(), 0.001);
        Assert.assertEquals(0.209, batchFuse.getDisbelief(), 0.001);
        Assert.assertEquals(0.140, batchFuse.getUncertainty(), 0.001);

        //associativity test
        Assert.assertEquals(SubjectiveOpinion.cumulativeFuse(triSourceExample), batchFuse);
    }

    @Test
    public void testCumulativeBatchFusion_mixed() {
        l.info("Testing SubjectiveLogic.cumulativeCollectionFuse(o)..");

        SubjectiveOpinion batchFuse = SubjectiveOpinion.cumulativeCollectionFuse(this.soPDsoNDsoP);

        Assert.assertTrue(batchFuse.isConsistent());
    }

    @Test
    public void testCumulativeBatchFusion_dogmatic() {
        l.info("Testing SubjectiveLogic.cumulativeCollectionFuse(o)..");

        SubjectiveOpinion batchFuse = SubjectiveOpinion.cumulativeCollectionFuse(this.soPDsoPDsoND);

        Assert.assertTrue(batchFuse.isConsistent());
    }

    @Test
    public void testCumulativeFusion_tri_associative_dogmatic() {
        l.info("Testing associativity of SubjectiveLogic.cumulativeFuse(o) for dogmatic inputs (tri-source)");

        SubjectiveOpinion batchFuse = SubjectiveOpinion.cumulativeFuse(soPDsoNDsoND);

        SubjectiveOpinion seqFuse123 = soPD.cumulativeFuse(soND).cumulativeFuse(soND);
        Assert.assertEquals(seqFuse123, batchFuse);
        SubjectiveOpinion seqFuse132 = soPD.cumulativeFuse(soND).cumulativeFuse(soND);
        Assert.assertEquals(seqFuse132, batchFuse);
        SubjectiveOpinion seqFuse213 = soND.cumulativeFuse(soPD).cumulativeFuse(soND);
        Assert.assertEquals(seqFuse213, batchFuse);
        SubjectiveOpinion seqFuse231 = soND.cumulativeFuse(soND).cumulativeFuse(soPD);
        Assert.assertEquals(seqFuse231, batchFuse);
        SubjectiveOpinion seqFuse312 = soND.cumulativeFuse(soPD).cumulativeFuse(soND);
        Assert.assertEquals(seqFuse312, batchFuse);
        SubjectiveOpinion seqFuse321 = soND.cumulativeFuse(soND).cumulativeFuse(soPD);
        Assert.assertEquals(seqFuse321, batchFuse);
    }

    @Test
    public void testRelativeWeights() {

        SubjectiveOpinion intermediate = soPD.cumulativeFuse(soPD);

        Assert.assertEquals(intermediate.cumulativeFuse(soND),
                soND.cumulativeFuse(intermediate));

        SubjectiveOpinion final_opinion = intermediate.cumulativeFuse(soND);

        //nondogmatic + dogmatic = dogmatic
        Assert.assertEquals(soPP.cumulativeFuse(soPD), soPD);
        Assert.assertEquals(soPP.cumulativeFuse(soPD), soPD);
        Assert.assertEquals(soPP.cumulativeFuse(soND), soND);
        Assert.assertEquals(soPP.cumulativeFuse(intermediate), intermediate);
        Assert.assertEquals(soPP.cumulativeFuse(final_opinion), final_opinion);

        SubjectiveOpinion nondogmaticIntermediate = soPP.cumulativeFuse(soPD);

        //(nondogmatic + dogmatic) + dogmatic = dogmatic + dogmatic
        Assert.assertEquals(nondogmaticIntermediate.cumulativeFuse(soPD), soPD);
        Assert.assertEquals(nondogmaticIntermediate.cumulativeFuse(soPD), soPD.cumulativeFuse(soPD));
        Assert.assertEquals(nondogmaticIntermediate.cumulativeFuse(intermediate), intermediate);

        //broken: (cases where not all dogmatic opinions agree..)
        Assert.assertEquals(nondogmaticIntermediate.cumulativeFuse(soND), soPD.cumulativeFuse(soND));
        Assert.assertEquals(nondogmaticIntermediate.cumulativeFuse(final_opinion), final_opinion.cumulativeFuse(soPD));
    }

    @Test
    public void testCumulativeFusion() {
        l.info("Testing Cumulative Fusion");

        List<SubjectiveOpinion> opinions = pastMisbehavior();
        SubjectiveOpinion result = SubjectiveOpinion.cumulativeFuse(opinions);
        l.info(String.format("Cumulative fused opinion: %s", result.toString()));

        List<SubjectiveOpinion> opinions2 = new ArrayList<>();
        opinions2.add(new SubjectiveOpinion(soP));
        opinions2.add(new SubjectiveOpinion(soN));
        opinions2.add(new SubjectiveOpinion(soN));
        opinions2.add(new SubjectiveOpinion(soP));
        SubjectiveOpinion result2 = SubjectiveOpinion.cumulativeFuse(opinions2);
        l.info(String.format("Cumulative fused opinion: %s", result2.toString()));

        List<SubjectiveOpinion> opinions3 = new ArrayList<>();
        opinions3.add(new SubjectiveOpinion(soP));
        opinions3.add(new SubjectiveOpinion(soN));
        opinions3.add(new SubjectiveOpinion(soP));
        opinions3.add(new SubjectiveOpinion(soN));
        SubjectiveOpinion result3 = SubjectiveOpinion.cumulativeFuse(opinions3);

        Assert.assertEquals(result2, result3);

        List<SubjectiveOpinion> opinions4 = new ArrayList<>();
        opinions4.add(new SubjectiveOpinion(soPD));
        opinions4.add(new SubjectiveOpinion(soND));
        opinions4.add(new SubjectiveOpinion(soP));
        opinions4.add(new SubjectiveOpinion(soN));
        opinions4.add(new SubjectiveOpinion(soND));
        SubjectiveOpinion result4 = SubjectiveOpinion.cumulativeFuse(opinions4);

        List<SubjectiveOpinion> opinions5 = new ArrayList<>();
        opinions5.add(new SubjectiveOpinion(soND));
        opinions5.add(new SubjectiveOpinion(soP));
        opinions5.add(new SubjectiveOpinion(soN));
        opinions5.add(new SubjectiveOpinion(soPD));
        opinions5.add(new SubjectiveOpinion(soND));
        SubjectiveOpinion result5 = SubjectiveOpinion.cumulativeFuse(opinions5);

        SubjectiveOpinion realRes4 = SubjectiveOpinion.cumulativeCollectionFuse(opinions4);
        SubjectiveOpinion realRes5 = SubjectiveOpinion.cumulativeCollectionFuse(opinions5);
        Assert.assertEquals(realRes4, realRes5);
        Assert.assertEquals(result4, result5);
        Assert.assertEquals(result4, realRes4);
        Assert.assertEquals(result5, realRes5);
    }

}
