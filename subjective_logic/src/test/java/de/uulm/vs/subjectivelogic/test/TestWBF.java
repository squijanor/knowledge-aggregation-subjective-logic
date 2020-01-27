package de.uulm.vs.subjectivelogic.test;

import java.util.ArrayList;
import java.util.List;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.junit.Assert;
import org.junit.Test;

import de.tum.i4.subjectivelogic.SubjectiveOpinion;

public class TestWBF extends TestFusionSetup {
    private final Logger l = LogManager.getLogger(getClass());

    @Test
    public void testWeightedBatchFusion_nonDogmatic() {
        l.info("Testing correctness of SubjectiveLogic.weightedCollectionFuse(o) with example from FUSION 2018 paper..");

        SubjectiveOpinion batchFuse = SubjectiveOpinion.weightedCollectionFuse(triSourceExample);

        //test that batch fusion is correct
        Assert.assertEquals(0.562, batchFuse.getBelief(), 0.001);
        Assert.assertEquals(0.146, batchFuse.getDisbelief(), 0.001);
        Assert.assertEquals(0.292, batchFuse.getUncertainty(), 0.001);
    }

    @Test
    public void testWeightedBatchFusion_mixed() {
        l.info("Testing SubjectiveLogic.weightedCollectionFuse(o)..");

        SubjectiveOpinion batchFuse = SubjectiveOpinion.weightedCollectionFuse(this.soPDsoNDsoP);

        Assert.assertTrue(batchFuse.isConsistent());
    }

    @Test
    public void testWeightedBatchFusion_dogmatic() {
        l.info("Testing SubjectiveLogic.weightedCollectionFuse(o)..");

        SubjectiveOpinion batchFuse = SubjectiveOpinion.weightedCollectionFuse(this.soPDsoPDsoND);

        Assert.assertTrue(batchFuse.isConsistent());
    }

    @Test
    public void testWeightedFusion() {
        l.info("Testing Weighted Belief Fusion");

        List<SubjectiveOpinion> opinions = pastMisbehavior();
        SubjectiveOpinion result = SubjectiveOpinion.weightedCollectionFuse(opinions);
        l.info(String.format("Weighted fused opinion: %s", result.toString()));

        List<SubjectiveOpinion> opinions2 = new ArrayList<>();
        opinions2.add(new SubjectiveOpinion(soP));
        opinions2.add(new SubjectiveOpinion(soN));
        opinions2.add(new SubjectiveOpinion(soN));
        opinions2.add(new SubjectiveOpinion(soP));
        SubjectiveOpinion result2 = SubjectiveOpinion.weightedCollectionFuse(opinions2);
        l.info(String.format("WB fused opinion: %s", result2.toString()));

        List<SubjectiveOpinion> opinions3 = new ArrayList<>();
        opinions3.add(new SubjectiveOpinion(soP));
        opinions3.add(new SubjectiveOpinion(soN));
        opinions3.add(new SubjectiveOpinion(soP));
        opinions3.add(new SubjectiveOpinion(soN));
        SubjectiveOpinion result3 = SubjectiveOpinion.weightedCollectionFuse(opinions3);

        Assert.assertEquals(result2, result3);

        List<SubjectiveOpinion> opinions4 = new ArrayList<>();
        opinions4.add(new SubjectiveOpinion(soPD));
        opinions4.add(new SubjectiveOpinion(soND));
        opinions4.add(new SubjectiveOpinion(soP));
        opinions4.add(new SubjectiveOpinion(soN));
        opinions4.add(new SubjectiveOpinion(soND));
        SubjectiveOpinion result4 = SubjectiveOpinion.weightedCollectionFuse(opinions4);

        List<SubjectiveOpinion> opinions5 = new ArrayList<>();
        opinions5.add(new SubjectiveOpinion(soND));
        opinions5.add(new SubjectiveOpinion(soP));
        opinions5.add(new SubjectiveOpinion(soN));
        opinions5.add(new SubjectiveOpinion(soPD));
        opinions5.add(new SubjectiveOpinion(soND));
        SubjectiveOpinion result5 = SubjectiveOpinion.weightedCollectionFuse(opinions5);

        SubjectiveOpinion realRes4 = SubjectiveOpinion.weightedCollectionFuse(opinions4);
        SubjectiveOpinion realRes5 = SubjectiveOpinion.weightedCollectionFuse(opinions5);
        Assert.assertEquals(realRes4, realRes5);
        Assert.assertEquals(result4, result5);
        Assert.assertEquals(result4, realRes4);
        Assert.assertEquals(result5, realRes5);
    }
}
