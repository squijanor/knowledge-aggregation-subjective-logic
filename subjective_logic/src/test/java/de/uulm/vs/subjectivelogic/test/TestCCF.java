package de.uulm.vs.subjectivelogic.test;

import java.util.ArrayList;
import java.util.List;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.junit.Assert;
import org.junit.Test;

import de.tum.i4.subjectivelogic.SubjectiveOpinion;

public class TestCCF extends TestFusionSetup {
    private final Logger l = LogManager.getLogger(getClass());

    @Test
    public void testCCFusion_nonDogmatic() {
        l.info("Testing correctness of SubjectiveLogic.ccCollectionFuse(o) with example from FUSION 2018 paper..");

        SubjectiveOpinion batchFuse = SubjectiveOpinion.ccCollectionFuse(triSourceExample);

        //test that batch fusion is correct
        Assert.assertEquals(0.629, batchFuse.getBelief(), 0.001);
        Assert.assertEquals(0.182, batchFuse.getDisbelief(), 0.001);
        Assert.assertEquals(0.189, batchFuse.getUncertainty(), 0.001);
    }

    @Test
    public void testCCBatchFusion_mixed() {
        l.info("Testing SubjectiveLogic.ccCollectionFuse(o)..");

        SubjectiveOpinion batchFuse = SubjectiveOpinion.ccCollectionFuse(this.soPDsoNDsoP);

        Assert.assertTrue(batchFuse.isConsistent());
    }

    @Test
    public void testCCBatchFusion_dogmatic() {
        l.info("Testing SubjectiveLogic.ccCollectionFuse(o)..");

        SubjectiveOpinion batchFuse = SubjectiveOpinion.ccCollectionFuse(this.soPDsoPDsoND);

        Assert.assertTrue(batchFuse.isConsistent());
    }

    @Test
    public void testCCFusion() {
        l.info("Testing CC Fusion");

        List<SubjectiveOpinion> opinions = pastMisbehavior();
        SubjectiveOpinion result = SubjectiveOpinion.ccCollectionFuse(opinions);
        l.info(String.format("CC fused opinion: %s", result.toString()));

        List<SubjectiveOpinion> opinions2 = new ArrayList<>();
        opinions2.add(new SubjectiveOpinion(soP));
        opinions2.add(new SubjectiveOpinion(soN));
        opinions2.add(new SubjectiveOpinion(soN));
        opinions2.add(new SubjectiveOpinion(soP));
        SubjectiveOpinion result2 = SubjectiveOpinion.ccCollectionFuse(opinions2);
        l.info(String.format("WB fused opinion: %s", result2.toString()));

        List<SubjectiveOpinion> opinions3 = new ArrayList<>();
        opinions3.add(new SubjectiveOpinion(soP));
        opinions3.add(new SubjectiveOpinion(soN));
        opinions3.add(new SubjectiveOpinion(soP));
        opinions3.add(new SubjectiveOpinion(soN));
        SubjectiveOpinion result3 = SubjectiveOpinion.ccCollectionFuse(opinions3);

        Assert.assertEquals(result2, result3);

        List<SubjectiveOpinion> opinions4 = new ArrayList<>();
        opinions4.add(new SubjectiveOpinion(soPD));
        opinions4.add(new SubjectiveOpinion(soND));
        opinions4.add(new SubjectiveOpinion(soP));
        opinions4.add(new SubjectiveOpinion(soN));
        opinions4.add(new SubjectiveOpinion(soND));
        SubjectiveOpinion result4 = SubjectiveOpinion.ccCollectionFuse(opinions4);

        List<SubjectiveOpinion> opinions5 = new ArrayList<>();
        opinions5.add(new SubjectiveOpinion(soND));
        opinions5.add(new SubjectiveOpinion(soP));
        opinions5.add(new SubjectiveOpinion(soN));
        opinions5.add(new SubjectiveOpinion(soPD));
        opinions5.add(new SubjectiveOpinion(soND));
        SubjectiveOpinion result5 = SubjectiveOpinion.ccCollectionFuse(opinions5);

        SubjectiveOpinion realRes4 = SubjectiveOpinion.ccCollectionFuse(opinions4);
        SubjectiveOpinion realRes5 = SubjectiveOpinion.ccCollectionFuse(opinions5);
        Assert.assertEquals(realRes4, realRes5);
        Assert.assertEquals(result4, result5);
        Assert.assertEquals(result4, realRes4);
        Assert.assertEquals(result5, realRes5);
    }
}
