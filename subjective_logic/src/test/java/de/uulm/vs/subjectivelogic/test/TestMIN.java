package de.uulm.vs.subjectivelogic.test;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.junit.Assert;
import org.junit.Test;

import de.tum.i4.subjectivelogic.SubjectiveOpinion;

public class TestMIN extends TestFusionSetup {
    private final Logger l = LogManager.getLogger(getClass());

    @Test
    public void testMin(){
        l.info("Testing minima of the examples used for other tests..");
        SubjectiveOpinion min = null;

        min = SubjectiveOpinion.minimumCollectionFuse(this.soPDsoNDsoND);
        Assert.assertEquals(min, soND);

        min = SubjectiveOpinion.minimumCollectionFuse(this.soPDsoNDsoP);
        Assert.assertEquals(min, soND);

        min = SubjectiveOpinion.minimumCollectionFuse(this.soPDsoPDsoND);
        Assert.assertEquals(min, soND);

        min = SubjectiveOpinion.minimumCollectionFuse(this.soPPsoPsoN);
        Assert.assertEquals(min, soN);

        min = SubjectiveOpinion.minimumCollectionFuse(this.triSourceExample);
        Assert.assertEquals(min, C1);
    }

}
