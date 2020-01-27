package de.uulm.vs.subjectivelogic.test;

import java.util.ArrayList;
import java.util.List;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.junit.Assert;
import org.junit.Test;

import de.tum.i4.subjectivelogic.SubjectiveOpinion;

public class TestMajority extends TestFusionSetup {
    private final Logger l = LogManager.getLogger(getClass());

    @Test
    public void testMajority(){
        l.info("Testing majorities of the examples used for other tests..");
        SubjectiveOpinion maj = null;

        maj = SubjectiveOpinion.majorityCollectionFuse(this.soPDsoNDsoND);
        Assert.assertEquals(maj, soND);

        maj = SubjectiveOpinion.majorityCollectionFuse(this.soPDsoNDsoP);
        Assert.assertEquals(maj, soPD);

        maj = SubjectiveOpinion.majorityCollectionFuse(this.soPDsoPDsoND);
        Assert.assertEquals(maj, soPD);

        maj = SubjectiveOpinion.majorityCollectionFuse(this.soPPsoPsoN);
        Assert.assertEquals(maj, soPD);

        maj = SubjectiveOpinion.majorityCollectionFuse(this.triSourceExample);
        Assert.assertEquals(maj, soPD);
    }

    @Test
    public void testTied(){
        l.info("Testing ties");
        List<SubjectiveOpinion> l = new ArrayList<SubjectiveOpinion>(this.soPDsoNDsoND);
        l.add(soPD);
        Assert.assertEquals(soVacuous, SubjectiveOpinion.majorityCollectionFuse(l));
    }

    @Test
    public void testUndecided(){
        l.info("Testing with different vacuous and undecided opinions");
        List<SubjectiveOpinion> l = new ArrayList<SubjectiveOpinion>(this.soPDsoNDsoND);
        l.add(soVacuous);
        Assert.assertEquals(soND, SubjectiveOpinion.majorityCollectionFuse(l));

        l = new ArrayList<SubjectiveOpinion>(this.soPDsoNDsoND);
        l.add(soNeutral);
        Assert.assertEquals(soND, SubjectiveOpinion.majorityCollectionFuse(l));

    }
}
