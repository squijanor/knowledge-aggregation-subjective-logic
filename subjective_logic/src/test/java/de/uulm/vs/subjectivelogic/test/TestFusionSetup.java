package de.uulm.vs.subjectivelogic.test;

import java.util.ArrayList;
import java.util.List;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.junit.Before;
import org.junit.Test;

import de.tum.i4.subjectivelogic.SubjectiveOpinion;

public class TestFusionSetup {
    private final Logger l = LogManager.getLogger(getClass());

    protected SubjectiveOpinion soVacuous;
    protected SubjectiveOpinion soNeutral;
    protected SubjectiveOpinion soP;
    protected SubjectiveOpinion soPP;
    protected SubjectiveOpinion soPD;
    protected SubjectiveOpinion soN;
    protected SubjectiveOpinion soNN;
    protected SubjectiveOpinion soND;
    protected List<SubjectiveOpinion> soPDsoPDsoND;
    protected List<SubjectiveOpinion> soPDsoNDsoND;
    protected List<SubjectiveOpinion> soPDsoNDsoP;
    protected List<SubjectiveOpinion> soPPsoPsoN;
    protected SubjectiveOpinion C1;
    protected SubjectiveOpinion C2;
    protected SubjectiveOpinion C3;
    protected List<SubjectiveOpinion> triSourceExample;

    private final double ATOMICITY = 0.5; // fixed baserate
    private final double NEUTRAL = 0.0;
    private final double SLIGHTLY = 0.3;
    private final double VERY = 0.7;
    private final double DOGMATIC = 1.0;


    @Before
    public void setUp() {
        l.info("Setting up opinions...");
        soVacuous = new SubjectiveOpinion(0.0, 0.0, 1.0, ATOMICITY);
        soNeutral = new SubjectiveOpinion(SLIGHTLY, SLIGHTLY, 1 - 2 * SLIGHTLY, ATOMICITY);
        soP = new SubjectiveOpinion(SLIGHTLY, 0, 1 - SLIGHTLY, ATOMICITY);
        soPP = new SubjectiveOpinion(VERY, 0, 1 - VERY, ATOMICITY);
        soN = new SubjectiveOpinion(0, SLIGHTLY, 1 - SLIGHTLY, ATOMICITY);
        soNN = new SubjectiveOpinion(0, VERY, 1 - VERY, ATOMICITY);
        soPD = new SubjectiveOpinion(1, true);
        soND = new SubjectiveOpinion(0, true);
        
        l.debug(String.format("Vacuous opinion: %s", soVacuous.toString()));
        l.debug(String.format("Neutral opinion: %s", soNeutral.toString()));
        l.debug(String.format("Slightly positive opinion: %s", soP.toString()));
        l.debug(String.format("Very positive opinion: %s", soPP.toString()));
        l.debug(String.format("Slightly negative opinion: %s", soN.toString()));
        l.debug(String.format("Very negative opinion: %s", soNN.toString()));
        l.debug(String.format("Dogmatic positive opinion: %s", soPD.toString()));
        l.debug(String.format("Dogmatic negative opinion: %s", soND.toString()));

        SubjectiveOpinion nonZero1 = new SubjectiveOpinion(soPD);
        SubjectiveOpinion nonZero2 = new SubjectiveOpinion(soPD);
        SubjectiveOpinion nonZero3 = new SubjectiveOpinion(soND);
        this.soPDsoPDsoND = new ArrayList<>();
        this.soPDsoPDsoND.add(nonZero1);
        this.soPDsoPDsoND.add(nonZero2);
        this.soPDsoPDsoND.add(nonZero3);

        nonZero1 = new SubjectiveOpinion(soPD);
        nonZero2 = new SubjectiveOpinion(soND);
        nonZero3 = new SubjectiveOpinion(soND);
        this.soPDsoNDsoND = new ArrayList<>();
        this.soPDsoNDsoND.add(nonZero1);
        this.soPDsoNDsoND.add(nonZero2);
        this.soPDsoNDsoND.add(nonZero3);

        nonZero1 = new SubjectiveOpinion(soPD);
        nonZero2 = new SubjectiveOpinion(soND);
        nonZero3 = new SubjectiveOpinion(soP);
        this.soPDsoNDsoP = new ArrayList<>();
        this.soPDsoNDsoP.add(nonZero1);
        this.soPDsoNDsoP.add(nonZero2);
        this.soPDsoNDsoP.add(nonZero3);

        nonZero1 = new SubjectiveOpinion(soPP);
        nonZero2 = new SubjectiveOpinion(soP);
        nonZero3 = new SubjectiveOpinion(soN);
        this.soPPsoPsoN = new ArrayList<>();
        this.soPPsoPsoN.add(nonZero1);
        this.soPPsoPsoN.add(nonZero2);
        this.soPPsoPsoN.add(nonZero3);

        C1 = new SubjectiveOpinion(0.1, 0.3, 0.6, 0.5);
        C2 = new SubjectiveOpinion(0.4, 0.2, 0.4, 0.5);
        C3 = new SubjectiveOpinion(0.7, 0.1, 0.2, 0.5);
        triSourceExample = new ArrayList<>();
        triSourceExample.add(C1);
        triSourceExample.add(C2);
        triSourceExample.add(C3);
    }

    @Test
    public void compareFusionOperators() {
        SubjectiveOpinion o1, o2;

        o1 = new SubjectiveOpinion(soP);
        o2 = new SubjectiveOpinion(soN);
        l.info("----- Slight belief and slight disbelief -----");
        l.info(String.format("Fusing opinions    %s and %s", o1.toString(), o2.toString()));
        l.info(String.format("Cumulative result: %s", o1.cumulativeFuse(o2).toString()));
        l.info(String.format("WBF result:        %s", o1.wbFuse(o2).toString()));
        l.info(String.format("CC result:         %s", o1.ccFuse(o2).toString()));


        o1 = new SubjectiveOpinion(soPP);
        o2 = new SubjectiveOpinion(soN);
        l.info("----- High belief and slight disbelief -----");
        l.info(String.format("Fusing opinions    %s and %s", o1.toString(), o2.toString()));
        l.info(String.format("Cumulative result: %s", o1.cumulativeFuse(o2).toString()));
        l.info(String.format("WBF result:        %s", o1.wbFuse(o2).toString()));
        l.info(String.format("CC result:         %s", o1.ccFuse(o2).toString()));

        l.info("----- High belief and slight belief -----");
        o1 = new SubjectiveOpinion(soPP);
        o2 = new SubjectiveOpinion(soP);
        l.info(String.format("Fusing opinions    %s and %s", o1.toString(), o2.toString()));
        l.info(String.format("Cumulative result: %s", o1.cumulativeFuse(o2).toString()));
        l.info(String.format("WBF result:        %s", o1.wbFuse(o2).toString()));
        l.info(String.format("CC result:         %s", o1.ccFuse(o2).toString()));

        o1 = new SubjectiveOpinion(soPD);
        o2 = new SubjectiveOpinion(soNN);
        l.info("----- Dogmatic belief and high disbelief -----");
        l.info(String.format("Fusing opinions    %s and %s", o1.toString(), o2.toString()));
        l.info(String.format("Cumulative result: %s", o1.cumulativeFuse(o2).toString()));
        l.info(String.format("WBF result:        %s", o1.wbFuse(o2).toString()));
        l.info(String.format("CC result:         %s", o1.ccFuse(o2).toString()));

        o1 = new SubjectiveOpinion(soPD);
        o2 = new SubjectiveOpinion(soND);
        l.info("----- Two contradicting dogmatic opinions -----");
        l.info(String.format("Fusing opinions    %s and %s", o1.toString(), o2.toString()));
        l.info(String.format("Cumulative result: %s", o1.cumulativeFuse(o2).toString()));
        l.info(String.format("WBF result:        %s", o1.wbFuse(o2).toString()));
        l.info(String.format("CC result:         %s", o1.ccFuse(o2).toString())); // TODO: this result does not seem right

    }

    protected List<SubjectiveOpinion> pastMisbehavior() {
        List<SubjectiveOpinion> opinions = new ArrayList<>();
        for (int i = 0; i < 2; i++) {
            opinions.add(new SubjectiveOpinion(soP));
        }
        opinions.add(new SubjectiveOpinion(soNN));
        return opinions;
    }
}
