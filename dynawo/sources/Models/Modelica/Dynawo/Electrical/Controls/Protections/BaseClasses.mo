within Dynawo.Electrical.Controls.Protections;

package BaseClasses

  extends Icons.BasesPackage;

  partial model BaseUFLS
    import Modelica.Constants;
    import Dynawo.Connectors;
    import Dynawo.NonElectrical.Logs.Timeline;
    import Dynawo.NonElectrical.Logs.TimelineKeys;

    parameter Types.Time tLagAction "Time-lag due to the actual trip action in s";

    parameter Integer nbSteps(min = 1, max = 10) "Number of steps in the UFLS scheme";

    parameter Real omega1Pu "Frequency of the 1st UFLS speed in pu (base omegaNom)";
    parameter Real omega2Pu if (nbSteps >= 2) "Frequency of the 2nd UFLS speed in pu (base omegaNom)";
    parameter Real omega3Pu if (nbSteps >= 3) "Frequency of the 3rd UFLS speed in pu (base omegaNom)";
    parameter Real omega4Pu if (nbSteps >= 4) "Frequency of the 4th UFLS speed in pu (base omegaNom)";
    parameter Real omega5Pu if (nbSteps >= 5) "Frequency of the 5th UFLS speed in pu (base omegaNom)";
    parameter Real omega6Pu if (nbSteps >= 6) "Frequency of the 6th UFLS speed in pu (base omegaNom)";
    parameter Real omega7Pu if (nbSteps >= 7) "Frequency of the 7th UFLS speed in pu (base omegaNom)";
    parameter Real omega8Pu if (nbSteps >= 8) "Frequency of the 8th UFLS speed in pu (base omegaNom)";
    parameter Real omega9Pu if (nbSteps >= 9) "Frequency of the 9th UFLS speed in pu (base omegaNom)";
    parameter Real omega10Pu if (nbSteps >= 10) "Frequency of the 10th UFLS speed in pu (base omegaNom)";

    parameter Real UFLSStep1 "Share of load disconnected by step 1 of UFLS (sum of all steps should be at most 1)";
    parameter Real UFLSStep2 if (nbSteps >= 2) "Share of load disconnected by step 2 of UFLS";
    parameter Real UFLSStep3 if (nbSteps >= 3) "Share of load disconnected by step 3 of UFLS";
    parameter Real UFLSStep4 if (nbSteps >= 4) "Share of load disconnected by step 4 of UFLS";
    parameter Real UFLSStep5 if (nbSteps >= 5) "Share of load disconnected by step 5 of UFLS";
    parameter Real UFLSStep6 if (nbSteps >= 6) "Share of load disconnected by step 6 of UFLS";
    parameter Real UFLSStep7 if (nbSteps >= 7) "Share of load disconnected by step 7 of UFLS";
    parameter Real UFLSStep8 if (nbSteps >= 8) "Share of load disconnected by step 8 of UFLS";
    parameter Real UFLSStep9 if (nbSteps >= 9) "Share of load disconnected by step 9 of UFLS";
    parameter Real UFLSStep10 if (nbSteps >= 10) "Share of load disconnected by step 10 of UFLS";

    Types.Frequency omegaMonitoredPu "Monitored frequency";

    Types.Time tThresholdReached1(start = Constants.inf) "Time when reaches 1st UFLS threshold";
    Types.Time tThresholdReached2(start = Constants.inf) if (nbSteps >= 2) "Time when reaches 2nd UFLS threshold";
    Types.Time tThresholdReached3(start = Constants.inf) if (nbSteps >= 3) "Time when reaches 3rd UFLS threshold";
    Types.Time tThresholdReached4(start = Constants.inf) if (nbSteps >= 4) "Time when reaches 4th UFLS threshold";
    Types.Time tThresholdReached5(start = Constants.inf) if (nbSteps >= 5) "Time when reaches 5th UFLS threshold";
    Types.Time tThresholdReached6(start = Constants.inf) if (nbSteps >= 6) "Time when reaches 6th UFLS threshold";
    Types.Time tThresholdReached7(start = Constants.inf) if (nbSteps >= 7) "Time when reaches 7th UFLS threshold";
    Types.Time tThresholdReached8(start = Constants.inf) if (nbSteps >= 8) "Time when reaches 8th UFLS threshold";
    Types.Time tThresholdReached9(start = Constants.inf) if (nbSteps >= 9) "Time when reaches 9th UFLS threshold";
    Types.Time tThresholdReached10(start = Constants.inf) if (nbSteps >= 10) "Time when reaches 10th UFLS threshold";

    Boolean step1Activated (start = false) "True if step 1 of UFLS has been activated";
    Boolean step2Activated (start = false) if (nbSteps >= 2) "True if step 2 of UFLS has been activated";
    Boolean step3Activated (start = false) if (nbSteps >= 3) "True if step 3 of UFLS has been activated";
    Boolean step4Activated (start = false) if (nbSteps >= 4) "True if step 4 of UFLS has been activated";
    Boolean step5Activated (start = false) if (nbSteps >= 5) "True if step 5 of UFLS has been activated";
    Boolean step6Activated (start = false) if (nbSteps >= 6) "True if step 6 of UFLS has been activated";
    Boolean step7Activated (start = false) if (nbSteps >= 7) "True if step 7 of UFLS has been activated";
    Boolean step8Activated (start = false) if (nbSteps >= 8) "True if step 8 of UFLS has been activated";
    Boolean step9Activated (start = false) if (nbSteps >= 9) "True if step 9 of UFLS has been activated";
    Boolean step10Activated (start = false) if (nbSteps >= 10) "True if step 10 of UFLS has been activated";

    Real deltaPQ (start = 0) "Reduction of P and Q due to UFLS disconnections, to connect to the variables deltaP and deltaQ in the load model";

  equation
    // Frequency comparison with first UFLS step
    when omegaMonitoredPu <= omega1Pu and not pre(step1Activated) then
      tThresholdReached1 = time;
      Timeline.logEvent1(TimelineKeys.UFLS1Arming);
    end when;

    // Frequency comparison with step 2 of UFLS
    if nbSteps >= 2 then
      when omegaMonitoredPu <= omega2Pu and not pre(step2Activated) then
        tThresholdReached2 = time;
        Timeline.logEvent1(TimelineKeys.UFLS2Arming);
      end when;
    end if;

    // Frequency comparison with step 3 of UFLS
    if nbSteps >= 3 then
      when omegaMonitoredPu <= omega3Pu and not pre(step3Activated) then
        tThresholdReached3 = time;
        Timeline.logEvent1(TimelineKeys.UFLS3Arming);
      end when;
    end if;

    // Frequency comparison with step 4 of UFLS
    if nbSteps >= 4 then
      when omegaMonitoredPu <= omega4Pu and not pre(step4Activated) then
        tThresholdReached4 = time;
        Timeline.logEvent1(TimelineKeys.UFLS4Arming);
      end when;
    end if;

    // Frequency comparison with step 5 of UFLS
    if nbSteps >= 5 then
      when omegaMonitoredPu <= omega5Pu and not pre(step5Activated) then
        tThresholdReached5 = time;
        Timeline.logEvent1(TimelineKeys.UFLS5Arming);
      end when;
    end if;

    // Frequency comparison with step 6 of UFLS
    if nbSteps >= 6 then
      when omegaMonitoredPu <= omega6Pu and not pre(step6Activated) then
        tThresholdReached6 = time;
        Timeline.logEvent1(TimelineKeys.UFLS6Arming);
      end when;
    end if;

    // Frequency comparison with step 7 of UFLS
    if nbSteps >= 7 then
      when omegaMonitoredPu <= omega7Pu and not pre(step7Activated) then
        tThresholdReached7 = time;
        Timeline.logEvent1(TimelineKeys.UFLS7Arming);
      end when;
    end if;

    // Frequency comparison with step 8 of UFLS
    if nbSteps >= 8 then
      when omegaMonitoredPu <= omega8Pu and not pre(step8Activated) then
        tThresholdReached8 = time;
        Timeline.logEvent1(TimelineKeys.UFLS8Arming);
      end when;
    end if;

    // Frequency comparison with step 9 of UFLS
    if nbSteps >= 9 then
      when omegaMonitoredPu <= omega9Pu and not pre(step9Activated) then
        tThresholdReached9 = time;
        Timeline.logEvent1(TimelineKeys.UFLS9Arming);
      end when;
    end if;

    // Frequency comparison with step 10 of UFLS
    if nbSteps >= 10 then
      when omegaMonitoredPu <= omega10Pu and not pre(step10Activated) then
        tThresholdReached10 = time;
        Timeline.logEvent1(TimelineKeys.UFLS10Arming);
      end when;
    end if;


    // Trips
    when time - tThresholdReached1 >= tLagAction then
      step1Activated = true;
      Timeline.logEvent1(TimelineKeys.UFLS1Activated);
    end when;
    if nbSteps >= 2 then
      when time - tThresholdReached2 >= tLagAction then
        step2Activated = true;
        Timeline.logEvent1(TimelineKeys.UFLS2Activated);
      end when;
    end if;
    if nbSteps >= 3 then
      when time - tThresholdReached3 >= tLagAction then
        step3Activated = true;
        Timeline.logEvent1(TimelineKeys.UFLS3Activated);
      end when;
    end if;
    if nbSteps >= 4 then
      when time - tThresholdReached4 >= tLagAction then
        step4Activated = true;
        Timeline.logEvent1(TimelineKeys.UFLS4Activated);
      end when;
    end if;
    if nbSteps >= 5 then
      when time - tThresholdReached5 >= tLagAction then
        step5Activated = true;
        Timeline.logEvent1(TimelineKeys.UFLS5Activated);
      end when;
    end if;
    if nbSteps >= 6 then
      when time - tThresholdReached6 >= tLagAction then
        step6Activated = true;
        Timeline.logEvent1(TimelineKeys.UFLS6Activated);
      end when;
    end if;
    if nbSteps >= 7 then
      when time - tThresholdReached7 >= tLagAction then
        step7Activated = true;
        Timeline.logEvent1(TimelineKeys.UFLS7Activated);
      end when;
    end if;
    if nbSteps >= 8 then
      when time - tThresholdReached8 >= tLagAction then
        step8Activated = true;
        Timeline.logEvent1(TimelineKeys.UFLS8Activated);
      end when;
    end if;
    if nbSteps >= 9 then
      when time - tThresholdReached9 >= tLagAction then
        step9Activated = true;
        Timeline.logEvent1(TimelineKeys.UFLS9Activated);
      end when;
    end if;
    if nbSteps >= 10 then
      when time - tThresholdReached10 >= tLagAction then
        step10Activated = true;
        Timeline.logEvent1(TimelineKeys.UFLS10Activated);
      end when;
    end if;

    if nbSteps >= 10 then
      if step10Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2 - UFLSStep3 - UFLSStep4 - UFLSStep5 - UFLSStep6 - UFLSStep7 - UFLSStep8 - UFLSStep9 - UFLSStep10;
      elseif step9Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2 - UFLSStep3 - UFLSStep4 - UFLSStep5 - UFLSStep6 - UFLSStep7 - UFLSStep8 - UFLSStep9;
      elseif step8Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2 - UFLSStep3 - UFLSStep4 - UFLSStep5 - UFLSStep6 - UFLSStep7 - UFLSStep8;
      elseif step7Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2 - UFLSStep3 - UFLSStep4 - UFLSStep5 - UFLSStep6 - UFLSStep7;
      elseif step6Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2 - UFLSStep3 - UFLSStep4 - UFLSStep5 - UFLSStep6;
      elseif step5Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2 - UFLSStep3 - UFLSStep4 - UFLSStep5;
      elseif step4Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2 - UFLSStep3 - UFLSStep4;
      elseif step3Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2 - UFLSStep3;
      elseif step2Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2;
      elseif step1Activated then
        deltaPQ = - UFLSStep1;
      else
        deltaPQ = 0;
      end if;

    elseif nbSteps >= 9 then
      if step9Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2 - UFLSStep3 - UFLSStep4 - UFLSStep5 - UFLSStep6 - UFLSStep7 - UFLSStep8 - UFLSStep9;
      elseif step8Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2 - UFLSStep3 - UFLSStep4 - UFLSStep5 - UFLSStep6 - UFLSStep7 - UFLSStep8;
      elseif step7Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2 - UFLSStep3 - UFLSStep4 - UFLSStep5 - UFLSStep6 - UFLSStep7;
      elseif step6Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2 - UFLSStep3 - UFLSStep4 - UFLSStep5 - UFLSStep6;
      elseif step5Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2 - UFLSStep3 - UFLSStep4 - UFLSStep5;
      elseif step4Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2 - UFLSStep3 - UFLSStep4;
      elseif step3Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2 - UFLSStep3;
      elseif step2Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2;
      elseif step1Activated then
        deltaPQ = - UFLSStep1;
      else
        deltaPQ = 0;
      end if;

    elseif nbSteps >= 8 then
      if step8Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2 - UFLSStep3 - UFLSStep4 - UFLSStep5 - UFLSStep6 - UFLSStep7 - UFLSStep8;
      elseif step7Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2 - UFLSStep3 - UFLSStep4 - UFLSStep5 - UFLSStep6 - UFLSStep7;
      elseif step6Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2 - UFLSStep3 - UFLSStep4 - UFLSStep5 - UFLSStep6;
      elseif step5Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2 - UFLSStep3 - UFLSStep4 - UFLSStep5;
      elseif step4Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2 - UFLSStep3 - UFLSStep4;
      elseif step3Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2 - UFLSStep3;
      elseif step2Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2;
      elseif step1Activated then
        deltaPQ = - UFLSStep1;
      else
        deltaPQ = 0;
      end if;

    elseif nbSteps >= 7 then
      if step7Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2 - UFLSStep3 - UFLSStep4 - UFLSStep5 - UFLSStep6 - UFLSStep7;
      elseif step6Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2 - UFLSStep3 - UFLSStep4 - UFLSStep5 - UFLSStep6;
      elseif step5Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2 - UFLSStep3 - UFLSStep4 - UFLSStep5;
      elseif step4Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2 - UFLSStep3 - UFLSStep4;
      elseif step3Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2 - UFLSStep3;
      elseif step2Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2;
      elseif step1Activated then
        deltaPQ = - UFLSStep1;
      else
        deltaPQ = 0;
      end if;

    elseif nbSteps >= 6 then
      if step6Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2 - UFLSStep3 - UFLSStep4 - UFLSStep5 - UFLSStep6;
      elseif step5Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2 - UFLSStep3 - UFLSStep4 - UFLSStep5;
      elseif step4Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2 - UFLSStep3 - UFLSStep4;
      elseif step3Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2 - UFLSStep3;
      elseif step2Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2;
      elseif step1Activated then
        deltaPQ = - UFLSStep1;
      else
        deltaPQ = 0;
      end if;

    elseif nbSteps >= 5 then
      if step5Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2 - UFLSStep3 - UFLSStep4 - UFLSStep5;
      elseif step4Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2 - UFLSStep3 - UFLSStep4;
      elseif step3Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2 - UFLSStep3;
      elseif step2Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2;
      elseif step1Activated then
        deltaPQ = - UFLSStep1;
      else
        deltaPQ = 0;
      end if;

    elseif nbSteps >= 4 then
      if step4Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2 - UFLSStep3 - UFLSStep4;
      elseif step3Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2 - UFLSStep3;
      elseif step2Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2;
      elseif step1Activated then
        deltaPQ = - UFLSStep1;
      else
        deltaPQ = 0;
      end if;

    elseif nbSteps >= 3 then
      if step3Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2 - UFLSStep3;
      elseif step2Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2;
      elseif step1Activated then
        deltaPQ = - UFLSStep1;
      else
        deltaPQ = 0;
      end if;

    elseif nbSteps >= 2 then
      if step2Activated then
        deltaPQ = - UFLSStep1 - UFLSStep2;
      elseif step1Activated then
        deltaPQ = - UFLSStep1;
      else
        deltaPQ = 0;
      end if;

    else
      if step1Activated then
        deltaPQ = - UFLSStep1;
      else
        deltaPQ = 0;
      end if;
    end if;

    annotation(
      preferredView = "text",
      Documentation(info = "<html><head></head><body> </body></html>"));
  end BaseUFLS;

end BaseClasses;
