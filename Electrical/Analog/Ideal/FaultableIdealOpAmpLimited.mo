within MSLFaults.Electrical.Analog.Ideal;
model FaultableIdealOpAmpLimited
  "Limited operational amplifier with offset, gain and output faults"
  import SI = Modelica.Units.SI;
  Modelica.Electrical.Analog.Interfaces.PositivePin in_p "Positive input"
    annotation (Placement(transformation(extent={{-110,-70},{-90,-50}}),iconTransformation(extent={{-110,-70},{-90,-50}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin in_n "Negative input"
    annotation (Placement(transformation(extent={{-110,50},{-90,70}}),iconTransformation(extent={{-110,50},{-90,70}})));
  Modelica.Electrical.Analog.Interfaces.PositivePin out "Output"
    annotation (Placement(transformation(extent={{90,-10},{110,10}})));
  Modelica.Electrical.Analog.Interfaces.PositivePin VMax "Positive output limit"
    annotation (Placement(transformation(extent={{-10,90},{10,110}}),iconTransformation(extent={{-10,90},{10,110}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin VMin "Negative output limit"
    annotation (Placement(transformation(extent={{-10,-110},{10,-90}}),iconTransformation(extent={{-10,-110},{10,-90}})));

  type FaultMode=enumeration(
    Normal "Nominal behaviour",
    InputOffsetDrift "Progressive equivalent input-offset drift",
    GainLoss "Finite differential gain",
    OutputLimitLoss "Reduced available output swing",
    OutputStuck "Drive output towards a calibrated fixed failure value");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mode";
  parameter Real severity(min=0,max=1)=1 "Multiplier applied to the fault envelope";
  parameter SI.Time faultStartTime=1 "Inclusive start of the scheduled fault window";
  parameter SI.Time faultEndTime=3 "Exclusive end of the scheduled fault window";
  parameter SI.Time transitionTime(min=0)=0.05 "Duration of a full-scale activation or recovery ramp";
  parameter SI.Time driftTime(min=Modelica.Constants.small)=1 "Time to reach the full progressive-drift target";
  parameter SI.Voltage inputOffsetFault=0.01 "Equivalent differential input offset at full effect";
  parameter Real gainFault(min=Modelica.Constants.small)=10
    "Denominator of the inverse-slope term at full effect";
  parameter Real outputSwingFactor(min=0,max=1)=0.5
    "Remaining fraction of the rail-to-rail output span";
  parameter SI.Voltage outputStuckValue=0
    "Fixed output used by the system-level OutputStuck abstraction";

  SI.Voltage vin "Physical differential input";
  SI.Voltage vin_effective "Differential input including offset fault";
  SI.Voltage VMax_effective "Effective upper output limit";
  SI.Voltage VMin_effective "Effective lower output limit";
  SI.Voltage out_internal "Output before the prescribed stuck-output transformation";
  Real faultActivation(min=0,max=1) "Severity-scaled fault envelope";
  Real driftActivation(min=0,max=1) "Fault envelope multiplied by progressive drift";
  Real startActivation(min=0,max=1) "Activation envelope divided by severity";
  Real endActivation(min=0,max=1) "One while active or recovering, otherwise zero";
  Real driftProgress(min=0,max=1) "Elapsed activation time divided by driftTime, limited to one";
  Real inverseGain(min=0) "Inverse-slope term in the normalised amplifier characteristic";
  Real outputSpanFactor(min=0,max=1) "Remaining fraction of the output-limit span";
protected
  Real s(start=0,final unit="1");
  constant SI.Voltage unitVoltage=1 "Voltage scale for the normalised characteristic" annotation(HideResult=true);

public
  parameter Boolean enabled=true "Enable the selected fault mode";
  parameter Boolean useExternalTrigger=false "Use trigger instead of the scheduled fault window" annotation(Evaluate=true);
  Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger
    "External fault activation request" annotation(Placement(transformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270),
      iconTransformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270)));
  output Boolean active "Fault activation request state";
  output Real effectLevel(min=0,max=1) "Severity-scaled fault envelope";
  output Integer activationCount "Number of activation starts";

protected
  MSLFaults.Utilities.Internal.Activation faultGate(enabled=enabled and faultMode<>FaultMode.Normal,
    permanent=false,severity=severity,startTime=faultStartTime,endTime=faultEndTime,
    transitionTime=transitionTime,useExternalTrigger=useExternalTrigger) "Reversible fault activation envelope";
equation
  connect(trigger,faultGate.externalTrigger);
  if not useExternalTrigger then faultGate.externalTrigger=false; end if;
  active=faultGate.active;
  effectLevel=faultGate.effectLevel;
  activationCount=faultGate.activationCount;

  startActivation=if severity>0 then effectLevel/severity else 0;
  endActivation=if active or effectLevel>0 then 1 else 0;
  faultActivation=effectLevel;
  driftProgress=min(1,max(0,faultGate.elapsedActiveTime/driftTime));
  driftActivation=effectLevel*driftProgress;

  in_p.i=0;
  in_n.i=0;
  VMax.i=0;
  VMin.i=0;
  vin=in_p.v-in_n.v;
  vin_effective=vin+(if faultMode==FaultMode.InputOffsetDrift then driftActivation*inputOffsetFault else 0);
  inverseGain=if faultMode==FaultMode.GainLoss then faultActivation/gainFault else 0;
  outputSpanFactor=if faultMode==FaultMode.OutputLimitLoss then 1-faultActivation*(1-outputSwingFactor) else 1;
  VMax_effective=(VMax.v+VMin.v)/2+outputSpanFactor*(VMax.v-VMin.v)/2;
  VMin_effective=(VMax.v+VMin.v)/2-outputSpanFactor*(VMax.v-VMin.v)/2;

  vin_effective=unitVoltage*smooth(0,if s<-1 then s+1 elseif s>1 then s-1 else 0)+unitVoltage*inverseGain*s;
  out_internal=smooth(0,if s<-1 then VMin_effective elseif s>1 then VMax_effective else
    (VMax_effective-VMin_effective)*s/2+(VMax_effective+VMin_effective)/2);
  out.v=if faultMode==FaultMode.OutputStuck then
    out_internal+faultActivation*(outputStuckValue-out_internal) else out_internal;



  annotation(defaultComponentName="opAmp",
    Documentation(info="<html><p>Read output limits from VMax and VMin without drawing current from them. InputOffsetDrift uses driftTime; GainLoss adds inverseGain to the internal characteristic. OutputLimitLoss contracts the rail span about its midpoint, and OutputStuck interpolates towards outputStuckValue.</p><p>Connect differential inputs, output, both limit pins and a feedback circuit; inspect vin_effective, out_internal and out.v.</p><p>gainFault is the inverse-slope parameter of the normalised characteristic; the dimensional small-signal output/input slope also depends on rail span. OutputStuck is applied after limiting and can lie outside the rails.</p><p>Select <code>faultMode</code> and configure the scheduled window or external trigger. <code>effectLevel</code> can remain nonzero during recovery after <code>active</code> becomes false. Nominal reference: <code>Modelica.Electrical.Analog.Ideal.IdealOpAmpLimited</code>.</p></html>"),
    Icon(coordinateSystem(preserveAspectRatio=true,extent={{-100,-100},{100,100}}),
      graphics={
        Line(points={{60,0},{90,0}}, color={0,0,255}, thickness=0.5),
        Polygon(points={{70,0},{-70,80},{-70,-80},{70,0}}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, lineColor={255,0,0}, lineThickness=0.75),
        Line(points={{-100,60},{-70,60}}, color={0,0,255}, thickness=0.5),
        Line(points={{-100,-60},{-70,-60}}, color={0,0,255}, thickness=0.5),
        Line(points={{-60,50},{-40,50}}, color={255,0,0}, thickness=0.8),
        Line(points={{-50,-40},{-50,-60}}, color={255,0,0}, thickness=0.8),
        Line(points={{-60,-50},{-40,-50}}, color={255,0,0}, thickness=0.8),
        Line(points={{70,0},{100,0}}, color={0,0,255}, thickness=0.5),
        Line(points={{-45,-10},{-10,-10},{-10,10},{20,10}}, color={255,0,0}, thickness=0.8),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Line(points={{0,40},{0,100}}, color={0,0,255}, thickness=0.5),
        Line(points={{0,-100},{0,-40}}, color={0,0,255}, thickness=0.5),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableIdealOpAmpLimited;
