within MSLFaults.Electrical.Analog.Ideal;
model FaultableIdealOpAmp "Four-pin ideal operational amplifier with input faults"
  Modelica.Units.SI.Voltage v1 "Voltage drop over the left port";
  Modelica.Units.SI.Voltage v2 "Voltage drop over the right port";
  Modelica.Units.SI.Current i1 "Current flowing from pos. to neg. pin of the left port";
  Modelica.Units.SI.Current i2 "Current flowing from pos. to neg. pin of the right port";
  Modelica.Electrical.Analog.Interfaces.PositivePin p1 "Positive pin of the left port" annotation (
      Placement(transformation(extent={{-110,-70},{-90,-50}}), iconTransformation(extent={{-110,-70},{-90,-50}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin n1 "Negative pin of the left port" annotation (
      Placement(transformation(extent={{-110,50},{-90,70}}), iconTransformation(extent={{-110,50},{-90,70}})));
  Modelica.Electrical.Analog.Interfaces.PositivePin p2 "Positive pin of the right port" annotation (
      Placement(transformation(extent={{90,-10},{110,10}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin n2 "Negative pin of the right port" annotation (
      Placement(transformation(
        origin={0,-100},
        extent={{10,-10},{-10,10}},
        rotation=270)));
  type FaultMode=enumeration(Normal "Nominal behaviour", InputOffset "Prescribed differential input offset", InputLeakage "Conductance on the constrained differential input", InputShort "High conductance on the constrained differential input");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mode";
  parameter Real severity(min=0,max=1)=1 "Multiplier applied to the fault envelope";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Inclusive start of the scheduled fault window";
  parameter Modelica.Units.SI.Time faultEndTime=3 "Exclusive end of the scheduled fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a full-scale activation or recovery ramp";
  Real faultActivation(min=0,max=1) "Severity-scaled fault envelope";
  Real startActivation(min=0,max=1) "Activation envelope divided by severity";
  Real endActivation(min=0,max=1) "One while active or recovering, otherwise zero";
  parameter Modelica.Units.SI.Voltage inputOffsetFault=0.01 "Equivalent differential input offset at full effect";
  parameter Modelica.Units.SI.Conductance inputLeakageFault=1e-6 "Input conductance at full leakage effect";

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

  v1 = p1.v - n1.v;
  v2 = p2.v - n2.v;
  0 = p1.i + n1.i;
  0 = p2.i + n2.i;
  i1 = p1.i;
  i2 = p2.i;
  v1 = if faultMode==FaultMode.InputOffset then faultActivation*inputOffsetFault else 0;
  i1 = if faultMode==FaultMode.InputLeakage then faultActivation*inputLeakageFault*v1
    elseif faultMode==FaultMode.InputShort then faultActivation*1e6*v1 else 0;


  annotation (defaultComponentName="opAmp",
    Documentation(info="<html><p>Use the ideal nullator input relation and an unconstrained output driven by the connected feedback circuit. InputOffset prescribes a differential input voltage proportional to effectLevel. InputLeakage and InputShort apply a conductance to the differential input.</p><p>Connect an appropriate feedback network and set inputOffsetFault; inspect the differential input and the circuit response.</p><p>In InputLeakage and InputShort the differential voltage is constrained to zero, so the added input current is also zero in this formulation.</p><p>Select <code>faultMode</code> and configure the scheduled window or external trigger. <code>effectLevel</code> can remain nonzero during recovery after <code>active</code> becomes false. Nominal reference: <code>Modelica.Electrical.Analog.Ideal.IdealOpAmp</code>.</p></html>"),
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
            100}}),
      graphics={
        Line(points={{60,0},{90,0}}, color={0,0,255}, thickness=0.5),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Polygon(points={{70,0},{-70,80},{-70,-80},{70,0}}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, lineColor={0,0,255}, lineThickness=0.5),
        Line(points={{0,-40},{0,-100}}, color={0,0,255}, thickness=0.5),
        Line(points={{-100,60},{-70,60}}, color={0,0,255}, thickness=0.5),
        Line(points={{-100,-60},{-70,-60}}, color={0,0,255}, thickness=0.5),
        Line(points={{70,0},{100,0}}, color={0,0,255}, thickness=0.5),
        Line(points={{-60,50},{-40,50}}, color={0,0,255}, thickness=0.5),
        Line(points={{-60,-50},{-40,-50}}, color={0,0,255}, thickness=0.5),
        Line(points={{-50,-40},{-50,-60}}, color={0,0,255}, thickness=0.5),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableIdealOpAmp;
