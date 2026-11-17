within MSLFaults.Electrical.Analog.Ideal;
model FaultableIdealTransformer "Ideal transformer with ratio and magnetising-path faults"
  extends Modelica.Electrical.Analog.Interfaces.TwoPort;
  parameter Real n(start=1) "Ratio of primary to secondary voltage";
  parameter Boolean considerMagnetization=false
    "Choice of considering magnetisation";
  parameter Modelica.Units.SI.Inductance Lm1(start=1)
    "Magnetisation inductance w.r.t. primary side"
    annotation (Dialog(enable=considerMagnetization));
  type FaultMode=enumeration(Normal "Nominal behaviour", TurnsRatioDrift "Turns-ratio change", MagnetizationLoss "Reduced magnetising inductance", PrimaryOpen "Finite primary winding resistance", SecondaryOpen "Finite secondary winding resistance");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mode";
  parameter Real severity(min=0,max=1)=1 "Multiplier applied to the fault envelope";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Inclusive start of the scheduled fault window";
  parameter Modelica.Units.SI.Time faultEndTime=3 "Exclusive end of the scheduled fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a full-scale activation or recovery ramp";
  Real faultActivation(min=0,max=1) "Severity-scaled fault envelope";
  Real startActivation(min=0,max=1) "Activation envelope divided by severity";
  Real endActivation(min=0,max=1) "One while active or recovering, otherwise zero";
  parameter Real nFault=0.8*n "Turns ratio at full effect";
  parameter Modelica.Units.SI.Inductance Lm1Fault=0.2*Lm1 "Primary magnetising inductance at full effect";
  parameter Modelica.Units.SI.Resistance R_open=1e10 "Finite resistance representing an open circuit";
  Real n_effective "Effective turns ratio";
  Modelica.Units.SI.Inductance Lm1_effective "Effective primary magnetising inductance";
  Modelica.Units.SI.Resistance R1_series "Additional primary series resistance";
  Modelica.Units.SI.Resistance R2_series "Additional secondary series resistance";
protected
  Modelica.Units.SI.Current im1 "Magnetisation current w.r.t. primary side";
  Modelica.Units.SI.MagneticFlux psim1 "Magnetic flux w.r.t. primary side";

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
  n_effective=if faultMode==FaultMode.TurnsRatioDrift then n+faultActivation*(nFault-n) else n;
  Lm1_effective=if faultMode==FaultMode.MagnetizationLoss then Lm1+faultActivation*(Lm1Fault-Lm1) else Lm1;
  R1_series=if faultMode==FaultMode.PrimaryOpen then faultActivation*R_open else 0;
  R2_series=if faultMode==FaultMode.SecondaryOpen then faultActivation*R_open else 0;

  im1 = i1 + i2/n_effective;
  if considerMagnetization then
    psim1 = Lm1_effective*im1;
    v1 = der(psim1)+R1_series*i1;
  else
    psim1 = 0;
    im1 = 0;
  end if;
  v1 = n_effective*(v2-R2_series*i2);


  annotation (defaultComponentName="transformer",
    Documentation(info="<html><p>Modify turns ratio n, primary magnetising inductance Lm1 or a finite series resistance. With considerMagnetization=true, primary voltage includes the derivative of magnetising flux linkage. The secondary series resistance enters the voltage-ratio relation.</p><p>Set n and considerMagnetization, connect both windings, and configure nFault, Lm1Fault or R_open for the selected mode.</p><p>MagnetizationLoss and PrimaryOpen affect the equations only when considerMagnetization=true. Keep the effective turns ratio nonzero.</p><p>Select <code>faultMode</code> and configure the scheduled window or external trigger. <code>effectLevel</code> can remain nonzero during recovery after <code>active</code> becomes false. Nominal reference: <code>Modelica.Electrical.Analog.Ideal.IdealTransformer</code>.</p></html>"),
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
            100}}),
      graphics={
        Text(extent={{-150,-110},{150,-150}}, textString="n=%n"),
        Text(extent={{-100,20},{-60,-20}}, textColor={0,0,255}, textString="1"),
        Text(extent={{60,20},{100,-20}}, textColor={0,0,255}, textString="2"),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Line(points={{-40,60},{-40,100},{-90,100}}, color={0,0,255}, thickness=0.5),
        Line(points={{40,60},{40,100},{90,100}}, color={0,0,255}, thickness=0.5),
        Line(points={{-40,-60},{-40,-100},{-90,-100}}, color={0,0,255}, thickness=0.5),
        Line(points={{40,-60},{40,-100},{90,-100}}, color={0,0,255}, thickness=0.5),
        Line(points={{-15,-7},{-14,-1},{-7,7},{7,7},{14,-1},{15,-7}}, color={0,0,255}, smooth=Smooth.Bezier, origin={-33,45}, rotation=270, thickness=0.5),
        Line(points={{-15,-7},{-14,-1},{-7,7},{7,7},{14,-1},{15,-7}}, color={0,0,255}, smooth=Smooth.Bezier, origin={-33,15}, rotation=270, thickness=0.5),
        Line(points={{-15,-7},{-14,-1},{-7,7},{7,7},{14,-1},{15,-7}}, color={0,0,255}, smooth=Smooth.Bezier, origin={-33,-15}, rotation=270, thickness=0.5),
        Line(points={{-15,-7},{-14,-1},{-7,7},{7,7},{14,-1},{15,-7}}, color={0,0,255}, smooth=Smooth.Bezier, origin={-33,-45}, rotation=270, thickness=0.5),
        Line(points={{-15,-7},{-14,-1},{-7,7},{7,7},{14,-1},{15,-7}}, color={0,0,255}, smooth=Smooth.Bezier, origin={33,45}, rotation=90, thickness=0.5),
        Line(points={{-15,-7},{-14,-1},{-7,7},{7,7},{14,-1},{15,-7}}, color={0,0,255}, smooth=Smooth.Bezier, origin={33,15}, rotation=90, thickness=0.5),
        Line(points={{-15,-7},{-14,-1},{-7,7},{7,7},{14,-1},{15,-7}}, color={0,0,255}, smooth=Smooth.Bezier, origin={33,-15}, rotation=90, thickness=0.5),
        Line(points={{-15,-7},{-14,-1},{-7,7},{7,7},{14,-1},{15,-7}}, color={0,0,255}, smooth=Smooth.Bezier, origin={33,-45}, rotation=90, thickness=0.5),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableIdealTransformer;
