within MSLFaults.Electrical.Analog.Sources;
model FaultableSignalVoltage "Signal-controlled voltage source with output faults"
  type FaultMode=enumeration(Normal "Nominal behaviour", OutputLoss "Attenuate the output towards zero", AmplitudeDrift "Scale the complete nominal waveform", AmplitudeDrop "Scale the complete nominal waveform", OffsetDrift "Additive output offset", StuckOutput "Interpolate towards a prescribed output");
  extends Modelica.Electrical.Analog.Icons.VoltageSource;
  Modelica.Electrical.Analog.Interfaces.PositivePin p "Positive electrical pin" annotation(Placement(transformation(extent={{-110,-10},{-90,10}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin n "Negative electrical pin" annotation(Placement(transformation(extent={{110,-10},{90,10}})));
  Modelica.Blocks.Interfaces.RealInput v(unit="V") annotation(Placement(transformation(origin={0,120},extent={{-20,-20},{20,20}},rotation=270)));
  Modelica.Units.SI.Current i "Current flowing from pin p to pin n";
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mode";
  parameter Real severity(min=0,max=1)=1 "Multiplier applied to the fault envelope";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Inclusive start of the scheduled fault window";
  parameter Modelica.Units.SI.Time faultEndTime=3 "Exclusive end of the scheduled fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a full-scale activation or recovery ramp";
  parameter Real amplitudeFault=0.5 "Scale factor applied to the complete nominal output at full effect";
  parameter Real offsetFault=0 "Additive electrical-output offset at full effect";
  parameter Real frequencyFault=0.5 "Unused by this source waveform";
  parameter Real stuckOutput=0 "Prescribed electrical output at full effect";
  Real faultActivation(min=0,max=1) "Severity-scaled fault envelope";
  Real startActivation(min=0,max=1) "Activation envelope divided by severity";
  Real endActivation(min=0,max=1) "One while active or recovering, otherwise zero";
  Real nominalOutput "Waveform before output scaling, offset and stuck-value transformation";
  Real effectiveOutput "Electrical output after the selected fault transformation";

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
  effectiveOutput = if faultMode == FaultMode.OutputLoss then nominalOutput*(1-faultActivation)
    elseif faultMode == FaultMode.AmplitudeDrift or faultMode == FaultMode.AmplitudeDrop then nominalOutput*(1+faultActivation*(amplitudeFault-1))
    elseif faultMode == FaultMode.OffsetDrift then nominalOutput+faultActivation*offsetFault
    elseif faultMode == FaultMode.StuckOutput then nominalOutput+faultActivation*(stuckOutput-nominalOutput)
    else nominalOutput;
  nominalOutput = v;
  p.v - n.v = effectiveOutput;
  0 = p.i + n.i;
  i = p.i;


  annotation (
    Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
            100,100}}), graphics={
                            Line(points={{-109,20},{-84,
          20}}, color={160,160,164}),Polygon(
            points={{-94,23},{-84,20},{-94,17},{-94,23}},
            lineColor={160,160,164},
            fillColor={160,160,164},
            fillPattern=FillPattern.Solid),Line(points={{91,20},{116,20}},
          color={160,160,164}),Text(
            extent={{-109,25},{-89,45}},
            textColor={160,160,164},
            textString="i"),Polygon(
            points={{106,23},{116,20},{106,17},{106,23}},
            lineColor={160,160,164},
            fillColor={160,160,164},
            fillPattern=FillPattern.Solid),Text(
            extent={{91,45},{111,25}},
            textColor={160,160,164},
            textString="i"),Line(points={{-119,-5},{-119,5}}, color={160,160,164}),
          Line(points={{-124,0},{-114,0}}, color={160,160,164}),Line(
          points={{116,0},{126,0}}, color={160,160,164})}),
    Documentation(info="<html><p>An external real signal provides the commanded electrical output. OutputLoss scales the output towards zero; amplitude and offset modes change its magnitude; StuckOutput interpolates towards stuckOutput.</p><p>Connect the commanded voltage to input v. Connect p and n with the prescribed voltage sign convention and inspect effectiveOutput.</p><p>AmplitudeDrift and AmplitudeDrop use the same scale interpolation, including any nominal offset. StuckOutput uses a fixed target. frequencyFault has no effect for this source.</p><p>Select <code>faultMode</code> and configure the scheduled window or external trigger. <code>effectLevel</code> can remain nonzero during recovery after <code>active</code> becomes false. Nominal reference: <code>Modelica.Electrical.Analog.Sources.SignalVoltage</code>.</p></html>"),
    Icon(graphics={
        Ellipse(extent={{-70,70},{70,-70}}, lineColor={255,0,0}, lineThickness=0.75),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
  
end FaultableSignalVoltage;
