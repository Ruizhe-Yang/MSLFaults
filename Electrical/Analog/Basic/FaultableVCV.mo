within MSLFaults.Electrical.Analog.Basic;
model FaultableVCV "Voltage-controlled voltage source with gain faults"
  import SI = Modelica.Units.SI;
  extends Modelica.Electrical.Analog.Interfaces.TwoPort;
  parameter Real gain(start=1) "Voltage gain";

  type FaultMode = enumeration(Normal "Nominal behaviour", GainDrift "Gain change", OutputLoss "Attenuate the output towards zero", StuckGain "Interpolate towards a prescribed gain");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mode";
  parameter Real severity(min=0,max=1)=1 "Multiplier applied to the fault envelope";
  parameter SI.Time faultStartTime=1 "Inclusive start of the scheduled fault window";
  parameter SI.Time faultEndTime=3 "Exclusive end of the scheduled fault window";
  parameter SI.Time transitionTime(min=0)=0.05 "Duration of a full-scale activation or recovery ramp";
  Real faultActivation(min=0,max=1) "Severity-scaled fault envelope";
  Real startActivation(min=0,max=1) "Activation envelope divided by severity";
  Real endActivation(min=0,max=1) "One while active or recovering, otherwise zero";
  parameter Real gainFault=0.5*gain "Gain at full fault effect";
  Real gain_effective "Effective gain";

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
  gain_effective = if faultMode == FaultMode.GainDrift or faultMode == FaultMode.StuckGain then gain + faultActivation*(gainFault - gain)
    elseif faultMode == FaultMode.OutputLoss then gain*(1 - faultActivation) else gain;
  v2 = v1*gain_effective;
  i1 = 0;


  annotation (defaultComponentName="vcv",
    Documentation(info="<html><p>Apply v2=v1*gain_effective using the two-port current and voltage conventions. The controlling port draws zero current. The selected drift or StuckGain mode interpolates the voltage gain towards gainFault; OutputLoss scales it towards zero.</p><p>Connect both ports, set gain and gainFault, and inspect the effective transfer parameter and port output.</p><p>The drift and StuckGain modes share the same reversible interpolation; StuckGain does not capture or latch a previous gain.</p><p>Select <code>faultMode</code> and configure the scheduled window or external trigger. <code>effectLevel</code> can remain nonzero during recovery after <code>active</code> becomes false. Nominal reference: <code>Modelica.Electrical.Analog.Basic.VCV</code>.</p></html>"),
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
            100}}),
      graphics={
        Rectangle(extent={{-80,80},{80,-80}}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, lineColor={255,0,0}, lineThickness=0.75),
        Line(points={{90,100},{40,100},{40,-100},{100,-100}}, color={0,0,255}, thickness=0.5),
        Ellipse(extent={{20,20},{60,-20}}, lineColor={0,0,255}, lineThickness=0.5),
        Line(points={{-20,60},{20,60}}, color={0,0,255}, thickness=0.5),
        Polygon(points={{20,60},{10,63},{10,57},{20,60}}, fillColor={0,0,255}, fillPattern=FillPattern.Solid, lineColor={0,0,255}, lineThickness=0.5),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Line(points={{-90,100},{-40,100},{-40,62}}, color={0,0,255}, thickness=0.5),
        Line(points={{-90,-100},{-40,-100},{-40,-60}}, color={0,0,255}, thickness=0.5),
        Text(extent={{108,94},{139,62}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableVCV;
