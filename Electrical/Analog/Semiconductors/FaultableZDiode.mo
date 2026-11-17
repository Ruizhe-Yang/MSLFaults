within MSLFaults.Electrical.Analog.Semiconductors;
model FaultableZDiode "Zener diode with junction, breakdown and leakage faults"
  import SI = Modelica.Units.SI;
  extends Modelica.Electrical.Analog.Interfaces.OnePort;
  parameter SI.Current Ids=1e-6;
  parameter SI.Voltage Vt(min=Modelica.Constants.small)=0.04 "Nominal junction voltage scale";
  parameter Real Maxexp(final min=Modelica.Constants.small)=30 "Upper argument for exponential-to-linear continuation";
  parameter SI.Resistance R=1e8;
  parameter SI.Voltage Bv=5.1 "Nominal reverse breakdown voltage";
  parameter SI.Current Ibv=0.7 "Breakdown-current scale";
  parameter Real Nbv=0.74 "Breakdown emission coefficient";
  extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(T=293.15);

  type FaultMode=enumeration(
    Normal "Nominal behaviour",
    SaturationCurrentDrift "Progressive transport-current drift",
    ForwardVoltageDrift "Forward-conduction voltage change",
    BreakdownVoltageDrift "Progressive Zener-voltage drift",
    ReverseLeakageIncrease "Parallel leakage increase",
    OpenCircuit "Finite open-circuit equivalent",
    ShortCircuit "Finite short-circuit equivalent");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mode";
  parameter Real severity(min=0,max=1)=1 "Multiplier applied to the fault envelope";
  parameter SI.Time faultStartTime=1 "Inclusive start of the scheduled fault window";
  parameter SI.Time faultEndTime=3 "Exclusive end of the scheduled fault window";
  parameter SI.Time transitionTime(min=0)=0.05 "Duration of a full-scale activation or recovery ramp";
  parameter SI.Time driftTime(min=Modelica.Constants.small)=1 "Time to reach the full progressive-drift target";
  parameter SI.Current IdsFault=10*Ids "Saturation current at completed drift";
  parameter SI.Voltage VtFault=1.5*Vt "Junction voltage scale at completed drift";
  parameter SI.Voltage BvFault=0.7*Bv "Breakdown voltage at completed drift";
  parameter SI.Resistance RLeakFault=R/100 "Parallel resistance at full leakage effect";
  parameter SI.Resistance ROpen=1e12 "Finite open-circuit resistance";
  parameter SI.Resistance RShort=1e-6 "Finite short-circuit resistance";

  Real faultActivation(min=0,max=1) "Severity-scaled fault envelope";
  Real driftActivation(min=0,max=1) "Fault envelope multiplied by progressive drift";
  Real startActivation(min=0,max=1) "Activation envelope divided by severity";
  Real endActivation(min=0,max=1) "One while active or recovering, otherwise zero";
  Real driftProgress(min=0,max=1) "Elapsed activation time divided by driftTime, limited to one";
  Real diodeConductionFactor(min=0,max=1) "Remaining fraction of semiconductor conduction";
  SI.Current Ids_effective "Effective saturation current";
  SI.Voltage Vt_effective "Effective junction voltage scale";
  SI.Voltage Bv_effective "Effective breakdown voltage";
  SI.Resistance R_effective "Effective parallel junction resistance";
  SI.Current i_semiconductor;

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

  Ids_effective=if faultMode==FaultMode.SaturationCurrentDrift then
    Ids+driftActivation*(IdsFault-Ids) else Ids;
  Vt_effective=if faultMode==FaultMode.ForwardVoltageDrift then
    Vt+driftActivation*(VtFault-Vt) else Vt;
  Bv_effective=if faultMode==FaultMode.BreakdownVoltageDrift then
    Bv+driftActivation*(BvFault-Bv) else Bv;
  R_effective=if faultMode==FaultMode.ReverseLeakageIncrease then
      R+faultActivation*(RLeakFault-R)
    elseif faultMode==FaultMode.OpenCircuit then R+faultActivation*(ROpen-R)
    elseif faultMode==FaultMode.ShortCircuit then R+faultActivation*(RShort-R)
    else R;
  diodeConductionFactor=if faultMode==FaultMode.OpenCircuit then
    1-faultActivation else 1;

  i_semiconductor=smooth(1,if v>Maxexp*Vt_effective then
      Ids_effective*(exp(Maxexp)*(1+v/Vt_effective-Maxexp)-1)
    elseif (v+Bv_effective)<-Maxexp*(Nbv*Vt_effective) then
      -Ids_effective-Ibv*exp(Maxexp)*(1-(v+Bv_effective)/(Nbv*Vt_effective)-Maxexp)
    else Ids_effective*(exp(v/Vt_effective)-1)-
      Ibv*exp(-(v+Bv_effective)/(Nbv*Vt_effective)));
  i=diodeConductionFactor*i_semiconductor+v/R_effective;
  LossPower=v*i;



  annotation(defaultComponentName="diode",
    Documentation(info="<html><p>Combine forward and reverse-breakdown exponential currents with a parallel resistance. Saturation-current, forward-voltage-scale and breakdown-voltage drift use driftTime. Leakage changes the parallel resistance; OpenCircuit suppresses the semiconductor current and ShortCircuit adds a finite shunt.</p><p>Set Bv and the other nominal junction parameters, then configure BvFault, IdsFault, VtFault or the selected resistance target.</p><p>The electrical law has no temperature dependence; the optional heat port exports dissipated power. The short-circuit mode retains both junction-current terms.</p><p>Select <code>faultMode</code> and configure the scheduled window or external trigger. <code>effectLevel</code> can remain nonzero during recovery after <code>active</code> becomes false. Nominal reference: <code>Modelica.Electrical.Analog.Semiconductors.ZDiode</code>.</p></html>"),
    Icon(coordinateSystem(preserveAspectRatio=true,extent={{-100,-100},{100,100}}),
      graphics={
        Polygon(points={{30,0},{-30,40},{-30,-40},{30,0}}, lineColor={255,0,0}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, lineThickness=0.75),
        Line(points={{-90,0},{40,0}}, color={255,0,0}, thickness=0.8),
        Line(points={{40,0},{90,0}}, color={255,0,0}, thickness=0.8),
        Line(points={{30,40},{30,-40}}, color={255,0,0}, thickness=0.8),
        Line(points={{30,-40},{20,-40}}, color={255,0,0}, thickness=0.8),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableZDiode;
