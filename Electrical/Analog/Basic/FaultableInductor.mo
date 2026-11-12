within MSLFaults.Electrical.Analog.Basic;
model FaultableInductor
  "Fault-enhanced MSL 4.0.0 inductor with winding and turn-short effects"
  import SI = Modelica.Units.SI;
  extends Modelica.Electrical.Analog.Interfaces.OnePort(i(start=0));
  parameter SI.Inductance L(start=1) "Inductance";

  type FaultMode = enumeration(
    Normal "Nominal MSL behavior",
    InductanceDrift "Progressive inductance drift",
    InductanceLoss "Abrupt or ramped inductance loss",
    WindingResistanceIncrease "Series winding resistance increase",
    TurnShort "Reduced effective turns plus additional winding loss",
    OpenCircuit "Finite high-resistance winding open circuit");
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter SI.Time faultStartTime=1;
  parameter SI.Time faultEndTime=3;
  parameter SI.Time transitionTime(min=0)=0.05;
  parameter SI.Time driftTime(min=Modelica.Constants.small)=1;
  parameter SI.Inductance L_drift=1.2*L "Inductance at completed drift";
  parameter SI.Inductance L_loss=0.5*L "Inductance after discrete loss";
  parameter SI.Resistance R_windingFault=10 "Added winding resistance target";
  parameter Real maxTurnShortRatio(min=0,max=0.95)=0.3
    "Shorted-turn fraction at severity one";
  parameter SI.Resistance R_turnShortLoss=1
    "Series-equivalent loss at maximum shorted-turn fraction";
  parameter SI.Resistance R_open=1e10 "Finite open-circuit resistance";

  Real faultActivation(min=0,max=1);
  Real driftActivation(min=0,max=1);
  Real startActivation(min=0,max=1);
  Real endActivation(min=0,max=1);
  Real driftProgress(min=0,max=1);
  Real turnShortRatio(min=0,max=0.95);
  SI.Inductance L_effective "Fault-adjusted inductance";
  SI.Resistance R_series "Fault-adjusted winding loss/open resistance";


public
  parameter Boolean enabled=true "Enable the selected fault mode";
  parameter Boolean useExternalTrigger=false "Use trigger instead of the internal time window" annotation(Evaluate=true);
  Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger
    annotation(Placement(transformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270),
      iconTransformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270)));
  output Boolean active "Requested or latched fault";
  output Real effectLevel(min=0,max=1) "Current realized fault effect";
  output Integer activationCount "Number of rising activations";
  Modelica.Units.SI.MagneticFlux psi "Flux linkage";
  Modelica.Units.SI.Energy storedEnergy;
  Modelica.Units.SI.Power lossPower,parameterPower;
protected
  MSLFaults.Utilities.Internal.Activation faultGate(enabled=enabled and faultMode<>FaultMode.Normal,
    permanent=false,severity=severity,startTime=faultStartTime,endTime=faultEndTime,
    transitionTime=transitionTime,useExternalTrigger=useExternalTrigger);
initial equation
  assert(L>0 and L_loss>0 and L_drift>0 and driftTime>0,"Inductances and drift time must be positive.");
  assert(R_windingFault>=0 and R_turnShortLoss>=0 and R_open>0 and maxTurnShortRatio>=0 and maxTurnShortRatio<1,"Invalid winding-loss targets.");
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
  turnShortRatio = if faultMode == FaultMode.TurnShort then
      faultActivation*maxTurnShortRatio else 0;

  L_effective = if faultMode == FaultMode.InductanceDrift then
      L + driftActivation*(L_drift-L)
    elseif faultMode == FaultMode.InductanceLoss then
      L + faultActivation*(L_loss-L)
    elseif faultMode == FaultMode.TurnShort then
      L*(1-turnShortRatio)^2 else L;
  R_series = if faultMode == FaultMode.WindingResistanceIncrease then
      faultActivation*R_windingFault
    elseif faultMode == FaultMode.TurnShort then
      faultActivation*R_turnShortLoss
    elseif faultMode == FaultMode.OpenCircuit then
      faultActivation*R_open else 0;

  v=der(psi)+R_series*i;


  psi=L_effective*i;
  storedEnergy=psi*psi/(2*L_effective);
  lossPower=R_series*i*i;
  parameterPower=der(storedEnergy)-v*i+lossPower;
  annotation (
    Documentation(info="<html><p><b>Purpose</b> FaultableInductor</p><p><b>Interfaces</b> extends Modelica.Electrical.Analog.Interfaces.OnePort(i(start=0)); Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger; output Boolean active \"Requested or latched fault\"; output Real effectLevel(min=0,max=1) \"Current realized fault effect\"; output Integer activationCount \"Number of rising activations\"</p><p><b>Parameters</b> parameter SI.Inductance L(start=1) \"Inductance\"; parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter SI.Time faultStartTime=1; parameter SI.Time faultEndTime=3; parameter SI.Time transitionTime(min=0)=0.05; parameter SI.Time driftTime(min=Modelica.Constants.small)=1; parameter SI.Inductance L_drift=1.2*L \"Inductance at completed drift\"; parameter SI.Inductance L_loss=0.5*L \"Inductance after discrete loss\"; parameter SI.Resistance R_windingFault=10 \"Added winding resistance target\"; parameter Real maxTurnShortRatio(min=0,max=0.95)=0.3; parameter SI.Resistance R_turnShortLoss=1</p><p><b>Behavior</b> FaultableInductor</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.InductorDegradation\">MSLFaults.Examples.InductorDegradation</a></p><p><b>Purpose</b> Fault-enhanced Inductor</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> Flux linkage is the storage coordinate. The original current retains its initialization attributes. Apply a finite ramp for an in-run parameter-power trace.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Electrical.Analog.Basic.Inductor</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.InductorDegradation\">MSLFaults.Examples.InductorDegradation</a></p><p>用法：将 FaultableInductor 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p>
<p>Normal mode and <code>severity=0</code> reproduce the MSL equation
<code>v=L*der(i)</code>. Drift and discrete loss use distinct temporal laws.</p>
<p><code>TurnShort</code> represents a calibrated shorted-turn fraction: inductance
scales approximately with the square of effective turns and a finite
series-equivalent loss represents fault-loop heating. This is a system-level
lumped approximation, not a detailed coupled shorted-winding model.</p>
<p><b>Evidence:</b> transformer/winding inter-turn-short studies model shorted-turn
ratio, changed self/mutual inductances, fault resistance and circulating current
(Energies 18 (2025) 5453, doi:10.3390/en18205453). Evidence level B.</p>
</html>"),
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
      graphics={
        Line(points={{60,0},{90,0}}, color={0,0,255}, thickness=0.5),
        Line(points={{-90,0},{-60,0}}, color={0,0,255}, thickness=0.5),
        Text(extent={{-150,-40},{150,-80}}, textString="L=%L"),
        Line(points={{-60,0},{-59,6},{-52,14},{-38,14},{-31,6},{-30,0}}, color={255,0,0}, smooth=Smooth.Bezier, thickness=0.8),
        Line(points={{-30,0},{-29,6},{-22,14},{-8,14},{-1,6},{0,0}}, color={255,0,0}, smooth=Smooth.Bezier, thickness=0.8),
        Line(points={{0,0},{1,6},{8,14},{22,14},{29,6},{30,0}}, color={255,0,0}, smooth=Smooth.Bezier, thickness=0.8),
        Line(points={{30,0},{31,6},{38,14},{52,14},{59,6},{60,0}}, color={255,0,0}, smooth=Smooth.Bezier, thickness=0.8),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableInductor;
