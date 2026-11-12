within MSLFaults.Electrical.Analog.Basic;
model FaultableResistor
  "Fault-enhanced MSL 4.0.0 resistor with distinct drift and step laws"
  import SI = Modelica.Units.SI;

  parameter SI.Resistance R(start=1) "Resistance at reference temperature";
  parameter SI.Temperature T_ref=300.15 "Reference temperature";
  parameter SI.LinearTemperatureCoefficient alpha=0
    "Temperature coefficient of resistance";

  extends Modelica.Electrical.Analog.Interfaces.OnePort;
  extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(T=T_ref);

  type FaultMode = enumeration(
      Normal "Nominal MSL behavior",
      ResistanceDrift "Progressive resistance drift",
      ResistanceStep "Abrupt resistance change",
      OpenCircuit "Finite high-resistance open circuit",
      ShortCircuit "Finite low-resistance short circuit",
      TemperatureCoefficientDrift "Progressive temperature-coefficient drift");

  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mode";
  parameter Real severity(min=0, max=1)=1 "Fault severity";
  parameter SI.Time faultStartTime=1 "Fault start time";
  parameter SI.Time faultEndTime=3 "Fault end time";
  parameter SI.Time transitionTime(min=0)=0.05 "Activation/deactivation ramp time";
  parameter SI.Time driftTime(min=Modelica.Constants.small)=1
    "Time from drift onset to its calibrated target";
  parameter SI.Resistance R_drift=2*R "Resistance at completed drift";
  parameter SI.Resistance R_step=2*R "Resistance after an abrupt step";
  parameter SI.Resistance R_open(min=Modelica.Constants.small)=1e10
    "Finite resistance representing an open circuit";
  parameter SI.Resistance R_short(min=Modelica.Constants.small)=1e-6
    "Finite resistance representing a short circuit";
  parameter SI.LinearTemperatureCoefficient alpha_fault=2*alpha
    "Temperature coefficient at completed drift";

  SI.Resistance R_actual
    "Temperature-adjusted resistance used by the constitutive equation";
  SI.Resistance R_effective "Fault-adjusted reference resistance";
  SI.LinearTemperatureCoefficient alpha_effective
    "Fault-adjusted temperature coefficient";
  Real faultActivation(min=0, max=1) "Severity-scaled activation envelope";
  Real driftActivation(min=0, max=1) "Severity-scaled progressive drift";
  Real startActivation(min=0, max=1);
  Real endActivation(min=0, max=1);
  Real driftProgress(min=0, max=1);


public
  parameter Boolean enabled=true "Enable the selected fault mode";
  parameter Boolean useExternalTrigger=false "Use trigger instead of the internal time window" annotation(Evaluate=true);
  Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger
    annotation(Placement(transformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270),
      iconTransformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270)));
  output Boolean active "Requested or latched fault";
  output Real effectLevel(min=0,max=1) "Current realized fault effect";
  output Integer activationCount "Number of rising activations";

protected
  MSLFaults.Utilities.Internal.Activation faultGate(enabled=enabled and faultMode<>FaultMode.Normal,
    permanent=false,severity=severity,startTime=faultStartTime,endTime=faultEndTime,
    transitionTime=transitionTime,useExternalTrigger=useExternalTrigger);
initial equation
  assert(R_open>0 and R_short>0 and (1+alpha*(T_heatPort-T_ref))>0,"Finite electrical targets must be positive.");
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

  R_effective = if faultMode == FaultMode.ResistanceDrift then
      R + driftActivation*(R_drift - R)
    elseif faultMode == FaultMode.ResistanceStep then
      R + faultActivation*(R_step - R)
    elseif faultMode == FaultMode.OpenCircuit then
      R + faultActivation*(R_open - R)
    elseif faultMode == FaultMode.ShortCircuit then
      R + faultActivation*(R_short - R)
    else R;
  alpha_effective = if faultMode == FaultMode.TemperatureCoefficientDrift then
      alpha + driftActivation*(alpha_fault - alpha) else alpha;

  assert((1 + alpha_effective*(T_heatPort - T_ref)) >= Modelica.Constants.eps,
    "Temperature outside the valid linear resistance model range");
  R_actual=R_effective*(1+alpha_effective*(T_heatPort-T_ref));
  v = R_actual*i;
  LossPower = v*i;


  assert(1+alpha_effective*(T_heatPort-T_ref)>0,"Electrical temperature correction must stay positive.");
  assert(R_actual>=0,"The passive electrical parameter must be nonnegative.");
  annotation (
    Documentation(info="<html><p><b>Purpose</b> FaultableResistor</p><p><b>Interfaces</b> extends Modelica.Electrical.Analog.Interfaces.OnePort; extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(T=T_ref); Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger; output Boolean active \"Requested or latched fault\"; output Real effectLevel(min=0,max=1) \"Current realized fault effect\"; output Integer activationCount \"Number of rising activations\"</p><p><b>Parameters</b> parameter SI.Resistance R(start=1) \"Resistance at reference temperature\"; parameter SI.Temperature T_ref=300.15 \"Reference temperature\"; parameter SI.LinearTemperatureCoefficient alpha=0; parameter FaultMode faultMode=FaultMode.Normal \"Selected fault mode\"; parameter Real severity(min=0, max=1)=1 \"Fault severity\"; parameter SI.Time faultStartTime=1 \"Fault start time\"; parameter SI.Time faultEndTime=3 \"Fault end time\"; parameter SI.Time transitionTime(min=0)=0.05 \"Activation/deactivation ramp time\"; parameter SI.Time driftTime(min=Modelica.Constants.small)=1; parameter SI.Resistance R_drift=2*R \"Resistance at completed drift\"; parameter SI.Resistance R_step=2*R \"Resistance after an abrupt step\"; parameter SI.Resistance R_open(min=Modelica.Constants.small)=1e10</p><p><b>Behavior</b> FaultableResistor</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.ResistorReplacement\">MSLFaults.Examples.ResistorReplacement</a></p><p><b>Purpose</b> Fault-enhanced Resistor</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> The reference-temperature value is corrected by the native linear temperature law. Fault targets are finite, in the same reference-temperature units.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Electrical.Analog.Basic.Resistor</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.ResistorReplacement\">MSLFaults.Examples.ResistorReplacement</a></p><p>用法：将 FaultableResistor 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p>
<p>Independent fault-enhanced implementation of
<code>Modelica.Electrical.Analog.Basic.Resistor</code>. Normal mode and
<code>severity=0</code> reproduce the MSL 4.0.0 constitutive and heat-loss
equations exactly.</p>
<p><code>ResistanceDrift</code> uses the separate progressive clock
<code>driftTime</code>; <code>ResistanceStep</code>, open circuit and short circuit
use the activation envelope. Open and short faults remain finite and never change
the connection topology.</p>
<p><b>Evidence:</b> resistor corrosion and aging support parametric drift,
resistive shorts and opens (Amin et al., Microelectronics Reliability 52 (2012),
doi:10.1016/j.microrel.2012.02.020). The equation mapping is evidence level A/B;
target values require component-specific calibration.</p>
</html>"),
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
      graphics={
        Rectangle(extent={{-70,30},{70,-30}}, lineColor={255,0,0}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, lineThickness=0.75),
        Line(points={{-90,0},{-70,0}}, color={0,0,255}, thickness=0.5),
        Line(points={{70,0},{90,0}}, color={0,0,255}, thickness=0.5),
        Text(extent={{-150,-40},{150,-80}}, textString="R=%R"),
        Line(visible=useHeatPort, points={{0,-100},{0,-30}}, color={127,0,0}, pattern=LinePattern.Dot, thickness=0.5),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableResistor;
