within MSLFaults.Blocks.Nonlinear;
block FaultableLimiter "Limit the range of a signal"
  parameter Real uMax(start=1) "Upper limits of input signals";
  parameter Real uMin= -uMax "Lower limits of input signals";
  parameter Boolean strict=false "= true, if strict limits with noEvent(..)"
    annotation (Evaluate=true, choices(checkBox=true), Dialog(tab="Advanced"));
  parameter Modelica.Blocks.Types.LimiterHomotopy homotopyType = Modelica.Blocks.Types.LimiterHomotopy.Linear "Simplified model for homotopy-based initialization"
    annotation (Evaluate=true, Dialog(group="Initialization"));
  extends Modelica.Blocks.Interfaces.SISO;
  type FaultMode=enumeration(Normal, UpperLimitDrift, LowerLimitDrift, Stuck);
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter Modelica.Units.SI.Time faultStartTime=1;
  parameter Modelica.Units.SI.Time faultEndTime=3;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05;
  parameter Real upperFault=uMax;
  parameter Real lowerFault=uMin;
  parameter Real stuckValue=0;
  Real faultActivation,startActivation,endActivation;
  Real upperEffective,lowerEffective;
  Real limitedValue;
  protected
  Real simplifiedExpr "Simplified expression for homotopy-based initialization";


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
  assert(uMin<=uMax,"Reversed nominal limiter bounds.");
equation
  connect(trigger,faultGate.externalTrigger);
  if not useExternalTrigger then faultGate.externalTrigger=false; end if;
  active=faultGate.active;
  effectLevel=faultGate.effectLevel;
  activationCount=faultGate.activationCount;
  startActivation=if severity>0 then effectLevel/severity else 0;
  endActivation=if active or effectLevel>0 then 1 else 0;

  faultActivation=effectLevel;

  assert(upperEffective >= lowerEffective, "Limiter: Limits must be consistent. However, upperEffective (=" + String(upperEffective) +
                       ") < lowerEffective (=" + String(lowerEffective) + ")");
  simplifiedExpr = (if homotopyType == Modelica.Blocks.Types.LimiterHomotopy.Linear then u
                    else if homotopyType == Modelica.Blocks.Types.LimiterHomotopy.UpperLimit then upperEffective
                    else if homotopyType == Modelica.Blocks.Types.LimiterHomotopy.LowerLimit then lowerEffective
                    else 0);
  if strict then
    if homotopyType == Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy then
      limitedValue = smooth(0, noEvent(if u > upperEffective then upperEffective else if u < lowerEffective then lowerEffective else u));
    else
      limitedValue = homotopy(actual = smooth(0, noEvent(if u > upperEffective then upperEffective else if u < lowerEffective then lowerEffective else u)),
                   simplified=simplifiedExpr);
    end if;
  else
    if homotopyType == Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy then
      limitedValue = smooth(0,if u > upperEffective then upperEffective else if u < lowerEffective then lowerEffective else u);
    else
      limitedValue = homotopy(actual = smooth(0,if u > upperEffective then upperEffective else if u < lowerEffective then lowerEffective else u),
                   simplified=simplifiedExpr);
    end if;
  end if;

  upperEffective=if faultMode==FaultMode.UpperLimitDrift then uMax+faultActivation*(upperFault-uMax) else uMax;
  lowerEffective=if faultMode==FaultMode.LowerLimitDrift then uMin+faultActivation*(lowerFault-uMin) else uMin;
  y=if faultMode==FaultMode.Stuck then limitedValue+faultActivation*(stuckValue-limitedValue) else limitedValue;
  annotation (
    Documentation(info="<html><p><b>Purpose</b> Fault-enhanced limiter with native strictness and homotopy settings</p><p><b>Interfaces</b> See the named arguments, record fields, or inherited connectors.</p><p><b>Parameters</b> parameter Real uMax(start=1) \"Upper limits of input signals\"; parameter Real uMin= -uMax \"Lower limits of input signals\"; parameter Boolean strict=false \"= true, if strict limits with noEvent(..)\"; parameter Modelica.Blocks.Types.LimiterHomotopy homotopyType = Modelica.Blocks.Types.LimiterHomotopy.Linear \"Simplified model for homotopy-based initialization\"; parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter Modelica.Units.SI.Time faultStartTime=1; parameter Modelica.Units.SI.Time faultEndTime=3; parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05; parameter Real upperFault=uMax; parameter Real lowerFault=uMin; parameter Real stuckValue=0</p><p><b>Behavior</b> Fault-enhanced limiter with native strictness and homotopy settings</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> y; active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.SensorAndSignalFaults\">MSLFaults.Examples.SensorAndSignalFaults</a></p><p><b>Purpose</b> Fault-enhanced Limiter</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> Nominal strict and homotopy modes follow the native limiter. Invalid effective bounds are rejected.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Blocks.Nonlinear.Limiter</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.SensorAndSignalFaults\">MSLFaults.Examples.SensorAndSignalFaults</a></p>
<p>
The Limiter block passes its input signal as output signal
as long as the input is within the specified upper and lower
limits. If this is not the case, the corresponding limits are passed
as output.
</p>
<p>
The parameter <code>homotopyType</code> in the Advanced tab specifies the
simplified behaviour if homotopy-based initialization is used:
</p>
<ul>
<li><code>NoHomotopy</code>: the actual expression with limits is used</li>
<li><code>Linear</code>: a linear behaviour y = u is assumed (default option)</li>
<li><code>UpperLimit</code>: it is assumed that the output is stuck at the upper limit u = uMax</li>
<li><code>LowerLimit</code>: it is assumed that the output is stuck at the lower limit u = uMin</li>
</ul>
<p>
If it is known a priori in which region the input signal will be located, this option can help
a lot by removing one strong nonlinearity from the initialization problem.
</p>
</html>"), Icon(coordinateSystem(
    preserveAspectRatio=true,
    extent={{-100,-100},{100,100}}),
      graphics={
        Line(points={{0,-70.2},{0,53.04}}, color={192,192,192}, thickness=0.5),
        Polygon(points={{0,70.2},{-6.24,53.04},{6.24,53.04},{0,70.2}}, lineColor={192,192,192}, fillColor={192,192,192}, fillPattern=FillPattern.Solid, lineThickness=0.5),
        Line(points={{-70.2,0},{53.04,0}}, color={192,192,192}, thickness=0.5),
        Polygon(points={{70.2,0},{53.04,-6.24},{53.04,6.24},{70.2,0}}, lineColor={192,192,192}, fillColor={192,192,192}, fillPattern=FillPattern.Solid, lineThickness=0.5),
        Line(points={{-62.4,-54.6},{-39,-54.6},{39,54.6},{62.4,54.6}}, color={255,0,0}, thickness=0.85),
        Text(extent={{-150,-150},{150,-110}}, textString="uMax=%uMax"),
        Line(visible=strict, points={{39,54.6},{62.4,54.6}}, color={255,0,0}, thickness=0.8),
        Line(visible=strict, points={{-62.4,-54.6},{-39,-54.6}}, color={255,0,0}, thickness=0.8),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableLimiter;
