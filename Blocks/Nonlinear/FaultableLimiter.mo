within MSLFaults.Blocks.Nonlinear;
block FaultableLimiter "Signal limiter with bound changes and a fixed-output fault"
  parameter Real uMax(start=1) "Nominal upper limit";
  parameter Real uMin= -uMax "Nominal lower limit";
  parameter Boolean strict=false "Suppress events in the limiting relation when true"
    annotation (Evaluate=true, choices(checkBox=true), Dialog(tab="Advanced"));
  parameter Modelica.Blocks.Types.LimiterHomotopy homotopyType = Modelica.Blocks.Types.LimiterHomotopy.Linear "Initialisation approximation for the limiting relation"
    annotation (Evaluate=true, Dialog(group="Initialization"));
  extends Modelica.Blocks.Interfaces.SISO;
  type FaultMode=enumeration(Normal "Nominal component relation", UpperLimitDrift "Interpolate towards the upper-limit target", LowerLimitDrift "Interpolate towards the lower-limit target", Stuck "Blend towards the prescribed fixed output");
  parameter FaultMode faultMode=FaultMode.Normal "Selected component-specific fault mechanism";
  parameter Real severity(min=0,max=1)=1 "Maximum activation-envelope level";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Inclusive start of the internal fault window";
  parameter Modelica.Units.SI.Time faultEndTime=3 "Exclusive end of the internal fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Time for a full unit-level transition";
  parameter Real upperFault=uMax "Upper-limit target at full effect";
  parameter Real lowerFault=uMin "Lower-limit target at full effect";
  parameter Real stuckValue=0 "Prescribed output at full fixed-value effect";
  Real faultActivation,startActivation,endActivation;
  Real upperEffective,lowerEffective;
  Real limitedValue;
  protected
  Real simplifiedExpr "Simplified expression used during homotopy initialisation";


public
  parameter Boolean enabled=true "Enable requests for the selected fault mode";
  parameter Boolean useExternalTrigger=false "Use the external trigger instead of the internal fault window" annotation(Evaluate=true);
  Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger
     "External fault request when enabled by useExternalTrigger" annotation(Placement(transformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270),
      iconTransformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270)));
  output Boolean active "Requested or latched fault state before transition shaping";
  output Real effectLevel(min=0,max=1) "Severity-scaled activation envelope";
  output Integer activationCount "Number of activation starts, including initial activation";

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
    Documentation(info="<html><p>Limit <code>u</code> between <code>uMin</code> and <code>uMax</code>. <code>UpperLimitDrift</code> or <code>LowerLimitDrift</code> interpolates the selected bound towards <code>upperFault</code> or <code>lowerFault</code>. The effective upper bound must remain at least the effective lower bound.</p><p><code>Stuck</code> blends the limited signal towards <code>stuckValue</code>, which may lie outside the bounds. <code>strict</code> controls event suppression in the limiting relation and <code>homotopyType</code> selects the initialisation approximation. Inspect <code>limitedValue</code>, <code>y</code> and the effective bounds.</p></html>"), Icon(coordinateSystem(
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
