within MSLFaults.Blocks.Continuous;
block FaultableFirstOrder "First-order response with parameter and output faults"
  import Init=Modelica.Blocks.Types.Init;
  parameter Real k(unit="1")=1 "Gain";
  parameter Modelica.Units.SI.Time T(start=1) "Time constant";
  parameter Init initType=Init.NoInit  "Initialisation option for the internal state" annotation(Evaluate=true,Dialog(group="Initialization"));
  parameter Real y_start=0  "Initial state or output value" annotation(Dialog(group="Initialization"));
  extends Modelica.Blocks.Interfaces.SISO(y(start=y_start));
  type FaultMode=enumeration(Normal "Nominal component relation", GainDrift "Interpolate the gain towards its target", TimeConstantDrift "Interpolate the time constant towards its target", Bias "Add the configured output offset", Stuck "Reduce the state derivative to zero at full effect", Dropout "Attenuate the output towards zero", Freeze "Blend towards the output captured at activation");
  parameter FaultMode faultMode=FaultMode.Normal "Selected component-specific fault mechanism";
  parameter Real severity(min=0,max=1)=1 "Maximum activation-envelope level";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Inclusive start of the internal fault window";
  parameter Modelica.Units.SI.Time faultEndTime=3 "Exclusive end of the internal fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Time for a full unit-level transition";
  parameter Real kFault=0.5*k "Gain target at full effect";
  parameter Modelica.Units.SI.Time TFault(min=Modelica.Constants.eps)=2*T "Time-constant target at full effect";
  parameter Real biasFault=1 "Additive output bias at full effect";
  Real faultActivation(min=0,max=1),startActivation(min=0,max=1),endActivation(min=0,max=1),k_effective;
  Modelica.Units.SI.Time T_effective;
  Real x(start=y_start);

public
  parameter Boolean enabled=true "Enable requests for the selected fault mode";
  parameter Boolean useExternalTrigger=false "Use the external trigger instead of the internal fault window" annotation(Evaluate=true);
  Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger
     "External fault request when enabled by useExternalTrigger" annotation(Placement(transformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270),
      iconTransformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270)));
  output Boolean active "Requested or latched fault state before transition shaping";
  output Real effectLevel(min=0,max=1) "Severity-scaled activation envelope";
  output Integer activationCount "Number of activation starts, including initial activation";
  discrete Real captured(start=0,fixed=true) "Signal value captured at activation";
protected
  MSLFaults.Utilities.Internal.Activation faultGate(enabled=enabled and faultMode<>FaultMode.Normal,
    permanent=false,severity=severity,startTime=faultStartTime,endTime=faultEndTime,
    transitionTime=transitionTime,useExternalTrigger=useExternalTrigger);
initial equation
  if initType==Init.SteadyState then
    der(x)=0;
  elseif initType==Init.InitialState or initType==Init.InitialOutput then
    x=y_start;
  end if;
  assert(T>0 and TFault>0,"First-order time constants must be positive.");
equation
  connect(trigger,faultGate.externalTrigger);
  if not useExternalTrigger then faultGate.externalTrigger=false; end if;
  active=faultGate.active;
  effectLevel=faultGate.effectLevel;
  activationCount=faultGate.activationCount;

  startActivation=if severity>0 then effectLevel/severity else 0;
  endActivation=if active or effectLevel>0 then 1 else 0;
  faultActivation=effectLevel;
  k_effective=if faultMode==FaultMode.GainDrift then k+faultActivation*(kFault-k) else k;
  T_effective=if faultMode==FaultMode.TimeConstantDrift then T+faultActivation*(TFault-T) else T;
  der(x)=(if faultMode==FaultMode.Stuck then 1-faultActivation else 1)*(k_effective*u-x)/T_effective;
  y=if faultMode==FaultMode.Freeze then x+faultActivation*(captured-x) else if faultMode==FaultMode.Bias then x+faultActivation*biasFault elseif faultMode==FaultMode.Dropout then (1-faultActivation)*x else x;

  when {initial(),active} then captured=x; end when;
  annotation(
    Documentation(info="<html><p>In Normal mode, use the first-order state relation <code>der(x)=(k*u-x)/T</code> with the selected initialisation option. <code>GainDrift</code> and <code>TimeConstantDrift</code> interpolate towards <code>kFault</code> and <code>TFault</code> using the activation envelope.</p><p><code>Stuck</code> reduces the state derivative and stops state evolution at full effect. <code>Freeze</code> blends the output towards the state captured at activation while the internal state continues to evolve. <code>Bias</code> and <code>Dropout</code> modify the output only. Use positive nominal and fault time constants; inspect <code>x</code>, <code>y</code> and <code>effectLevel</code> during recovery.</p></html>"),
    Icon(graphics={
        Line(points={{-74,60},{-74,-65},{67,-65}}, color={128,128,128}, thickness=0.45, pattern=LinePattern.Solid),
        Line(points={{-68,-65},{-64,-51.0194},{-60,-38.784},{-56,-28.0758},{-52,-18.7044},{-48,-10.5027},{-44,-3.32484},{-40,2.95704},{-36,8.45478},{-32,13.2662},{-28,17.4771},{-24,21.1624},{-20,24.3876},{-16,27.2102},{-12,29.6805},{-8,31.8424},{-4,33.7345},{0,35.3904},{4,36.8396},{8,38.1079},{12,39.2179},{16,40.1893},{20,41.0394},{24,41.7835},{28,42.4346},{32,43.0045},{36,43.5033},{40,43.9397},{44,44.3217},{48,44.6561},{52,44.9486},{56,45.2047},{60,45.4288},{64,45.6249}}, color={255,0,0}, thickness=0.9, pattern=LinePattern.Solid),
        Text(extent={{-37,-21},{40,12}}, textString="PT1", textColor={64,64,64}, fontSize=0),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableFirstOrder;
