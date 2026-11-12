within MSLFaults.Blocks.Continuous;
block FaultableFirstOrder "Independent fault-enhanced MSL 4.0.0 FirstOrder"
  import Init=Modelica.Blocks.Types.Init;
  parameter Real k(unit="1")=1 "Gain";
  parameter Modelica.Units.SI.Time T(start=1) "Time constant";
  parameter Init initType=Init.NoInit annotation(Evaluate=true,Dialog(group="Initialization"));
  parameter Real y_start=0 annotation(Dialog(group="Initialization"));
  extends Modelica.Blocks.Interfaces.SISO(y(start=y_start));
  type FaultMode=enumeration(Normal, GainDrift, TimeConstantDrift, Bias, Stuck, Dropout, Freeze "Hold the value captured on activation");
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter Modelica.Units.SI.Time faultStartTime=1;
  parameter Modelica.Units.SI.Time faultEndTime=3;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05;
  parameter Real kFault=0.5*k;
  parameter Modelica.Units.SI.Time TFault(min=Modelica.Constants.eps)=2*T;
  parameter Real biasFault=1;
  Real faultActivation(min=0,max=1),startActivation(min=0,max=1),endActivation(min=0,max=1),k_effective;
  Modelica.Units.SI.Time T_effective;
  Real x(start=y_start);

public
  parameter Boolean enabled=true "Enable the selected fault mode";
  parameter Boolean useExternalTrigger=false "Use trigger instead of the internal time window" annotation(Evaluate=true);
  Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger
    annotation(Placement(transformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270),
      iconTransformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270)));
  output Boolean active "Requested or latched fault";
  output Real effectLevel(min=0,max=1) "Current realized fault effect";
  output Integer activationCount "Number of rising activations";
  discrete Real captured(start=0,fixed=true);
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
    Documentation(info="<html><p><b>Purpose</b> Independent fault-enhanced MSL 4.0.0 FirstOrder</p><p><b>Interfaces</b> Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger; output Boolean active \"Requested or latched fault\"; output Real effectLevel(min=0,max=1) \"Current realized fault effect\"; output Integer activationCount \"Number of rising activations\"</p><p><b>Parameters</b> parameter Real k(unit=\"1\")=1 \"Gain\"; parameter Modelica.Units.SI.Time T(start=1) \"Time constant\"; parameter Init initType=Init.NoInit; parameter Real y_start=0; parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter Modelica.Units.SI.Time faultStartTime=1; parameter Modelica.Units.SI.Time faultEndTime=3; parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05; parameter Real kFault=0.5*k; parameter Modelica.Units.SI.Time TFault(min=Modelica.Constants.eps)=2*T; parameter Real biasFault=1</p><p><b>Behavior</b> Independent fault-enhanced MSL 4.0.0 FirstOrder</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.SensorAndSignalFaults\">MSLFaults.Examples.SensorAndSignalFaults</a></p><p><b>Purpose</b> Fault-enhanced FirstOrder</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> Freeze captures the nominal signal at the activation edge. Subsequent physical evolution is unaffected.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Blocks.Continuous.FirstOrder</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.SensorAndSignalFaults\">MSLFaults.Examples.SensorAndSignalFaults</a></p><p>用法：将 FaultableFirstOrder 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p></html>"),
    Icon(graphics={
        Line(points={{-74,60},{-74,-65},{67,-65}}, color={128,128,128}, thickness=0.45, pattern=LinePattern.Solid),
        Line(points={{-68,-65},{-64,-51.0194},{-60,-38.784},{-56,-28.0758},{-52,-18.7044},{-48,-10.5027},{-44,-3.32484},{-40,2.95704},{-36,8.45478},{-32,13.2662},{-28,17.4771},{-24,21.1624},{-20,24.3876},{-16,27.2102},{-12,29.6805},{-8,31.8424},{-4,33.7345},{0,35.3904},{4,36.8396},{8,38.1079},{12,39.2179},{16,40.1893},{20,41.0394},{24,41.7835},{28,42.4346},{32,43.0045},{36,43.5033},{40,43.9397},{44,44.3217},{48,44.6561},{52,44.9486},{56,45.2047},{60,45.4288},{64,45.6249}}, color={255,0,0}, thickness=0.9, pattern=LinePattern.Solid),
        Text(extent={{-37,-21},{40,12}}, textString="PT1", textColor={64,64,64}, fontSize=0),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableFirstOrder;
