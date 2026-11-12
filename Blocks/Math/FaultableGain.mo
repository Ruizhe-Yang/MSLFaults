within MSLFaults.Blocks.Math;
block FaultableGain "Independent fault-enhanced MSL 4.0.0 Gain"
  parameter Real k(start=1,unit="1") "Gain value multiplied with input signal";
  Modelica.Blocks.Interfaces.RealInput u annotation(Placement(transformation(extent={{-140,-20},{-100,20}})));
  Modelica.Blocks.Interfaces.RealOutput y annotation(Placement(transformation(extent={{100,-10},{120,10}})));
  type FaultMode=enumeration(Normal, GainDrift, Bias, Stuck, Dropout, Saturation, Freeze "Hold the value captured on activation");
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter Modelica.Units.SI.Time faultStartTime=1;
  parameter Modelica.Units.SI.Time faultEndTime=3;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05;
  parameter Real kFault=0.5*k;
  parameter Real biasFault=1;
  parameter Real stuckValue=0;
  parameter Real saturationLimit=1e9;
  Real faultActivation(min=0,max=1),startActivation(min=0,max=1),endActivation(min=0,max=1),k_effective,y_raw;

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
  assert(saturationLimit>=0,"The gain output limit must be nonnegative.");
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
  y_raw=k_effective*u;
  y=if faultMode==FaultMode.Freeze then y_raw+faultActivation*(captured-y_raw) else if faultMode==FaultMode.Bias then y_raw+faultActivation*biasFault elseif faultMode==FaultMode.Stuck then y_raw+faultActivation*(stuckValue-y_raw) elseif faultMode==FaultMode.Dropout then (1-faultActivation)*y_raw elseif faultMode==FaultMode.Saturation then y_raw+faultActivation*(min(saturationLimit,max(-saturationLimit,y_raw))-y_raw) else y_raw;

  when {initial(),active} then captured=y_raw; end when;
  annotation(
    Documentation(info="<html><p><b>Purpose</b> Independent fault-enhanced MSL 4.0.0 Gain</p><p><b>Interfaces</b> Modelica.Blocks.Interfaces.RealInput u; Modelica.Blocks.Interfaces.RealOutput y; Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger; output Boolean active \"Requested or latched fault\"; output Real effectLevel(min=0,max=1) \"Current realized fault effect\"; output Integer activationCount \"Number of rising activations\"</p><p><b>Parameters</b> parameter Real k(start=1,unit=\"1\") \"Gain value multiplied with input signal\"; parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter Modelica.Units.SI.Time faultStartTime=1; parameter Modelica.Units.SI.Time faultEndTime=3; parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05; parameter Real kFault=0.5*k; parameter Real biasFault=1; parameter Real stuckValue=0; parameter Real saturationLimit=1e9; parameter Boolean enabled=true \"Enable the selected fault mode\"; parameter Boolean useExternalTrigger=false \"Use trigger instead of the internal time window\"</p><p><b>Behavior</b> Independent fault-enhanced MSL 4.0.0 Gain</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.SensorAndSignalFaults\">MSLFaults.Examples.SensorAndSignalFaults</a></p><p><b>Purpose</b> Fault-enhanced Gain</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> Freeze captures the nominal signal at the activation edge. Subsequent physical evolution is unaffected.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Blocks.Math.Gain</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.SensorAndSignalFaults\">MSLFaults.Examples.SensorAndSignalFaults</a></p><p>用法：将 FaultableGain 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p></html>"),
    Icon(coordinateSystem(preserveAspectRatio=true,extent={{-100,-100},{100,100}}),
      graphics={
        Polygon(points={{-100,-100},{-100,100},{100,0},{-100,-100}}, lineColor={255,0,0}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, lineThickness=0.75),
        Text(extent={{-150,-140},{150,-100}}, textString="k=%k"),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Text(extent={{-64,30},{25,-30}}, textString="k", textColor={255,0,0}, fontSize=0),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableGain;
