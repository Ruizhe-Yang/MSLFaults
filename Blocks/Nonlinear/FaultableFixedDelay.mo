within MSLFaults.Blocks.Nonlinear;
block FaultableFixedDelay "Independent fault-enhanced MSL 4.0.0 FixedDelay"
  extends Modelica.Blocks.Interfaces.SISO;
  parameter Modelica.Units.SI.Time delayTime(start=1) "Delay time";
  type FaultMode=enumeration(Normal, DelayDrift, DelayIncrease, DelayDecrease, Bias, Dropout);
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter Modelica.Units.SI.Time faultStartTime=1;
  parameter Modelica.Units.SI.Time faultEndTime=3;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05;
  parameter Modelica.Units.SI.Time delayTimeFault(min=0)=2*delayTime;
  parameter Real biasFault=1;
  final parameter Modelica.Units.SI.Time delayMax=max(delayTime,delayTimeFault);
  Real faultActivation(min=0,max=1),startActivation(min=0,max=1),endActivation(min=0,max=1),y_raw;
  Modelica.Units.SI.Time delayTime_effective;

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
equation
  connect(trigger,faultGate.externalTrigger);
  if not useExternalTrigger then faultGate.externalTrigger=false; end if;
  active=faultGate.active;
  effectLevel=faultGate.effectLevel;
  activationCount=faultGate.activationCount;

  startActivation=if severity>0 then effectLevel/severity else 0;
  endActivation=if active or effectLevel>0 then 1 else 0;
  faultActivation=effectLevel;
  delayTime_effective=if faultMode==FaultMode.DelayDrift or faultMode==FaultMode.DelayIncrease or faultMode==FaultMode.DelayDecrease then delayTime+faultActivation*(delayTimeFault-delayTime) else delayTime;
  y_raw=delay(u,delayTime_effective,delayMax);
  y=if faultMode==FaultMode.Bias then y_raw+faultActivation*biasFault elseif faultMode==FaultMode.Dropout then (1-faultActivation)*y_raw else y_raw;


  annotation(
    Documentation(info="<html><p><b>Purpose</b> Independent fault-enhanced MSL 4.0.0 FixedDelay</p><p><b>Interfaces</b> Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger; output Boolean active \"Requested or latched fault\"; output Real effectLevel(min=0,max=1) \"Current realized fault effect\"; output Integer activationCount \"Number of rising activations\"</p><p><b>Parameters</b> parameter Modelica.Units.SI.Time delayTime(start=1) \"Delay time\"; parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter Modelica.Units.SI.Time faultStartTime=1; parameter Modelica.Units.SI.Time faultEndTime=3; parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05; parameter Modelica.Units.SI.Time delayTimeFault(min=0)=2*delayTime; parameter Real biasFault=1; final parameter Modelica.Units.SI.Time delayMax=max(delayTime,delayTimeFault); parameter Boolean enabled=true \"Enable the selected fault mode\"; parameter Boolean useExternalTrigger=false \"Use trigger instead of the internal time window\"</p><p><b>Behavior</b> Independent fault-enhanced MSL 4.0.0 FixedDelay</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.SensorAndSignalFaults\">MSLFaults.Examples.SensorAndSignalFaults</a></p><p><b>Purpose</b> Fault-enhanced FixedDelay</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> The established physical equations and fault modes use the common activation and severity law.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Blocks.Nonlinear.FixedDelay</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.SensorAndSignalFaults\">MSLFaults.Examples.SensorAndSignalFaults</a></p><p>用法：将 FaultableFixedDelay 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p></html>"),
    Icon(graphics={
        Line(points={{-74,58},{-74,-64},{68,-64}}, color={128,128,128}, thickness=0.45, pattern=LinePattern.Solid),
        Line(points={{-63,-45},{-38,-45},{-38,40},{62,40}}, color={128,128,128}, thickness=0.5, pattern=LinePattern.Dash),
        Line(points={{-63,-45},{12,-45},{12,40},{62,40}}, color={255,0,0}, thickness=0.9, pattern=LinePattern.Solid),
        Line(points={{-38,-5},{12,-5}}, color={64,64,64}, thickness=0.6, pattern=LinePattern.Solid, arrow={Arrow.None,Arrow.Filled}),
        Text(extent={{-34,-34},{8,-12}}, textString="T", textColor={64,64,64}, fontSize=0),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableFixedDelay;
