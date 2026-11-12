within MSLFaults.Fluid.Sensors;
model FaultablePressure "Pressure fluid sensor with local signal faults"
  extends Modelica.Fluid.Sensors.BaseClasses.PartialAbsoluteSensor(redeclare replaceable package Medium=Modelica.Media.Water.StandardWater constrainedby Modelica.Media.Interfaces.PartialMedium);
  Modelica.Blocks.Interfaces.RealOutput p(final quantity="Pressure",final unit="Pa",displayUnit="bar",min=0) annotation(Placement(transformation(extent={{100,-10},{120,10}})));
  type FaultMode=enumeration(Normal "正常", Bias "偏置", GainError "增益误差", NoiseIncrease "噪声增加", Stuck "输出卡死", Dropout "输出丢失", Saturation "输出饱和", Freeze "Hold the value captured on activation");
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter Modelica.Units.SI.Time faultStartTime=1;
  parameter Modelica.Units.SI.Time faultEndTime=3;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05;
  parameter Real biasFault=1;
  parameter Real gainFault=0.9;
  parameter Real noiseAmplitude=0.01;
  parameter Modelica.Units.SI.Frequency noiseFrequency=37;
  parameter Real stuckValue=0;
  parameter Real saturationLimit=1e12;
  Real faultActivation(min=0,max=1);
  Real startActivation(min=0,max=1);
  Real endActivation(min=0,max=1);
  Real measuredActual;

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
equation
  connect(trigger,faultGate.externalTrigger);
  if not useExternalTrigger then faultGate.externalTrigger=false; end if;
  active=faultGate.active;
  effectLevel=faultGate.effectLevel;
  activationCount=faultGate.activationCount;

  startActivation=if severity>0 then effectLevel/severity else 0;
  endActivation=if active or effectLevel>0 then 1 else 0;
  faultActivation=effectLevel;
  measuredActual=port.p;
  p=if faultMode==FaultMode.Freeze then measuredActual+faultActivation*(captured-measuredActual) else if faultMode==FaultMode.Bias then measuredActual+faultActivation*biasFault elseif faultMode==FaultMode.GainError then measuredActual*(1+faultActivation*(gainFault-1)) elseif faultMode==FaultMode.NoiseIncrease then measuredActual+faultActivation*noiseAmplitude*sin(2*Modelica.Constants.pi*noiseFrequency*time) elseif faultMode==FaultMode.Stuck then measuredActual+faultActivation*(stuckValue-measuredActual) elseif faultMode==FaultMode.Dropout then measuredActual*(1-faultActivation) elseif faultMode==FaultMode.Saturation then measuredActual+faultActivation*(min(saturationLimit,max(-saturationLimit,measuredActual))-measuredActual) else measuredActual;

  when {initial(),active} then captured=measuredActual; end when;
  annotation(
    Documentation(info="<html><p><b>Purpose</b> Pressure fluid sensor with local signal faults</p><p><b>Interfaces</b> Modelica.Blocks.Interfaces.RealOutput p(final quantity=\"Pressure\",final unit=\"Pa\",displayUnit=\"bar\",min=0); Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger; output Boolean active \"Requested or latched fault\"; output Real effectLevel(min=0,max=1) \"Current realized fault effect\"; output Integer activationCount \"Number of rising activations\"</p><p><b>Parameters</b> parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter Modelica.Units.SI.Time faultStartTime=1; parameter Modelica.Units.SI.Time faultEndTime=3; parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05; parameter Real biasFault=1; parameter Real gainFault=0.9; parameter Real noiseAmplitude=0.01; parameter Modelica.Units.SI.Frequency noiseFrequency=37; parameter Real stuckValue=0; parameter Real saturationLimit=1e12; parameter Boolean enabled=true \"Enable the selected fault mode\"</p><p><b>Behavior</b> Pressure fluid sensor with local signal faults</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.FluidValveDegradation\">MSLFaults.Examples.FluidValveDegradation</a></p><p><b>Purpose</b> Fault-enhanced Pressure</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> Freeze captures the nominal signal at the activation edge. Subsequent physical evolution is unaffected. An ideal absolute pressure sensor carries zero mass flow.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Fluid.Sensors.Pressure</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.FluidValveDegradation\">MSLFaults.Examples.FluidValveDegradation</a></p><p>用法：将 FaultablePressure 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p></html>"),
    Icon(graphics={
        Ellipse(extent={{-70,-70},{70,70}}, lineColor={255,0,0}, fillColor={255,255,255}, fillPattern=FillPattern.None, pattern=LinePattern.Solid, lineThickness=0.8),
        Text(extent={{-53,-17},{53,-53}}, textString="Pa", textColor={64,64,64}, fontSize=0),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultablePressure;
