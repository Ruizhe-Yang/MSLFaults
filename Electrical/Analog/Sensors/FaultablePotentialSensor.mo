within MSLFaults.Electrical.Analog.Sensors;
model FaultablePotentialSensor "Sensor to measure the potential"
  extends Modelica.Icons.RoundSensor;

  Modelica.Electrical.Analog.Interfaces.PositivePin p "Pin to be measured" annotation (Placement(
        transformation(extent={{-110,-10},{-90,10}})));
  Modelica.Blocks.Interfaces.RealOutput phi(unit="V")
    "Absolute voltage potential as output signal"
      annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  type FaultMode = enumeration(Normal "正常", Bias "偏置", GainError "增益误差", NoiseIncrease "噪声增加", Stuck "输出卡死", Dropout "输出丢失", Saturation "输出饱和");
  parameter FaultMode faultMode=FaultMode.Normal "故障模式";
  parameter Real severity(min=0,max=1)=1 "故障严重度";
  parameter Modelica.Units.SI.Time faultStartTime=1;
  parameter Modelica.Units.SI.Time faultEndTime=3;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05;
  parameter Real biasFault=1 "偏置故障目标";
  parameter Real gainFault=0.8 "增益故障目标";
  parameter Real noiseAmplitude=0.01 "确定性噪声幅值";
  parameter Modelica.Units.SI.Frequency noiseFrequency=37 "确定性噪声频率";
  parameter Real stuckValue=0 "卡死输出值";
  parameter Real saturationLimit(min=Modelica.Constants.small)=1e6 "对称饱和限值";
  Real phi_actual "未受信号故障影响的真实测量值";
  Real faultActivation(min=0,max=1);
  Real startActivation(min=0,max=1);
  Real endActivation(min=0,max=1);

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

  p.i = 0;
  phi_actual = p.v;
  phi = if faultMode == FaultMode.Bias then phi_actual + faultActivation*biasFault
    elseif faultMode == FaultMode.GainError then phi_actual*(1 + faultActivation*(gainFault - 1))
    elseif faultMode == FaultMode.NoiseIncrease then phi_actual + faultActivation*noiseAmplitude*sin(2*Modelica.Constants.pi*noiseFrequency*time)
    elseif faultMode == FaultMode.Stuck then phi_actual + faultActivation*(stuckValue - phi_actual)
    elseif faultMode == FaultMode.Dropout then phi_actual*(1 - faultActivation)
    elseif faultMode == FaultMode.Saturation then phi_actual + faultActivation*(min(saturationLimit,max(-saturationLimit,phi_actual)) - phi_actual)
    else phi_actual;


  annotation (
    Icon(coordinateSystem(
        preserveAspectRatio=true,
        extent={{-100,-100},{100,100}}),
      graphics={
        Line(points={{-70,0},{-90,0}}, color={0,0,255}, thickness=0.5),
        Line(points={{100,0},{70,0}}, color={0,0,127}, thickness=0.5),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Text(extent={{-30,-10},{30,-70}}, textString="V", textColor={64,64,64}),
        Ellipse(extent={{-70,70},{70,-70}}, lineColor={255,0,0}, lineThickness=0.75),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }),
    Documentation(revisions="<html>
<ul>
<li><em> 1998   </em>
       by Christoph Clauss<br> initially implemented<br>
       </li>
</ul>
</html>",
        info="<html><p>用法：将 FaultablePotentialSensor 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p>
<p>The potential sensor converts the voltage of a node (with respect to the ground node) into a real valued signal. It does not influence the current sum at the node which voltage is measured, therefore, the electrical behavior is not influenced by the sensor.</p>
</html>"));
  annotation(Documentation(info="<html><p><b>Purpose</b> Sensor to measure the potential</p><p><b>Interfaces</b> Modelica.Electrical.Analog.Interfaces.PositivePin p \"Pin to be measured\" annotation (Placement(; Modelica.Blocks.Interfaces.RealOutput phi(unit=\"V\"); Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger; output Boolean active \"Requested or latched fault\"; output Real effectLevel(min=0,max=1) \"Current realized fault effect\"; output Integer activationCount \"Number of rising activations\"</p><p><b>Parameters</b> parameter FaultMode faultMode=FaultMode.Normal \"故障模式\"; parameter Real severity(min=0,max=1)=1 \"故障严重度\"; parameter Modelica.Units.SI.Time faultStartTime=1; parameter Modelica.Units.SI.Time faultEndTime=3; parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05; parameter Real biasFault=1 \"偏置故障目标\"; parameter Real gainFault=0.8 \"增益故障目标\"; parameter Real noiseAmplitude=0.01 \"确定性噪声幅值\"; parameter Modelica.Units.SI.Frequency noiseFrequency=37 \"确定性噪声频率\"; parameter Real stuckValue=0 \"卡死输出值\"; parameter Real saturationLimit(min=Modelica.Constants.small)=1e6 \"对称饱和限值\"; parameter Boolean enabled=true \"Enable the selected fault mode\"</p><p><b>Behavior</b> Sensor to measure the potential</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.SensorAndSignalFaults\">MSLFaults.Examples.SensorAndSignalFaults</a></p><p><b>Purpose</b> Fault-enhanced PotentialSensor</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> The established physical equations and fault modes use the common activation and severity law.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Electrical.Analog.Sensors.PotentialSensor</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.SensorAndSignalFaults\">MSLFaults.Examples.SensorAndSignalFaults</a></p></html>"));
end FaultablePotentialSensor;
