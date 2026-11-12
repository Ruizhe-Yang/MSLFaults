within MSLFaults.Thermal.HeatTransfer.Sensors;
model FaultableRelTemperatureSensor "Relative temperature sensor"
  extends Modelica.Icons.RectangularSensor;
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port_a annotation (Placement(transformation(extent={{
            -110,-10},{-90,10}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b port_b annotation (Placement(transformation(extent={{
            90,-10},{110,10}})));
  Modelica.Blocks.Interfaces.RealOutput T_rel(unit="K", displayUnit="K")
    "Relative temperature as output signal"
    annotation (absoluteValue=false, Placement(transformation(
        origin={0,-110},
        extent={{10,-10},{-10,10}},
        rotation=90), iconTransformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={0,-110})));
  type FaultMode=enumeration(Normal "正常", Bias "偏置", GainError "增益误差", NoiseIncrease "噪声增加", Stuck "输出卡死", Dropout "输出丢失", Saturation "输出饱和");
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter Modelica.Units.SI.Time faultStartTime=1;
  parameter Modelica.Units.SI.Time faultEndTime=3;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05;
  Real faultActivation(min=0,max=1);
  Real startActivation(min=0,max=1);
  Real endActivation(min=0,max=1);
  parameter Real biasFault=1;
  parameter Real gainFault=0.9;
  parameter Real noiseAmplitude=0.01;
  parameter Modelica.Units.SI.Frequency noiseFrequency=37;
  parameter Real stuckValue=0;
  parameter Real saturationLimit=1e9;
  Real T_rel_actual;

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

  T_rel_actual=port_a.T-port_b.T;
  T_rel=if faultMode==FaultMode.Bias then T_rel_actual+faultActivation*biasFault elseif faultMode==FaultMode.GainError then T_rel_actual*(1+faultActivation*(gainFault-1)) elseif faultMode==FaultMode.NoiseIncrease then T_rel_actual+faultActivation*noiseAmplitude*sin(2*Modelica.Constants.pi*noiseFrequency*time) elseif faultMode==FaultMode.Stuck then T_rel_actual+faultActivation*(stuckValue-T_rel_actual) elseif faultMode==FaultMode.Dropout then T_rel_actual*(1-faultActivation) elseif faultMode==FaultMode.Saturation then T_rel_actual+faultActivation*(min(saturationLimit,max(-saturationLimit,T_rel_actual))-T_rel_actual) else T_rel_actual;
  0 = port_a.Q_flow;
  0 = port_b.Q_flow;


  annotation (
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
            100,100}}),
      graphics={
        Line(points={{-90,0},{-70,0},{-70,0}}, color={191,0,0}, thickness=0.5),
        Line(points={{-90,0},{-70,0},{-70,0}}, color={191,0,0}, thickness=0.5),
        Line(points={{70,0},{90,0},{90,0}}, color={191,0,0}, thickness=0.5),
        Line(points={{0,-38},{0,-100}}, color={0,0,127}, thickness=0.5),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Text(extent={{-24,20},{66,-40}}, textColor={64,64,64}, textString="K"),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }),
    Documentation(info="<html><p><b>Purpose</b> Relative temperature sensor</p><p><b>Interfaces</b> Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port_a annotation (Placement(transformation(extent={{; Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b port_b annotation (Placement(transformation(extent={{; Modelica.Blocks.Interfaces.RealOutput T_rel(unit=\"K\", displayUnit=\"K\"); Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger; output Boolean active \"Requested or latched fault\"; output Real effectLevel(min=0,max=1) \"Current realized fault effect\"; output Integer activationCount \"Number of rising activations\"</p><p><b>Parameters</b> parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter Modelica.Units.SI.Time faultStartTime=1; parameter Modelica.Units.SI.Time faultEndTime=3; parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05; parameter Real biasFault=1; parameter Real gainFault=0.9; parameter Real noiseAmplitude=0.01; parameter Modelica.Units.SI.Frequency noiseFrequency=37; parameter Real stuckValue=0; parameter Real saturationLimit=1e9; parameter Boolean enabled=true \"Enable the selected fault mode\"</p><p><b>Behavior</b> Relative temperature sensor</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.ThermalDegradation\">MSLFaults.Examples.ThermalDegradation</a></p><p><b>Purpose</b> Fault-enhanced RelTemperatureSensor</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> The established physical equations and fault modes use the common activation and severity law.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Thermal.HeatTransfer.Sensors.RelTemperatureSensor</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.ThermalDegradation\">MSLFaults.Examples.ThermalDegradation</a></p><p>用法：将 FaultableRelTemperatureSensor 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p>
<p>
The relative temperature \"port_a.T - port_b.T\" is determined between
the two ports of this component and is provided as output signal in Kelvin.
</p>
</html>"));
end FaultableRelTemperatureSensor;
