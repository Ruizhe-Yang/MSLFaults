within MSLFaults.Electrical.Analog.Sensors;
model FaultableMultiSensor "Sensor to measure current, voltage and power"
  extends Modelica.Icons.RoundSensor;
  Modelica.Electrical.Analog.Interfaces.PositivePin pc
      "Positive pin, current path"
    annotation (Placement(transformation(extent={{-90,-10},{-110,10}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin nc
      "Negative pin, current path"
    annotation (Placement(transformation(extent={{110,-10},{90,10}})));
  Modelica.Electrical.Analog.Interfaces.PositivePin pv
      "Positive pin, voltage path"
    annotation (Placement(transformation(extent={{-10,110},{10,90}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin nv
      "Negative pin, voltage path"
    annotation (Placement(transformation(extent={{10,-110},{-10,-90}})));
  Modelica.Blocks.Interfaces.RealOutput i(final quantity="ElectricCurrent", final unit="A")
    "Current as output signal" annotation (Placement(transformation(
        origin={-60,-110},
        extent={{10,10},{-10,-10}},
        rotation=90)));
  Modelica.Blocks.Interfaces.RealOutput v(final quantity="ElectricPotential", final unit="V")
    "Voltage as output signal" annotation (Placement(transformation(
        origin={60,-110},
        extent={{10,10},{-10,-10}},
        rotation=90)));
  Modelica.Blocks.Interfaces.RealOutput power(final quantity="Power", final unit="W")
    "Instantaneous power as output signal"
    annotation (Placement(transformation(
          origin={-110,-60},
          extent={{-10,10},{10,-10}},
          rotation=180)));
  type FaultMode = enumeration(Normal "正常", Bias "偏置", GainError "增益误差", NoiseIncrease "噪声增加", Stuck "输出卡死", Dropout "输出丢失", Saturation "输出饱和");
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter Modelica.Units.SI.Time faultStartTime=1;
  parameter Modelica.Units.SI.Time faultEndTime=3;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05;
  parameter Real biasCurrent=0.1;
  parameter Real biasVoltage=0.1;
  parameter Real biasPower=0.1;
  parameter Real gainFault=0.8;
  parameter Real noiseAmplitude=0.01;
  parameter Modelica.Units.SI.Frequency noiseFrequency=37;
  parameter Real stuckCurrent=0;
  parameter Real stuckVoltage=0;
  parameter Real stuckPower=0;
  parameter Real saturationLimit(min=Modelica.Constants.small)=1e6;
  Real i_actual;
  Real v_actual;
  Real power_actual;
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

  pc.i + nc.i = 0;
  pc.v - nc.v = 0;
  pv.i = 0;
  nv.i = 0;
  i_actual = pc.i;
  v_actual = pv.v - nv.v;
  power_actual = v_actual*i_actual;
  i = if faultMode == FaultMode.Bias then i_actual + faultActivation*biasCurrent
    elseif faultMode == FaultMode.GainError then i_actual*(1 + faultActivation*(gainFault - 1))
    elseif faultMode == FaultMode.NoiseIncrease then i_actual + faultActivation*noiseAmplitude*sin(2*Modelica.Constants.pi*noiseFrequency*time)
    elseif faultMode == FaultMode.Stuck then i_actual + faultActivation*(stuckCurrent-i_actual)
    elseif faultMode == FaultMode.Dropout then i_actual*(1-faultActivation)
    elseif faultMode == FaultMode.Saturation then i_actual + faultActivation*(min(saturationLimit,max(-saturationLimit,i_actual))-i_actual) else i_actual;
  v = if faultMode == FaultMode.Bias then v_actual + faultActivation*biasVoltage
    elseif faultMode == FaultMode.GainError then v_actual*(1 + faultActivation*(gainFault - 1))
    elseif faultMode == FaultMode.NoiseIncrease then v_actual + faultActivation*noiseAmplitude*sin(2*Modelica.Constants.pi*noiseFrequency*time)
    elseif faultMode == FaultMode.Stuck then v_actual + faultActivation*(stuckVoltage-v_actual)
    elseif faultMode == FaultMode.Dropout then v_actual*(1-faultActivation)
    elseif faultMode == FaultMode.Saturation then v_actual + faultActivation*(min(saturationLimit,max(-saturationLimit,v_actual))-v_actual) else v_actual;
  power = if faultMode == FaultMode.Bias then power_actual + faultActivation*biasPower
    elseif faultMode == FaultMode.GainError then power_actual*(1 + faultActivation*(gainFault - 1))
    elseif faultMode == FaultMode.NoiseIncrease then power_actual + faultActivation*noiseAmplitude*sin(2*Modelica.Constants.pi*noiseFrequency*time)
    elseif faultMode == FaultMode.Stuck then power_actual + faultActivation*(stuckPower-power_actual)
    elseif faultMode == FaultMode.Dropout then power_actual*(1-faultActivation)
    elseif faultMode == FaultMode.Saturation then power_actual + faultActivation*(min(saturationLimit,max(-saturationLimit,power_actual))-power_actual) else power_actual;


  annotation (
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
      graphics={
        Line(points={{0,100},{0,70}}, color={0,0,255}, thickness=0.5),
        Line(points={{0,-70},{0,-100}}, color={0,0,255}, thickness=0.5),
        Line(points={{-100,0},{100,0}}, color={0,0,255}, thickness=0.5),
        Line(points={{0,70},{0,40}}, color={64,64,64}, thickness=0.5),
        Line(points={{-100,-60},{-80,-60},{-56,-42}}, color={0,0,127}, thickness=0.5),
        Line(points={{-60,-100},{-60,-80},{-42,-56}}, color={0,0,127}, thickness=0.5),
        Line(points={{60,-100},{60,-80},{42,-56}}, color={0,0,127}, thickness=0.5),
        Text(extent={{-100,-40},{-60,-80}}, textColor={64,64,64}, textString="W"),
        Text(extent={{-80,-60},{-40,-100}}, textColor={64,64,64}, textString="A"),
        Text(extent={{40,-60},{80,-100}}, textColor={64,64,64}, textString="V"),
        Text(textColor={0,0,255}, extent={{-150,120},{150,160}}, textString="%name"),
        Ellipse(extent={{-70,70},{70,-70}}, lineColor={255,0,0}, lineThickness=0.75),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }),
    Documentation(info="<html><p><b>Purpose</b> Sensor to measure current, voltage and power</p><p><b>Interfaces</b> Modelica.Electrical.Analog.Interfaces.PositivePin pc; Modelica.Electrical.Analog.Interfaces.NegativePin nc; Modelica.Electrical.Analog.Interfaces.PositivePin pv; Modelica.Electrical.Analog.Interfaces.NegativePin nv; Modelica.Blocks.Interfaces.RealOutput i(final quantity=\"ElectricCurrent\", final unit=\"A\"); Modelica.Blocks.Interfaces.RealOutput v(final quantity=\"ElectricPotential\", final unit=\"V\"); Modelica.Blocks.Interfaces.RealOutput power(final quantity=\"Power\", final unit=\"W\"); Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger; output Boolean active \"Requested or latched fault\"; output Real effectLevel(min=0,max=1) \"Current realized fault effect\"; output Integer activationCount \"Number of rising activations\"</p><p><b>Parameters</b> parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter Modelica.Units.SI.Time faultStartTime=1; parameter Modelica.Units.SI.Time faultEndTime=3; parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05; parameter Real biasCurrent=0.1; parameter Real biasVoltage=0.1; parameter Real biasPower=0.1; parameter Real gainFault=0.8; parameter Real noiseAmplitude=0.01; parameter Modelica.Units.SI.Frequency noiseFrequency=37; parameter Real stuckCurrent=0</p><p><b>Behavior</b> Sensor to measure current, voltage and power</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.SensorAndSignalFaults\">MSLFaults.Examples.SensorAndSignalFaults</a></p><p><b>Purpose</b> Fault-enhanced MultiSensor</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> The established physical equations and fault modes use the common activation and severity law.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Electrical.Analog.Sensors.MultiSensor</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.SensorAndSignalFaults\">MSLFaults.Examples.SensorAndSignalFaults</a></p><p>用法：将 FaultableMultiSensor 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p>
<p>This multi sensor measures current, voltage and instantaneous electrical power of a single-phase system and has a separated voltage and current path.
The pins of the voltage path are pv and nv, the pins of the current path are pc and nc.
The internal resistance of the current path is zero, the internal resistance of the voltage path is infinite.</p>
</html>", revisions="<html>
<ul>
<li><em>20170306</em> first implementation by Anton Haumer</li>
</ul>
</html>"));
end FaultableMultiSensor;
