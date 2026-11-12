within MSLFaults.Fluid.Sensors;
model FaultableMassFlowRate "MassFlowRate fluid flow sensor with local signal faults"
  replaceable package Medium=Modelica.Media.Water.StandardWater constrainedby Modelica.Media.Interfaces.PartialMedium;
  Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare package Medium=Medium) annotation(Placement(transformation(extent={{-110,-10},{-90,10}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare package Medium=Medium) annotation(Placement(transformation(extent={{90,-10},{110,10}})));
  Modelica.Blocks.Interfaces.RealOutput m_flow(unit="kg/s") annotation(Placement(transformation(origin={0,110},extent={{10,-10},{-10,10}},rotation=270)));
  type FaultMode=enumeration(Normal "正常", Bias "偏置", GainError "增益误差", NoiseIncrease "噪声增加", Stuck "输出卡死", Dropout "输出丢失", Saturation "输出饱和");
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
  Modelica.Fluid.Sensors.MassFlowRate nominal(redeclare package Medium=Medium);

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
  connect(port_a,nominal.port_a);
  connect(port_b,nominal.port_b);
  measuredActual=nominal.m_flow;
  m_flow=if faultMode==FaultMode.Bias then measuredActual+faultActivation*biasFault elseif faultMode==FaultMode.GainError then measuredActual*(1+faultActivation*(gainFault-1)) elseif faultMode==FaultMode.NoiseIncrease then measuredActual+faultActivation*noiseAmplitude*sin(2*Modelica.Constants.pi*noiseFrequency*time) elseif faultMode==FaultMode.Stuck then measuredActual+faultActivation*(stuckValue-measuredActual) elseif faultMode==FaultMode.Dropout then measuredActual*(1-faultActivation) elseif faultMode==FaultMode.Saturation then measuredActual+faultActivation*(min(saturationLimit,max(-saturationLimit,measuredActual))-measuredActual) else measuredActual;


  annotation(
    Documentation(info="<html><p><b>Purpose</b> MassFlowRate fluid flow sensor with local signal faults</p><p><b>Interfaces</b> Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare package Medium=Medium); Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare package Medium=Medium); Modelica.Blocks.Interfaces.RealOutput m_flow(unit=\"kg/s\"); Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger; output Boolean active \"Requested or latched fault\"; output Real effectLevel(min=0,max=1) \"Current realized fault effect\"; output Integer activationCount \"Number of rising activations\"</p><p><b>Parameters</b> replaceable package Medium=Modelica.Media.Water.StandardWater constrainedby Modelica.Media.Interfaces.PartialMedium; parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter Modelica.Units.SI.Time faultStartTime=1; parameter Modelica.Units.SI.Time faultEndTime=3; parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05; parameter Real biasFault=1; parameter Real gainFault=0.9; parameter Real noiseAmplitude=0.01; parameter Modelica.Units.SI.Frequency noiseFrequency=37; parameter Real stuckValue=0; parameter Real saturationLimit=1e12</p><p><b>Behavior</b> MassFlowRate fluid flow sensor with local signal faults</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.FluidValveDegradation\">MSLFaults.Examples.FluidValveDegradation</a></p><p><b>Purpose</b> Fault-enhanced MassFlowRate</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> The established physical equations and fault modes use the common activation and severity law.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Fluid.Sensors.MassFlowRate</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.FluidValveDegradation\">MSLFaults.Examples.FluidValveDegradation</a></p><p>用法：将 FaultableMassFlowRate 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p></html>"),
    Icon(graphics={
        Ellipse(extent={{-70,-70},{70,70}}, lineColor={255,0,0}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.8),
        Line(points={{-48,23},{-39,18}}, color={64,64,64}, thickness=0.5, pattern=LinePattern.Solid),
        Line(points={{-25,48},{-20,39}}, color={64,64,64}, thickness=0.5, pattern=LinePattern.Solid),
        Line(points={{0,55},{0,44}}, color={64,64,64}, thickness=0.5, pattern=LinePattern.Solid),
        Line(points={{25,48},{20,39}}, color={64,64,64}, thickness=0.5, pattern=LinePattern.Solid),
        Line(points={{48,23},{39,18}}, color={64,64,64}, thickness=0.5, pattern=LinePattern.Solid),
        Line(points={{0,0},{30,33}}, color={64,64,64}, thickness=0.9, pattern=LinePattern.Solid),
        Ellipse(extent={{-4,-4},{4,4}}, lineColor={64,64,64}, fillColor={64,64,64}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.4),
        Text(extent={{-53,-17},{53,-53}}, textString="kg/s", textColor={64,64,64}, fontSize=0),
        Line(points={{-100,0},{-70,0}}, color={0,127,255}, thickness=0.7, pattern=LinePattern.Solid),
        Line(points={{70,0},{100,0}}, color={0,127,255}, thickness=0.7, pattern=LinePattern.Solid),
        Line(points={{0,70},{0,100}}, color={0,0,255}, thickness=0.6, pattern=LinePattern.Solid),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}, fontSize=0),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableMassFlowRate;
