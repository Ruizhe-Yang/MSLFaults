within MSLFaults.Fluid.Machines;
model FaultableControlledPump "ControlledPump with set-point and performance faults"
  replaceable package Medium=Modelica.Media.Water.StandardWater constrainedby Modelica.Media.Interfaces.PartialMedium "工作介质";
  Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare package Medium=Medium)
    annotation(Placement(transformation(extent={{-110,-10},{-90,10}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare package Medium=Medium)
    annotation(Placement(transformation(extent={{90,-10},{110,10}})));
  parameter Medium.AbsolutePressure p_a_nominal=1e5;
  parameter Medium.AbsolutePressure p_b_nominal=2e5;
  parameter Medium.MassFlowRate m_flow_nominal=1;
  parameter Boolean control_m_flow=true;
  parameter Boolean use_m_flow_set=false;
  parameter Boolean use_p_set=false;
  Modelica.Blocks.Interfaces.RealInput m_flow_set(unit="kg/s") if use_m_flow_set annotation(Placement(transformation(extent={{-70,70},{-30,110}})));
  Modelica.Blocks.Interfaces.RealInput p_set(unit="Pa") if use_p_set annotation(Placement(transformation(extent={{30,70},{70,110}})));
  type FaultMode=enumeration(Normal "正常", OutputLoss "泵输出丢失", SetpointBias "设定值偏置", GainLoss "控制增益下降", StuckCommand "控制指令卡死");
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter Modelica.Units.SI.Time faultStartTime=1;
  parameter Modelica.Units.SI.Time faultEndTime=3;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05;
  Real faultActivation(min=0,max=1);
  Real startActivation(min=0,max=1);
  Real endActivation(min=0,max=1);
  parameter Real gainFault=0.5;
  parameter Medium.MassFlowRate massFlowBias=0;
  parameter Modelica.Units.SI.PressureDifference pressureBias=0;
  parameter Real stuckMassFlow=0;
  parameter Modelica.Units.SI.Pressure stuckPressure=p_b_nominal;
  Real m_flow_command;
  Real p_command;
  Modelica.Fluid.Machines.ControlledPump nominal(redeclare package Medium=Medium,p_a_nominal=p_a_nominal,p_b_nominal=p_b_nominal,m_flow_nominal=m_flow_nominal,control_m_flow=control_m_flow,use_m_flow_set=true,use_p_set=true);
protected
  Modelica.Blocks.Interfaces.RealInput m_flow_set_internal(unit="kg/s");
  Modelica.Blocks.Interfaces.RealInput p_set_internal(unit="Pa");

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
  connect(m_flow_set,m_flow_set_internal);
  connect(p_set,p_set_internal);
  if not use_m_flow_set then
    m_flow_set_internal=m_flow_nominal;
  end if;
  if not use_p_set then
    p_set_internal=p_b_nominal;
  end if;
  m_flow_command=m_flow_set_internal;
  p_command=p_set_internal;
  nominal.m_flow_set=if faultMode==FaultMode.OutputLoss then m_flow_command*(1-faultActivation) elseif faultMode==FaultMode.SetpointBias then m_flow_command+faultActivation*massFlowBias elseif faultMode==FaultMode.GainLoss then m_flow_command*(1+faultActivation*(gainFault-1)) elseif faultMode==FaultMode.StuckCommand then m_flow_command+faultActivation*(stuckMassFlow-m_flow_command) else m_flow_command;
  nominal.p_set=if faultMode==FaultMode.OutputLoss then port_a.p+(p_command-port_a.p)*(1-faultActivation) elseif faultMode==FaultMode.SetpointBias then p_command+faultActivation*pressureBias elseif faultMode==FaultMode.GainLoss then port_a.p+(p_command-port_a.p)*(1+faultActivation*(gainFault-1)) elseif faultMode==FaultMode.StuckCommand then p_command+faultActivation*(stuckPressure-p_command) else p_command;
  connect(port_a,nominal.port_a);connect(port_b,nominal.port_b);


  annotation(Icon(coordinateSystem(preserveAspectRatio=true,extent={{-100,-100},{100,100}}),
      graphics={
        Ellipse(extent={{-70,70},{70,-70}}, lineColor={255,0,0}, fillColor={0,127,255}, fillPattern=FillPattern.Solid, lineThickness=0.75),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Polygon(points={{-38,-40},{-38,40},{49,0},{-38,-40}}, lineColor={255,255,255}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.5),
        Line(points={{-100,0},{-70,0}}, color={0,127,255}, thickness=0.75, pattern=LinePattern.Solid),
        Line(points={{70,0},{100,0}}, color={0,127,255}, thickness=0.75, pattern=LinePattern.Solid),
        Text(extent={{-38,-79},{38,-104}}, textString="set", textColor={64,64,64}, fontSize=0),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }),Documentation(info="<html><p><b>Purpose</b> ControlledPump with set-point and performance faults</p><p><b>Interfaces</b> Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare package Medium=Medium); Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare package Medium=Medium); Modelica.Blocks.Interfaces.RealInput m_flow_set(unit=\"kg/s\") if use_m_flow_set; Modelica.Blocks.Interfaces.RealInput p_set(unit=\"Pa\") if use_p_set</p><p><b>Parameters</b> replaceable package Medium=Modelica.Media.Water.StandardWater constrainedby Modelica.Media.Interfaces.PartialMedium \"工作介质\"; parameter Medium.AbsolutePressure p_a_nominal=1e5; parameter Medium.AbsolutePressure p_b_nominal=2e5; parameter Medium.MassFlowRate m_flow_nominal=1; parameter Boolean control_m_flow=true; parameter Boolean use_m_flow_set=false; parameter Boolean use_p_set=false; parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter Modelica.Units.SI.Time faultStartTime=1; parameter Modelica.Units.SI.Time faultEndTime=3; parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05</p><p><b>Behavior</b> ControlledPump with set-point and performance faults</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.FluidValveDegradation\">MSLFaults.Examples.FluidValveDegradation</a></p><p><b>Purpose</b> Fault-enhanced ControlledPump</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> The established physical equations and fault modes use the common activation and severity law.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Fluid.Machines.ControlledPump</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.FluidValveDegradation\">MSLFaults.Examples.FluidValveDegradation</a></p><p>用法：将 FaultableControlledPump 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p><p>本模型保留 Modelica.Fluid 的 replaceable Medium、FluidPort 和 stream 连接语义。名义 MSL 组件作为内部正常物理模型；故障支路、控制量变换及激活方程全部在本文件可见，Normal 或 severity=0 时透明。</p></html>"));
end FaultableControlledPump;
