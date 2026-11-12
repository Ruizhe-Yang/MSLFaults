within MSLFaults.Fluid.Pipes;
model FaultableDynamicPipe "DynamicPipe with an inline transparent fault pressure element"
  replaceable package Medium=Modelica.Media.Water.StandardWater constrainedby Modelica.Media.Interfaces.PartialMedium "工作介质";
  Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare package Medium=Medium)
    annotation(Placement(transformation(extent={{-110,-10},{-90,10}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare package Medium=Medium)
    annotation(Placement(transformation(extent={{90,-10},{110,10}})));
  parameter Modelica.Units.SI.Length length=1 "Length";
  parameter Modelica.Units.SI.Diameter diameter=0.1 "Diameter";
  parameter Modelica.Units.SI.Height height_ab=0 "Height difference port_b - port_a";
  parameter Modelica.Units.SI.Length roughness=2.5e-5 "Average roughness";
  parameter Integer nParallel(min=1)=1 "Number of identical parallel pipes";
  parameter Boolean allowFlowReversal=true;
  parameter Integer nNodes(min=1)=2 "Number of discrete flow volumes";
  parameter Boolean use_HeatTransfer=false;
  Modelica.Fluid.Interfaces.HeatPorts_a[nNodes] heatPorts if use_HeatTransfer
    annotation(Placement(transformation(extent={{-30,36},{32,52}})));
  type FaultMode=enumeration(Normal "正常", RoughnessIncrease "粗糙度增加", Fouling "结垢", PartialBlockage "局部堵塞", CompleteBlockage "完全堵塞");
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter Modelica.Units.SI.Time faultStartTime=1;
  parameter Modelica.Units.SI.Time faultEndTime=3;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05;
  Real faultActivation(min=0,max=1);
  Real startActivation(min=0,max=1);
  Real endActivation(min=0,max=1);
  parameter Real blockageCoefficient(unit="Pa.s2/kg2")=1e8;
  parameter Real completeBlockageCoefficient(unit="Pa.s2/kg2")=1e14;
  Real K_fault(unit="Pa.s2/kg2");
  model InlineFaultPressureLoss "Local, topology-invariant fault pressure loss"
    replaceable package Medium=Modelica.Media.Interfaces.PartialMedium;
    Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare package Medium=Medium);
    Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare package Medium=Medium);
    Modelica.Blocks.Interfaces.RealInput K(unit="Pa.s2/kg2");
  equation
    port_a.m_flow+port_b.m_flow=0;
    port_a.p-port_b.p=K*port_a.m_flow*abs(port_a.m_flow);
    port_a.h_outflow=inStream(port_b.h_outflow);
    port_b.h_outflow=inStream(port_a.h_outflow);
    port_a.Xi_outflow=inStream(port_b.Xi_outflow);
    port_b.Xi_outflow=inStream(port_a.Xi_outflow);
    port_a.C_outflow=inStream(port_b.C_outflow);
    port_b.C_outflow=inStream(port_a.C_outflow);
  end InlineFaultPressureLoss;
  InlineFaultPressureLoss faultLoss(redeclare package Medium=Medium);
  Modelica.Fluid.Pipes.DynamicPipe nominal(
    redeclare package Medium=Medium,
    length=length,diameter=diameter,height_ab=height_ab,roughness=roughness,
    nNodes=nNodes,use_HeatTransfer=use_HeatTransfer,allowFlowReversal=allowFlowReversal);

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
  K_fault=if faultMode==FaultMode.RoughnessIncrease then faultActivation*0.1*blockageCoefficient
    elseif faultMode==FaultMode.Fouling then faultActivation*blockageCoefficient
    elseif faultMode==FaultMode.PartialBlockage then 10*faultActivation*blockageCoefficient
    elseif faultMode==FaultMode.CompleteBlockage then faultActivation*completeBlockageCoefficient else 0;
  faultLoss.K=K_fault;
  connect(port_a,faultLoss.port_a);
  connect(faultLoss.port_b,nominal.port_a);
  connect(nominal.port_b,port_b);
  connect(heatPorts,nominal.heatPorts);


  annotation(Icon(coordinateSystem(preserveAspectRatio=true,extent={{-100,-100},{100,100}}),
      graphics={
        Rectangle(extent={{-100,44},{100,-44}}, lineColor={255,0,0}, fillColor={0,127,255}, fillPattern=FillPattern.HorizontalCylinder, lineThickness=0.75),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Line(points={{-34,-40},{-34,40}}, color={255,255,255}, thickness=0.65, pattern=LinePattern.Dash),
        Line(points={{34,-40},{34,40}}, color={255,255,255}, thickness=0.65, pattern=LinePattern.Dash),
        Text(extent={{-24,-55},{24,-80}}, textString="n", textColor={64,64,64}, fontSize=0),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }),Documentation(info="<html><p><b>Purpose</b> DynamicPipe with an inline transparent fault pressure element</p><p><b>Interfaces</b> Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare package Medium=Medium); Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare package Medium=Medium); Modelica.Fluid.Interfaces.HeatPorts_a[nNodes] heatPorts if use_HeatTransfer; Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare package Medium=Medium); Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare package Medium=Medium); Modelica.Blocks.Interfaces.RealInput K(unit=\"Pa.s2/kg2\")</p><p><b>Parameters</b> replaceable package Medium=Modelica.Media.Water.StandardWater constrainedby Modelica.Media.Interfaces.PartialMedium \"工作介质\"; parameter Modelica.Units.SI.Length length=1 \"Length\"; parameter Modelica.Units.SI.Diameter diameter=0.1 \"Diameter\"; parameter Modelica.Units.SI.Height height_ab=0 \"Height difference port_b - port_a\"; parameter Modelica.Units.SI.Length roughness=2.5e-5 \"Average roughness\"; parameter Integer nParallel(min=1)=1 \"Number of identical parallel pipes\"; parameter Boolean allowFlowReversal=true; parameter Integer nNodes(min=1)=2 \"Number of discrete flow volumes\"; parameter Boolean use_HeatTransfer=false; parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter Modelica.Units.SI.Time faultStartTime=1</p><p><b>Behavior</b> DynamicPipe with an inline transparent fault pressure element</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.FluidValveDegradation\">MSLFaults.Examples.FluidValveDegradation</a></p><p><b>Purpose</b> Fault-enhanced DynamicPipe</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> The established physical equations and fault modes use the common activation and severity law.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Fluid.Pipes.DynamicPipe</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.FluidValveDegradation\">MSLFaults.Examples.FluidValveDegradation</a></p><p>用法：将 FaultableDynamicPipe 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p><p>本模型保留 Modelica.Fluid 的 replaceable Medium、FluidPort 和 stream 连接语义。名义 MSL 组件作为内部正常物理模型；故障支路、控制量变换及激活方程全部在本文件可见，Normal 或 severity=0 时透明。</p></html>"));
end FaultableDynamicPipe;
