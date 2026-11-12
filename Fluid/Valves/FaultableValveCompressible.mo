within MSLFaults.Fluid.Valves;
model FaultableValveCompressible "ValveCompressible with inline opening faults"
  replaceable package Medium=Modelica.Media.Air.SimpleAir constrainedby Modelica.Media.Interfaces.PartialMedium "工作介质";
  Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare package Medium=Medium)
    annotation(Placement(transformation(extent={{-110,-10},{-90,10}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare package Medium=Medium)
    annotation(Placement(transformation(extent={{90,-10},{110,10}})));
  parameter Modelica.Units.SI.Pressure dp_nominal=1e5 "Nominal pressure drop";
  parameter Medium.MassFlowRate m_flow_nominal=1 "Nominal mass flow rate";
  parameter Real opening_nominal(min=0,max=1)=1 "Nominal opening";
  parameter Real opening_min(min=0,max=1)=0 "Remaining opening at closed command";
  parameter Real Fxt_full=0.5 "Nominal choked-flow factor";
  type FaultMode=enumeration(Normal "正常", StuckClosed "阀门卡闭", StuckOpen "阀门卡开", OpeningBias "开度偏置", ActuatorGainLoss "执行器增益下降", LeakageIncrease "泄漏增加");
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter Modelica.Units.SI.Time faultStartTime=1;
  parameter Modelica.Units.SI.Time faultEndTime=3;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05;
  Real faultActivation(min=0,max=1);
  Real startActivation(min=0,max=1);
  Real endActivation(min=0,max=1);
  parameter Real stuckOpening(min=0,max=1)=0;
  parameter Real openingBias=0.1;
  parameter Real actuatorGainFault(min=0,max=1)=0.5;
  Modelica.Blocks.Interfaces.RealInput opening annotation(Placement(transformation(origin={0,100},extent={{-20,-20},{20,20}},rotation=270)));
  Real opening_effective(min=0,max=1);
  Modelica.Fluid.Valves.ValveCompressible nominal(
    redeclare package Medium=Medium,
    dp_nominal=dp_nominal,m_flow_nominal=m_flow_nominal,opening_nominal=opening_nominal,Fxt_full=Fxt_full);

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
  opening_effective=max(opening_min,min(1,if faultMode==FaultMode.StuckClosed then opening+faultActivation*(stuckOpening-opening)
    elseif faultMode==FaultMode.StuckOpen then opening+faultActivation*(1-opening)
    elseif faultMode==FaultMode.OpeningBias then opening+faultActivation*openingBias
    elseif faultMode==FaultMode.ActuatorGainLoss then opening*(1+faultActivation*(actuatorGainFault-1))
    elseif faultMode==FaultMode.LeakageIncrease then max(opening,faultActivation*0.2) else opening));
  nominal.opening=opening_effective;
  connect(port_a,nominal.port_a);
  connect(port_b,nominal.port_b);


  annotation(Icon(coordinateSystem(preserveAspectRatio=true,extent={{-100,-100},{100,100}}),
      graphics={
        Polygon(points={{-100,50},{100,-50},{100,50},{0,0},{-100,-50},{-100,50}}, lineColor={255,0,0}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, lineThickness=0.75),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Line(points={{-48,-84},{-29,-73},{-9,-66},{15,-62},{48,-61}}, color={64,64,64}, thickness=0.75, pattern=LinePattern.Solid),
        Text(extent={{-33,-97},{33,-116}}, textString="gas", textColor={64,64,64}, fontSize=0),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }),Documentation(info="<html><p><b>Purpose</b> ValveCompressible with inline opening faults</p><p><b>Interfaces</b> Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare package Medium=Medium); Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare package Medium=Medium); Modelica.Blocks.Interfaces.RealInput opening; Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger; output Boolean active \"Requested or latched fault\"; output Real effectLevel(min=0,max=1) \"Current realized fault effect\"; output Integer activationCount \"Number of rising activations\"</p><p><b>Parameters</b> replaceable package Medium=Modelica.Media.Air.SimpleAir constrainedby Modelica.Media.Interfaces.PartialMedium \"工作介质\"; parameter Modelica.Units.SI.Pressure dp_nominal=1e5 \"Nominal pressure drop\"; parameter Medium.MassFlowRate m_flow_nominal=1 \"Nominal mass flow rate\"; parameter Real opening_nominal(min=0,max=1)=1 \"Nominal opening\"; parameter Real opening_min(min=0,max=1)=0 \"Remaining opening at closed command\"; parameter Real Fxt_full=0.5 \"Nominal choked-flow factor\"; parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter Modelica.Units.SI.Time faultStartTime=1; parameter Modelica.Units.SI.Time faultEndTime=3; parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05; parameter Real stuckOpening(min=0,max=1)=0</p><p><b>Behavior</b> ValveCompressible with inline opening faults</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.FluidValveDegradation\">MSLFaults.Examples.FluidValveDegradation</a></p><p><b>Purpose</b> Fault-enhanced ValveCompressible</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> The established physical equations and fault modes use the common activation and severity law.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Fluid.Valves.ValveCompressible</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.FluidValveDegradation\">MSLFaults.Examples.FluidValveDegradation</a></p><p>用法：将 FaultableValveCompressible 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p><p>本模型保留 Modelica.Fluid 的 replaceable Medium、FluidPort 和 stream 连接语义。名义 MSL 组件作为内部正常物理模型；故障支路、控制量变换及激活方程全部在本文件可见，Normal 或 severity=0 时透明。</p></html>"));
end FaultableValveCompressible;
