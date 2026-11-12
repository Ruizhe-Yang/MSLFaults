within MSLFaults.Thermal.FluidHeatFlow.Components;
model FaultablePipe "Pipe with optional heat exchange"
  extends Modelica.Thermal.FluidHeatFlow.BaseClasses.TwoPort;
  extends Modelica.Thermal.FluidHeatFlow.BaseClasses.SimpleFriction;

  parameter Boolean useHeatPort = false "= true, if HeatPort is enabled"
    annotation(Evaluate=true, HideResult=true, choices(checkBox=true));
  parameter Modelica.Units.SI.Length h_g(start=0)
    "Geodetic height (height difference from flowPort_a to flowPort_b)";
  parameter Modelica.Units.SI.Acceleration g(final min=0)=Modelica.Constants.g_n "Gravitation";
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatPort(T=T_q, Q_flow=Q_flowHeatPort) if useHeatPort
    annotation (Placement(transformation(extent={{-10,-110},{10,-90}})));
  type FaultMode=enumeration(Normal "正常", PressureLossIncrease "压降增加", Blockage "堵塞", Leakage "泄漏", HeatTransferLoss "换热损失");
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter Modelica.Units.SI.Time faultStartTime=1;
  parameter Modelica.Units.SI.Time faultEndTime=3;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05;
  Real faultActivation(min=0,max=1);
  Real startActivation(min=0,max=1);
  Real endActivation(min=0,max=1);
  parameter Modelica.Units.SI.Pressure pressureDropFault=1e5;
  parameter Real heatTransferScaleFault(min=0)=0.5;
  Modelica.Units.SI.Pressure addedPressureDrop;
  Real heatTransferScale;
protected
  Modelica.Units.SI.HeatFlowRate Q_flowHeatPort "Heat flow at conditional heatPort";

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
  if not useHeatPort then
    Q_flowHeatPort=0;
  end if;
  // coupling with FrictionModel
  volumeFlow = V_flow;
  addedPressureDrop=if faultMode==FaultMode.PressureLossIncrease then faultActivation*pressureDropFault*sign(V_flow) elseif faultMode==FaultMode.Blockage then faultActivation*1e9*V_flow elseif faultMode==FaultMode.Leakage then -faultActivation*0.5*pressureDrop else 0;
  dp=pressureDrop+medium.rho*g*h_g+addedPressureDrop;
  // energy exchange with medium
  heatTransferScale=if faultMode==FaultMode.HeatTransferLoss then 1+faultActivation*(heatTransferScaleFault-1) else 1;
  Q_flow=heatTransferScale*Q_flowHeatPort+Q_friction;


annotation (Documentation(info="<html><p><b>Purpose</b> Pipe with optional heat exchange</p><p><b>Interfaces</b> Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatPort(T=T_q, Q_flow=Q_flowHeatPort) if useHeatPort</p><p><b>Parameters</b> parameter Boolean useHeatPort = false \"= true, if HeatPort is enabled\"; parameter Modelica.Units.SI.Length h_g(start=0); parameter Modelica.Units.SI.Acceleration g(final min=0)=Modelica.Constants.g_n \"Gravitation\"; parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter Modelica.Units.SI.Time faultStartTime=1; parameter Modelica.Units.SI.Time faultEndTime=3; parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05; parameter Modelica.Units.SI.Pressure pressureDropFault=1e5; parameter Real heatTransferScaleFault(min=0)=0.5</p><p><b>Behavior</b> Pipe with optional heat exchange</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.ThermalDegradation\">MSLFaults.Examples.ThermalDegradation</a></p><p><b>Purpose</b> Fault-enhanced Pipe</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> The established physical equations and fault modes use the common activation and severity law.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Thermal.FluidHeatFlow.Components.Pipe</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.ThermalDegradation\">MSLFaults.Examples.ThermalDegradation</a></p><p>用法：将 FaultablePipe 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p>
<p>Pipe with optional heat exchange.</p>
<p>
Thermodynamic equations are defined by BaseClasses.TwoPort.
Q_flow is defined by heatPort.Q_flow (useHeatPort=true) or zero (useHeatPort=false).</p>
<p>
<strong>Note:</strong> Setting parameter m (mass of medium within pipe) to zero
leads to neglect of temperature transient cv*m*der(T).
</p>
<p>
<strong>Note:</strong> Injecting heat into a pipe with zero mass flow causes
temperature rise defined by storing heat in medium's mass.
</p>
</html>"),
  Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
            100}}),
      graphics={
        Rectangle(extent={{-90,20},{90,-20}}, lineColor={255,0,0}, fillColor={0,0,255}, fillPattern=FillPattern.Solid, lineThickness=0.75),
        Polygon(visible=useHeatPort, points={{-10,-90},{-10,-40},{0,-20},{10,-40},{10,-90},{-10,-90}}, lineColor={255,0,0}, lineThickness=0.75),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultablePipe;
