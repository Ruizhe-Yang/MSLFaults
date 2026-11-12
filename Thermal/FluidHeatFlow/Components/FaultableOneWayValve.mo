within MSLFaults.Thermal.FluidHeatFlow.Components;
model FaultableOneWayValve "Simple one-way valve"
  extends Modelica.Thermal.FluidHeatFlow.BaseClasses.TwoPort(m(start=0), final tapT=1);

  parameter Modelica.Units.SI.VolumeFlowRate V_flowNominal(start=1) "Nominal volume flow rate (forward)";
  parameter Modelica.Units.SI.Pressure dpForward(displayUnit="bar")=1e-6 "Pressure drop at nominal flow (forward)";
  parameter Modelica.Units.SI.Pressure dpNominal(displayUnit="bar", start=1e5) "Nominal pressure (backward)";
  parameter Modelica.Units.SI.VolumeFlowRate V_flowBackward(start=1E-6) "Leakage volume flow rate (backward)";
  parameter Real frictionLoss(min=0, max=1, start=0)
    "Part of friction losses fed to medium";
  Boolean backward(start=true) "State forward=false / backward=true";
protected
  Real s(start=0, final unit="1")
    "Auxiliary variable for actual position on the valve characteristic";
  /* s < 0: backward, leakage flow
         s > 0: forward, small pressure drop */
  constant Modelica.Units.SI.VolumeFlowRate unitVolumeFlowRate = 1;
public
  constant Modelica.Units.SI.Pressure unitPressureDrop = 1;  type FaultMode=enumeration(Normal "正常", StuckClosed "单向阀卡闭", StuckOpen "单向阀卡开", ForwardResistanceIncrease "正向阻力增加", ReverseLeakageIncrease "反向泄漏增加");
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter Modelica.Units.SI.Time faultStartTime=1;
  parameter Modelica.Units.SI.Time faultEndTime=3;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05;
  Real faultActivation(min=0,max=1);
  Real startActivation(min=0,max=1);
  Real endActivation(min=0,max=1);
  parameter Modelica.Units.SI.Pressure dpForwardFault=10*dpForward;
  parameter Modelica.Units.SI.VolumeFlowRate reverseLeakFault=100*V_flowBackward;
  Modelica.Units.SI.Pressure dpForward_effective;
  Modelica.Units.SI.VolumeFlowRate V_flowBackward_effective;
  Boolean backward_effective;

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
  backward_effective=if faultMode==FaultMode.StuckClosed and faultActivation>0.5 then true elseif faultMode==FaultMode.StuckOpen and faultActivation>0.5 then false else s<0;
  backward=backward_effective;
  dpForward_effective=if faultMode==FaultMode.ForwardResistanceIncrease or faultMode==FaultMode.StuckClosed then dpForward+faultActivation*(dpForwardFault-dpForward) else dpForward;
  V_flowBackward_effective=if faultMode==FaultMode.ReverseLeakageIncrease or faultMode==FaultMode.StuckOpen then V_flowBackward+faultActivation*(reverseLeakFault-V_flowBackward) else V_flowBackward;
  dp=(s*unitVolumeFlowRate)*(if backward_effective then 1 else dpForward_effective/V_flowNominal);
  V_flow=(s*unitPressureDrop)*(if backward_effective then V_flowBackward_effective/dpNominal else 1);
  Q_flow = frictionLoss*V_flow*dp;


annotation (Documentation(info="<html><p><b>Purpose</b> Simple one-way valve</p><p><b>Interfaces</b> See the named arguments, record fields, or inherited connectors.</p><p><b>Parameters</b> parameter Modelica.Units.SI.VolumeFlowRate V_flowNominal(start=1) \"Nominal volume flow rate (forward)\"; parameter Modelica.Units.SI.Pressure dpForward(displayUnit=\"bar\")=1e-6 \"Pressure drop at nominal flow (forward)\"; parameter Modelica.Units.SI.Pressure dpNominal(displayUnit=\"bar\", start=1e5) \"Nominal pressure (backward)\"; parameter Modelica.Units.SI.VolumeFlowRate V_flowBackward(start=1E-6) \"Leakage volume flow rate (backward)\"; parameter Real frictionLoss(min=0, max=1, start=0)</p><p><b>Behavior</b> Simple one-way valve</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.ThermalDegradation\">MSLFaults.Examples.ThermalDegradation</a></p><p><b>Purpose</b> Fault-enhanced OneWayValve</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> The established physical equations and fault modes use the common activation and severity law.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Thermal.FluidHeatFlow.Components.OneWayValve</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.ThermalDegradation\">MSLFaults.Examples.ThermalDegradation</a></p><p>用法：将 FaultableOneWayValve 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p>
<p>Simple one-way valve, comparable to the electrical <a href=\"modelica://Modelica.Electrical.Analog.Ideal.IdealDiode\">ideal diode</a> model.</p>
<ul>
<li>from flowPort_a to flowPort_b: small pressure drop, linearly dependent on volumeFlow</li>
<li>from flowPort_b to flowPort_a: small leakage flow, linearly dependent on pressure drop</li>
</ul>
</html>"),
  Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
            100}}),
      graphics={
        Polygon(points={{-90,10},{-60,10},{-60,60},{0,0},{60,60},{60,10},{90,10},{90,-10},{60,-10},{60,-60},{0,0},{-60,-60},{-60,-10},{-90,-10},{-90,10}}, lineColor={255,0,0}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, lineThickness=0.75),
        Text(extent={{-150,-70},{150,-110}}, textString="%name", textColor={0,0,255}),
        Line(points={{-60,60},{60,-60},{50,-40},{40,-50},{60,-60}}, thickness=0.5, color={64,64,64}),
        Polygon(points={{50,-40},{60,-60},{40,-50},{50,-40}}, fillPattern=FillPattern.Solid, lineColor={64,64,64}, lineThickness=0.5),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableOneWayValve;
