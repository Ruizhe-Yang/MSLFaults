within MSLFaults.Thermal.FluidHeatFlow.Components;
model FaultableValve "Simple valve"
  extends Modelica.Thermal.FluidHeatFlow.BaseClasses.TwoPort(m(start=0), final tapT=1);

  parameter Boolean LinearCharacteristic(start=true)
    "Type of characteristic"
    annotation(Dialog(group="Standard characteristic"), choices(choice=true "Linear", choice=false
        "Exponential"));
  parameter Real y1(min=small, start=1) "Max. valve opening"
    annotation(Dialog(group="Standard characteristic"));
  parameter Modelica.Units.SI.VolumeFlowRate Kv1(min=small, start=1)
    "Max. flow @ y = y1"
    annotation(Dialog(group="Standard characteristic"));
  parameter Real kv0(min=small,max=1-small, start=0.01)
    "Leakage flow / max.flow @ y = 0"
    annotation(Dialog(group="Standard characteristic"));
  parameter Modelica.Units.SI.Pressure dp0(start=1) "Standard pressure drop"
    annotation(Dialog(group="Standard characteristic"));
  parameter Modelica.Units.SI.Density rho0(start=10)
    "Standard medium's density"
    annotation(Dialog(group="Standard characteristic"));
  parameter Real frictionLoss(min=0, max=1, start=0)
    "Part of friction losses fed to medium";
  type FaultMode=enumeration(Normal "正常", StuckClosed "阀门卡闭", StuckOpen "阀门卡开", OpeningBias "开度偏置", LeakageIncrease "泄漏增加", Blockage "堵塞");
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter Modelica.Units.SI.Time faultStartTime=1;
  parameter Modelica.Units.SI.Time faultEndTime=3;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05;
  Real faultActivation(min=0,max=1);
  Real startActivation(min=0,max=1);
  Real endActivation(min=0,max=1);
  parameter Real stuckOpening(min=0)=0;
  parameter Real openingBias=0.1;
  parameter Real leakageFault(min=Modelica.Constants.small,max=1)=0.1;
  Real y_effective;
  Real kv0_effective;
protected
  constant Modelica.Units.SI.VolumeFlowRate unitVolumeFlowRate = 1;
  constant Real small = Modelica.Constants.small;
  constant Modelica.Units.SI.VolumeFlowRate smallVolumeFlowRate = eps*unitVolumeFlowRate;
  constant Real eps = Modelica.Constants.eps;
  Real yLim "Limited fault-affected valve opening";
  Modelica.Units.SI.VolumeFlowRate Kv "Standard flow rate";
public
  Modelica.Blocks.Interfaces.RealInput y annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={0,100})));

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
initial algorithm
  assert(y1>small, "Valve characteristic: y1 has to be > 0 !");
  assert(Kv1>smallVolumeFlowRate, "Valve characteristic: Kv1 has to be > 0 !");
  assert(kv0>small, "Valve characteristic: kv0 has to be > 0 !");
  assert(kv0<1-eps, "Valve characteristic: kv0 has to be < 1 !");
equation
  connect(trigger,faultGate.externalTrigger);
  if not useExternalTrigger then faultGate.externalTrigger=false; end if;
  active=faultGate.active;
  effectLevel=faultGate.effectLevel;
  activationCount=faultGate.activationCount;

  startActivation=if severity>0 then effectLevel/severity else 0;
  endActivation=if active or effectLevel>0 then 1 else 0;
  faultActivation=effectLevel;
  y_effective=if faultMode==FaultMode.StuckClosed then y+faultActivation*(stuckOpening-y) elseif faultMode==FaultMode.StuckOpen then y+faultActivation*(y1-y) elseif faultMode==FaultMode.OpeningBias then y+faultActivation*openingBias elseif faultMode==FaultMode.Blockage then y*(1-faultActivation) else y;
  kv0_effective=if faultMode==FaultMode.LeakageIncrease then kv0+faultActivation*(leakageFault-kv0) else kv0;
  yLim=max(min(y_effective,y1),0);
  // evaluate standard characteristic
  Kv/Kv1=if LinearCharacteristic then (kv0_effective+(1-kv0_effective)*yLim/y1) else kv0_effective*exp(Modelica.Math.log(1/kv0_effective)*yLim/y1);
  // pressure drop under real conditions
  dp/dp0 = medium.rho/rho0*(V_flow/Kv)*abs(V_flow/Kv);
  // no energy exchange with medium
  Q_flow = frictionLoss*V_flow*dp;


annotation (Documentation(info="<html><p><b>Purpose</b> Simple valve</p><p><b>Interfaces</b> See the named arguments, record fields, or inherited connectors.</p><p><b>Parameters</b> parameter Boolean LinearCharacteristic(start=true); parameter Real y1(min=small, start=1) \"Max. valve opening\"; parameter Modelica.Units.SI.VolumeFlowRate Kv1(min=small, start=1); parameter Real kv0(min=small,max=1-small, start=0.01); parameter Modelica.Units.SI.Pressure dp0(start=1) \"Standard pressure drop\"; parameter Modelica.Units.SI.Density rho0(start=10); parameter Real frictionLoss(min=0, max=1, start=0); parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter Modelica.Units.SI.Time faultStartTime=1; parameter Modelica.Units.SI.Time faultEndTime=3; parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05</p><p><b>Behavior</b> Simple valve</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.ThermalDegradation\">MSLFaults.Examples.ThermalDegradation</a></p><p><b>Purpose</b> Fault-enhanced Valve</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> The established physical equations and fault modes use the common activation and severity law.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Thermal.FluidHeatFlow.Components.Valve</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.ThermalDegradation\">MSLFaults.Examples.ThermalDegradation</a></p><p>用法：将 FaultableValve 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p>
<p>Simple controlled valve.</p>
<p>
Standard characteristic Kv=<em>f </em>(y) is given at standard conditions (dp0, rho0),
</p>
<ul>
<li>either linear :<code> Kv/Kv1 = Kv0/Kv1 + (1-Kv0/Kv1) * y/Y1</code></li>
<li>or exponential:<code> Kv/Kv1 = Kv0/Kv1 * exp[log(Kv1/Kv0) * y/Y1]</code></li>
</ul>
<p>
where:
</p>
<ul>
<li><code>Kv0 ... min. flow @ y = 0</code></li>
<li><code>Y1 .... max. valve opening</code></li>
<li><code>Kv1 ... max. flow @ y = Y1</code></li>
</ul>
<p>
Flow resistance under real conditions is calculated by
</p>
<blockquote><pre>
V_flow**2 * rho / dp = Kv(y)**2 * rho0 / dp0
</pre></blockquote>
</html>"),
  Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
            100}}),
      graphics={
        Polygon(points={{-90,10},{-60,10},{-60,60},{0,0},{60,60},{60,10},{90,10},{90,-10},{60,-10},{60,-60},{0,0},{-60,-60},{-60,-10},{-90,-10},{-90,10}}, lineColor={255,0,0}, fillColor={0,0,255}, fillPattern=FillPattern.Solid, lineThickness=0.75),
        Line(points={{0,80},{0,0}}, color={0,0,127}, thickness=0.5),
        Text(extent={{-150,-70},{150,-110}}, textString="%name", textColor={0,0,255}),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableValve;
