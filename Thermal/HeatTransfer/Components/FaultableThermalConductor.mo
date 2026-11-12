within MSLFaults.Thermal.HeatTransfer.Components;
model FaultableThermalConductor
  "Lumped thermal element transporting heat without storing it"
  extends Modelica.Thermal.HeatTransfer.Interfaces.Element1D;
  parameter Modelica.Units.SI.ThermalConductance G
    "Constant thermal conductance of material";

  type FaultMode=enumeration(Normal "正常", ConductanceDrift "热导漂移", ConductanceLoss "热导下降", ThermalOpen "热开路", ThermalShort "热短路");
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter Modelica.Units.SI.Time faultStartTime=1;
  parameter Modelica.Units.SI.Time faultEndTime=3;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05;
  Real faultActivation(min=0,max=1);
  Real startActivation(min=0,max=1);
  Real endActivation(min=0,max=1);
  parameter Modelica.Units.SI.ThermalConductance GFault=0.5*G;
  parameter Modelica.Units.SI.ThermalConductance GOpen=1e-9;
  parameter Modelica.Units.SI.ThermalConductance GShort=1e9;
  Modelica.Units.SI.ThermalConductance G_effective;

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
  G_effective=if faultMode==FaultMode.ConductanceDrift or faultMode==FaultMode.ConductanceLoss then G+faultActivation*(GFault-G) elseif faultMode==FaultMode.ThermalOpen then G+faultActivation*(GOpen-G) elseif faultMode==FaultMode.ThermalShort then G+faultActivation*(GShort-G) else G;
  Q_flow=G_effective*dT;

  assert(G_effective>=0,"Passive heat transfer requires nonnegative conductance.");
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
            100,100}}),
      graphics={
        Rectangle(extent={{-90,70},{90,-70}}, pattern=LinePattern.None, fillColor={192,192,192}, fillPattern=FillPattern.Backward, lineColor={64,64,64}, lineThickness=0.5),
        Line(points={{-90,70},{-90,-70}}, thickness=0.5, color={64,64,64}),
        Line(points={{90,70},{90,-70}}, thickness=0.5, color={64,64,64}),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Text(extent={{-150,-80},{150,-110}}, textString="G=%G"),
        Text(extent={{108,94},{139,62}}, textString="F", textColor={255,0,0}, fontSize=0)
      }),
    Documentation(info="<html><p><b>Purpose</b> FaultableThermalConductor</p><p><b>Interfaces</b> Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger; output Boolean active \"Requested or latched fault\"; output Real effectLevel(min=0,max=1) \"Current realized fault effect\"; output Integer activationCount \"Number of rising activations\"</p><p><b>Parameters</b> parameter Modelica.Units.SI.ThermalConductance G; parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter Modelica.Units.SI.Time faultStartTime=1; parameter Modelica.Units.SI.Time faultEndTime=3; parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05; parameter Modelica.Units.SI.ThermalConductance GFault=0.5*G; parameter Modelica.Units.SI.ThermalConductance GOpen=1e-9; parameter Modelica.Units.SI.ThermalConductance GShort=1e9; parameter Boolean enabled=true \"Enable the selected fault mode\"; parameter Boolean useExternalTrigger=false \"Use trigger instead of the internal time window\"</p><p><b>Behavior</b> FaultableThermalConductor</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.ThermalDegradation\">MSLFaults.Examples.ThermalDegradation</a></p><p><b>Purpose</b> Fault-enhanced ThermalConductor</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> Heat flow is antisymmetric. The effective conductance is nonnegative.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Thermal.HeatTransfer.Components.ThermalConductor</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.ThermalDegradation\">MSLFaults.Examples.ThermalDegradation</a></p><p>用法：将 FaultableThermalConductor 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p>
<p>
This is a model for transport of heat without storing it; see also:
<a href=\"modelica://Modelica.Thermal.HeatTransfer.Components.ThermalResistor\">ThermalResistor</a>.
It may be used for complicated geometries where
the thermal conductance G (= inverse of thermal resistance)
is determined by measurements and is assumed to be constant
over the range of operations. If the component consists mainly of
one type of material and a regular geometry, it may be calculated,
e.g., with one of the following equations:
</p>
<ul>
<li><p>
    Conductance for a <strong>box</strong> geometry under the assumption
    that heat flows along the box length:</p>
    <blockquote><pre>
G = k*A/L
k: Thermal conductivity (material constant)
A: Area of box
L: Length of box
    </pre></blockquote>
    </li>
<li><p>
    Conductance for a <strong>cylindrical</strong> geometry under the assumption
    that heat flows from the inside to the outside radius
    of the cylinder:</p>
    <blockquote><pre>
G = 2*pi*k*L/log(r_out/r_in)
pi   : Modelica.Constants.pi
k    : Thermal conductivity (material constant)
L    : Length of cylinder
log  : Modelica.Math.log;
r_out: Outer radius of cylinder
r_in : Inner radius of cylinder
    </pre></blockquote>
    </li>
</ul>
<blockquote><pre>
Typical values for k at 20 degC in W/(m.K):
  aluminium   220
  concrete      1
  copper      384
  iron         74
  silver      407
  steel        45 .. 15 (V2A)
  wood         0.1 ... 0.2
</pre></blockquote>
</html>"));
end FaultableThermalConductor;
