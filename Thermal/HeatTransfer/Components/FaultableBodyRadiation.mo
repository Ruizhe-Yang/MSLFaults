within MSLFaults.Thermal.HeatTransfer.Components;
model FaultableBodyRadiation "Lumped thermal element for radiation heat transfer"
  extends Modelica.Thermal.HeatTransfer.Interfaces.Element1D;
  parameter Real Gr(unit="m2")
    "Net radiation conductance between two surfaces (see docu)";
  type FaultMode=enumeration(Normal "正常", EmissivityDrift "辐射系数漂移", SurfaceContamination "表面污染", RadiationLoss "辐射能力下降", RadiationShort "辐射耦合增强");
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter Modelica.Units.SI.Time faultStartTime=1;
  parameter Modelica.Units.SI.Time faultEndTime=3;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05;
  Real faultActivation(min=0,max=1);
  Real startActivation(min=0,max=1);
  Real endActivation(min=0,max=1);
  parameter Real GrFault(unit="m2")=0.5*Gr;
  parameter Real GrShort(unit="m2")=1000*Gr;
  Real Gr_effective(unit="m2");

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
  Gr_effective=if faultMode==FaultMode.EmissivityDrift or faultMode==FaultMode.SurfaceContamination then Gr+faultActivation*(GrFault-Gr) elseif faultMode==FaultMode.RadiationLoss then Gr*(1-faultActivation) elseif faultMode==FaultMode.RadiationShort then Gr+faultActivation*(GrShort-Gr) else Gr;
  Q_flow=Gr_effective*Modelica.Constants.sigma*(port_a.T^4-port_b.T^4);


  annotation (
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
            100,100}}),
      graphics={
        Rectangle(extent={{50,80},{90,-80}}, fillColor={192,192,192}, fillPattern=FillPattern.Backward, lineColor={64,64,64}, lineThickness=0.5),
        Rectangle(extent={{-90,80},{-50,-80}}, fillColor={192,192,192}, fillPattern=FillPattern.Backward, lineColor={64,64,64}, lineThickness=0.5),
        Line(points={{-36,10},{36,10}}, color={191,0,0}, thickness=0.5),
        Line(points={{-36,10},{-26,16}}, color={191,0,0}, thickness=0.5),
        Line(points={{-36,10},{-26,4}}, color={191,0,0}, thickness=0.5),
        Line(points={{-36,-10},{36,-10}}, color={191,0,0}, thickness=0.5),
        Line(points={{26,-16},{36,-10}}, color={191,0,0}, thickness=0.5),
        Line(points={{26,-4},{36,-10}}, color={191,0,0}, thickness=0.5),
        Line(points={{-36,-30},{36,-30}}, color={191,0,0}, thickness=0.5),
        Line(points={{-36,-30},{-26,-24}}, color={191,0,0}, thickness=0.5),
        Line(points={{-36,-30},{-26,-36}}, color={191,0,0}, thickness=0.5),
        Line(points={{-36,30},{36,30}}, color={191,0,0}, thickness=0.5),
        Line(points={{26,24},{36,30}}, color={191,0,0}, thickness=0.5),
        Line(points={{26,36},{36,30}}, color={191,0,0}, thickness=0.5),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Text(extent={{-150,-90},{150,-120}}, textString="Gr=%Gr"),
        Rectangle(extent={{-50,80},{-44,-80}}, lineColor={191,0,0}, fillColor={191,0,0}, fillPattern=FillPattern.Solid, lineThickness=0.5),
        Rectangle(extent={{45,80},{50,-80}}, lineColor={191,0,0}, fillColor={191,0,0}, fillPattern=FillPattern.Solid, lineThickness=0.5),
        Text(extent={{108,94},{139,62}}, textString="F", textColor={255,0,0}, fontSize=0)
      }),
    Documentation(info="<html><p><b>Purpose</b> Lumped thermal element for radiation heat transfer</p><p><b>Interfaces</b> Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger; output Boolean active \"Requested or latched fault\"; output Real effectLevel(min=0,max=1) \"Current realized fault effect\"; output Integer activationCount \"Number of rising activations\"</p><p><b>Parameters</b> parameter Real Gr(unit=\"m2\"); parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter Modelica.Units.SI.Time faultStartTime=1; parameter Modelica.Units.SI.Time faultEndTime=3; parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05; parameter Real GrFault(unit=\"m2\")=0.5*Gr; parameter Real GrShort(unit=\"m2\")=1000*Gr; parameter Boolean enabled=true \"Enable the selected fault mode\"; parameter Boolean useExternalTrigger=false \"Use trigger instead of the internal time window\"</p><p><b>Behavior</b> Lumped thermal element for radiation heat transfer</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.ThermalDegradation\">MSLFaults.Examples.ThermalDegradation</a></p><p><b>Purpose</b> Fault-enhanced BodyRadiation</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> The established physical equations and fault modes use the common activation and severity law.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Thermal.HeatTransfer.Components.BodyRadiation</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.ThermalDegradation\">MSLFaults.Examples.ThermalDegradation</a></p><p>用法：将 FaultableBodyRadiation 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p>
<p>
This is a model describing the thermal radiation, i.e., electromagnetic
radiation emitted between two bodies as a result of their temperatures.
The following constitutive equation is used:
</p>
<blockquote><pre>
Q_flow = Gr*sigma*(port_a.T^4 - port_b.T^4);
</pre></blockquote>
<p>
where Gr is the radiation conductance and sigma is the Stefan-Boltzmann
constant (= Modelica.Constants.sigma). Gr may be determined by
measurements and is assumed to be constant over the range of operations.
</p>
<p>
For simple cases, Gr may be analytically computed. The analytical
equations use epsilon, the emission value of a body which is in the
range 0..1. Epsilon=1, if the body absorbs all radiation (= black body).
Epsilon=0, if the body reflects all radiation and does not absorb any.
</p>
<blockquote><pre>
Typical values for epsilon:
aluminium, polished    0.04
copper, polished       0.04
gold, polished         0.02
paper                  0.09
rubber                 0.95
silver, polished       0.02
wood                   0.85..0.9
</pre></blockquote>
<p><strong>Analytical Equations for Gr</strong></p>
<p>
<strong>Small convex object in large enclosure</strong>
(e.g., a hot machine in a room):
</p>
<blockquote><pre>
Gr = e*A
where
   e: Emission value of object (0..1)
   A: Surface area of object where radiation
      heat transfer takes place
</pre></blockquote>
<p><strong>Two parallel plates</strong>:</p>
<blockquote><pre>
Gr = A/(1/e1 + 1/e2 - 1)
where
   e1: Emission value of plate1 (0..1)
   e2: Emission value of plate2 (0..1)
   A : Area of plate1 (= area of plate2)
</pre></blockquote>
<p><strong>Two long cylinders in each other</strong>, where radiation takes
place from the inner to the outer cylinder):
</p>
<blockquote><pre>
Gr = 2*pi*r1*L/(1/e1 + (1/e2 - 1)*(r1/r2))
where
   pi: = Modelica.Constants.pi
   r1: Radius of inner cylinder
   r2: Radius of outer cylinder
   L : Length of the two cylinders
   e1: Emission value of inner cylinder (0..1)
   e2: Emission value of outer cylinder (0..1)
</pre></blockquote>
</html>"));
end FaultableBodyRadiation;
