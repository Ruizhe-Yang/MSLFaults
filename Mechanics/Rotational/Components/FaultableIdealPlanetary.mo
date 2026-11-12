within MSLFaults.Mechanics.Rotational.Components;
model FaultableIdealPlanetary "Ideal planetary gear box"
  parameter Real ratio(start=100/50)
    "Number of ring_teeth/sun_teeth (e.g., ratio=100/50)";

  // kinematic relationship
  Modelica.Mechanics.Rotational.Interfaces.Flange_a sun "Flange of sun shaft" annotation (Placement(
        transformation(extent={{-110,-10},{-90,10}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_a carrier "Flange of carrier shaft" annotation (
      Placement(transformation(extent={{-110,30},{-90,50}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b ring "Flange of ring shaft" annotation (Placement(
        transformation(extent={{90,-10},{110,10}})));

  type FaultMode=enumeration(Normal "正常", GearRatioError "齿数比误差", ToothDamage "齿损伤", LockedGear "行星机构锁死");
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter Modelica.Units.SI.Time faultStartTime=1;
  parameter Modelica.Units.SI.Time faultEndTime=3;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05;
  Real faultActivation(min=0,max=1);
  Real startActivation(min=0,max=1);
  Real endActivation(min=0,max=1);
  parameter Real ratioFault=0.9*ratio;
  parameter Modelica.Units.SI.Torque toothTorque=1;
  Real ratio_effective;
  Modelica.Units.SI.Torque faultTorque;

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

  ratio_effective=if faultMode==FaultMode.GearRatioError then ratio+faultActivation*(ratioFault-ratio) else ratio;
  faultTorque=if faultMode==FaultMode.ToothDamage then faultActivation*toothTorque*sin(40*Modelica.Constants.pi*time)
    elseif faultMode==FaultMode.LockedGear then faultActivation*1e8*der(carrier.phi) else 0;
  (1+ratio_effective)*carrier.phi=sun.phi+ratio_effective*ring.phi;
  ring.tau=ratio_effective*sun.tau+faultTorque;
  carrier.tau=-(1+ratio_effective)*sun.tau-faultTorque;


  annotation (
    Documentation(info="<html><p><b>Purpose</b> Ideal planetary gear box</p><p><b>Interfaces</b> Modelica.Mechanics.Rotational.Interfaces.Flange_a sun \"Flange of sun shaft\" annotation (Placement(; Modelica.Mechanics.Rotational.Interfaces.Flange_a carrier \"Flange of carrier shaft\" annotation (; Modelica.Mechanics.Rotational.Interfaces.Flange_b ring \"Flange of ring shaft\" annotation (Placement(; Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger; output Boolean active \"Requested or latched fault\"; output Real effectLevel(min=0,max=1) \"Current realized fault effect\"; output Integer activationCount \"Number of rising activations\"</p><p><b>Parameters</b> parameter Real ratio(start=100/50); parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter Modelica.Units.SI.Time faultStartTime=1; parameter Modelica.Units.SI.Time faultEndTime=3; parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05; parameter Real ratioFault=0.9*ratio; parameter Modelica.Units.SI.Torque toothTorque=1; parameter Boolean enabled=true \"Enable the selected fault mode\"; parameter Boolean useExternalTrigger=false \"Use trigger instead of the internal time window\"</p><p><b>Behavior</b> Ideal planetary gear box</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.RotationalDamping\">MSLFaults.Examples.RotationalDamping</a></p><p><b>Purpose</b> Fault-enhanced IdealPlanetary</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> The established physical equations and fault modes use the common activation and severity law.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Mechanics.Rotational.Components.IdealPlanetary</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.RotationalDamping\">MSLFaults.Examples.RotationalDamping</a></p><p>用法：将 FaultableIdealPlanetary 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p>
<p>
The IdealPlanetary gear box is an ideal gear without inertia,
elasticity, damping or backlash consisting
of an inner <strong>sun</strong> wheel, an outer <strong>ring</strong> wheel and a
<strong>planet</strong> wheel located between sun and ring wheel. The bearing
of the planet wheel shaft is fixed in the planet <strong>carrier</strong>.
The component can be connected to other elements at the
sun, ring and/or carrier flanges. It is not possible to connect
to the planet wheel. If inertia shall not be neglected,
the sun, ring and carrier inertias can be easily added by attaching
inertias (= model Inertia) to the corresponding connectors.
The inertias of the planet wheels are always neglected.
</p>
<p>
The icon of the planetary gear signals that the sun and carrier
flanges are on the left side and the ring flange is on the right side
of the gear box. However, this component is generic and is valid
independently how the flanges are actually placed (e.g., sun wheel
may be placed on the right side instead on the left side in reality).
</p>
<p>
The ideal planetary gearbox is uniquely defined by the ratio
of the number of ring teeth zr with respect to the number of
sun teeth zs. For example, if there are 100 ring teeth and
50 sun teeth then ratio = zr/zs = 2. The number of planet teeth
zp has to fulfill the following relationship:
</p>
<blockquote><pre>
<strong>zp := (zr - zs) / 2</strong>
</pre></blockquote>
<p>
Therefore, in the above example zp = 25 is required.
</p>
<p>
According to the overall convention, the positive direction of all
vectors, especially the absolute angular velocities and cut-torques
in the flanges, are along the axis vector displayed in the icon.
</p>

</html>"),
    Icon(coordinateSystem(preserveAspectRatio=true,
      extent={{-100.0,-100.0},{100.0,100.0}}),
      graphics={
        Rectangle(lineColor={160,160,164}, fillColor={160,160,164}, fillPattern=FillPattern.Solid, extent={{-50,100},{50,105}}, lineThickness=0.5),
        Rectangle(lineColor={160,160,164}, fillColor={160,160,164}, fillPattern=FillPattern.Solid, extent={{-50,-105},{50,-100}}, lineThickness=0.5),
        Line(points={{-90,40},{-70,40}}, color={64,64,64}, thickness=0.5),
        Line(points={{-80,50},{-60,50}}, color={64,64,64}, thickness=0.5),
        Line(points={{-70,50},{-70,40}}, color={64,64,64}, thickness=0.5),
        Line(points={{-80,80},{-59,80}}, color={64,64,64}, thickness=0.5),
        Line(points={{-70,100},{-70,80}}, color={64,64,64}, thickness=0.5),
        Text(textColor={0,0,255}, extent={{-150,155},{150,125}}, textString="%name"),
        Text(extent={{-150,-150},{150,-110}}, textString="ratio=%ratio"),
        Rectangle(lineColor={64,64,64}, fillColor={255,255,255}, fillPattern=FillPattern.HorizontalCylinder, extent={{10,-100},{50,100}}, lineThickness=0.5),
        Rectangle(lineColor={64,64,64}, fillColor={192,192,192}, fillPattern=FillPattern.HorizontalCylinder, extent={{-100,-10},{-50,10}}, lineThickness=0.5),
        Rectangle(lineColor={64,64,64}, fillColor={192,192,192}, fillPattern=FillPattern.HorizontalCylinder, extent={{50,-10},{100,10}}, lineThickness=0.5),
        Rectangle(lineColor={64,64,64}, fillColor={192,192,192}, fillPattern=FillPattern.HorizontalCylinder, extent={{-80,60},{-50,70}}, lineThickness=0.5),
        Rectangle(origin={-30,65}, fillColor={192,192,192}, fillPattern=FillPattern.HorizontalCylinder, extent={{-20,-19},{20,19}}, lineColor={64,64,64}, lineThickness=0.5),
        Rectangle(origin={-30,48}, fillColor={153,153,153}, fillPattern=FillPattern.Solid, extent={{-20,-2},{20,2}}, lineColor={64,64,64}, lineThickness=0.5),
        Rectangle(origin={-30,82}, fillColor={204,204,204}, fillPattern=FillPattern.Solid, extent={{-20,-2},{20,2}}, lineColor={64,64,64}, lineThickness=0.5),
        Rectangle(origin={-30,59}, fillColor={204,204,204}, fillPattern=FillPattern.Solid, extent={{-20,-4},{20,4}}, lineColor={64,64,64}, lineThickness=0.5),
        Rectangle(origin={-30,73}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, extent={{-20,-3},{20,3}}, lineColor={64,64,64}, lineThickness=0.5),
        Rectangle(origin={-30,-1}, fillColor={255,255,255}, fillPattern=FillPattern.HorizontalCylinder, extent={{-20,-35},{20,35}}, lineColor={64,64,64}, lineThickness=0.5),
        Rectangle(origin={-30,32}, fillColor={153,153,153}, fillPattern=FillPattern.Solid, extent={{-20,-2},{20,2}}, lineColor={64,64,64}, lineThickness=0.5),
        Rectangle(origin={-30,23}, fillColor={204,204,204}, fillPattern=FillPattern.Solid, extent={{-20,-3},{20,3}}, lineColor={64,64,64}, lineThickness=0.5),
        Rectangle(origin={-30,-10}, fillColor={204,204,204}, fillPattern=FillPattern.Solid, extent={{-20,-4},{20,4}}, lineColor={64,64,64}, lineThickness=0.5),
        Rectangle(origin={-30,-34}, fillColor={102,102,102}, fillPattern=FillPattern.Solid, extent={{-20,-2},{20,2}}, lineColor={64,64,64}, lineThickness=0.5),
        Rectangle(origin={-30,-25}, fillColor={153,153,153}, fillPattern=FillPattern.Solid, extent={{-20,-3},{20,3}}, lineColor={64,64,64}, lineThickness=0.5),
        Rectangle(origin={-30,8}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, extent={{-20,-4},{20,4}}, lineColor={64,64,64}, lineThickness=0.5),
        Rectangle(origin={-30,65}, fillColor={192,192,192}, extent={{-20,-19},{20,19}}, lineColor={64,64,64}, lineThickness=0.5),
        Rectangle(origin={-30,-1}, fillColor={255,255,255}, extent={{-20,-35},{20,35}}, lineColor={64,64,64}, lineThickness=0.5),
        Rectangle(lineColor={64,64,64}, fillColor={255,255,255}, extent={{10,-100},{50,100}}, lineThickness=0.5),
        Text(extent={{108,94},{139,62}}, textString="F", textColor={255,0,0}, fontSize=0)
      }),
  Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100,-100},{100,100}}), graphics={
    Polygon(lineColor = {128,128,128},
      fillColor = {128,128,128},
      fillPattern = FillPattern.Solid,
      points = {{58,130},{28,140},{28,120},{58,130}}),
    Line(points = {{-52,130},{28,130}})}));
end FaultableIdealPlanetary;
