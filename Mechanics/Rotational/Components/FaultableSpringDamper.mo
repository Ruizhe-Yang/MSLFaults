within MSLFaults.Mechanics.Rotational.Components;
model FaultableSpringDamper "Linear 1D rotational spring and damper in parallel"
  parameter Modelica.Units.SI.RotationalSpringConstant c(final min=0, start=1.0e5)
    "Spring constant";
  parameter Modelica.Units.SI.RotationalDampingConstant d(final min=0, start=0)
    "Damping constant";
  parameter Modelica.Units.SI.Angle phi_rel0=0 "Unstretched spring angle";
  extends
    Modelica.Mechanics.Rotational.Interfaces.PartialCompliantWithRelativeStates;
  extends
    Modelica.Thermal.HeatTransfer.Interfaces.PartialElementaryConditionalHeatPortWithoutT;
  type FaultMode=enumeration(Normal "正常", StiffnessLoss "刚度下降", DampingLoss "阻尼下降", DampingIncrease "阻尼增加", Broken "断裂", Seizure "卡死");
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter Modelica.Units.SI.Time faultStartTime=1;
  parameter Modelica.Units.SI.Time faultEndTime=3;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05;
  Real faultActivation(min=0,max=1);
  Real startActivation(min=0,max=1);
  Real endActivation(min=0,max=1);
  parameter Modelica.Units.SI.RotationalSpringConstant cFault=0.5*c;
  parameter Modelica.Units.SI.RotationalDampingConstant dFault=0.5*d;
  parameter Modelica.Units.SI.RotationalDampingConstant dSeized=1e9;
  Modelica.Units.SI.RotationalSpringConstant c_effective;
  Modelica.Units.SI.RotationalDampingConstant d_effective;
protected
  Modelica.Units.SI.Torque tau_c "Spring torque";
  Modelica.Units.SI.Torque tau_d "Damping torque";

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
    permanent=faultMode==FaultMode.Broken,severity=severity,startTime=faultStartTime,endTime=faultEndTime,
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

  c_effective=if faultMode==FaultMode.StiffnessLoss then c+faultActivation*(cFault-c)
    elseif faultMode==FaultMode.Broken then c*(1-faultActivation) else c;
  d_effective=if faultMode==FaultMode.DampingLoss or faultMode==FaultMode.DampingIncrease then d+faultActivation*(dFault-d)
    elseif faultMode==FaultMode.Broken then d*(1-faultActivation)
    elseif faultMode==FaultMode.Seizure then d+faultActivation*(dSeized-d) else d;
  tau_c=c_effective*(phi_rel-phi_rel0);
  tau_d=d_effective*w_rel;
  tau = tau_c + tau_d;
  lossPower = tau_d*w_rel;


  annotation (
    Documentation(info="<html><p><b>Purpose</b> Linear 1D rotational spring and damper in parallel</p><p><b>Interfaces</b> See the named arguments, record fields, or inherited connectors.</p><p><b>Parameters</b> parameter Modelica.Units.SI.RotationalSpringConstant c(final min=0, start=1.0e5); parameter Modelica.Units.SI.RotationalDampingConstant d(final min=0, start=0); parameter Modelica.Units.SI.Angle phi_rel0=0 \"Unstretched spring angle\"; parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter Modelica.Units.SI.Time faultStartTime=1; parameter Modelica.Units.SI.Time faultEndTime=3; parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05; parameter Modelica.Units.SI.RotationalSpringConstant cFault=0.5*c; parameter Modelica.Units.SI.RotationalDampingConstant dFault=0.5*d; parameter Modelica.Units.SI.RotationalDampingConstant dSeized=1e9</p><p><b>Behavior</b> Linear 1D rotational spring and damper in parallel</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.RotationalDamping\">MSLFaults.Examples.RotationalDamping</a></p><p><b>Purpose</b> Fault-enhanced SpringDamper</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> The established physical equations and fault modes use the common activation and severity law.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery. Broken latch until a new simulation. faultEndTime does not repair a latched fault.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Mechanics.Rotational.Components.SpringDamper</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.RotationalDamping\">MSLFaults.Examples.RotationalDamping</a></p><p>用法：将 FaultableSpringDamper 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p>
<p>
A <strong>spring</strong> and <strong>damper</strong> element <strong>connected in parallel</strong>.
The component can be
connected either between two inertias/gears to describe the shaft elasticity
and damping, or between an inertia/gear and the housing (component Fixed),
to describe a coupling of the element with the housing via a spring/damper.
</p>

<p>
See also the discussion
<a href=\"modelica://Modelica.Mechanics.Rotational.UsersGuide.StateSelection\">State Selection</a>
in the User's Guide of the Rotational library.
</p>
</html>"),
    Icon(coordinateSystem(preserveAspectRatio=true,
        extent={{-100,-100},{100,100}}),
      graphics={
        Line(points={{-80,40},{-60,40},{-45,10},{-15,70},{15,10},{45,70},{60,40},{80,40}}, color={64,64,64}, thickness=0.5),
        Line(points={{-80,40},{-80,-40}}, color={64,64,64}, thickness=0.5),
        Line(points={{-80,-40},{-50,-40}}, color={64,64,64}, thickness=0.5),
        Rectangle(extent={{-50,-10},{40,-70}}, fillColor={192,192,192}, fillPattern=FillPattern.Solid, lineColor={64,64,64}, lineThickness=0.5),
        Line(points={{-50,-10},{70,-10}}, color={64,64,64}, thickness=0.5),
        Line(points={{-50,-70},{70,-70}}, color={64,64,64}, thickness=0.5),
        Line(points={{40,-40},{80,-40}}, color={64,64,64}, thickness=0.5),
        Line(points={{80,40},{80,-40}}, color={64,64,64}, thickness=0.5),
        Line(points={{-90,0},{-80,0}}, color={64,64,64}, thickness=0.5),
        Line(points={{80,0},{90,0}}, color={64,64,64}, thickness=0.5),
        Text(origin={0,-9}, extent={{-150,-144},{150,-104}}, textString="d=%d"),
        Text(extent={{-190,155},{190,125}}, textColor={0,0,255}, textString="%name"),
        Text(origin={0,-7}, extent={{-150,-108},{150,-68}}, textString="c=%c"),
        Line(visible=useHeatPort, points={{-100,-100},{-100,-55},{-5,-55}}, color={191,0,0}, pattern=LinePattern.Dot, thickness=0.5),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableSpringDamper;
