within MSLFaults.Mechanics.Rotational.Components;
model FaultableInertia "1D-rotational component with inertia"
  extends Modelica.Mechanics.Rotational.Interfaces.PartialTwoFlanges;
  parameter Modelica.Units.SI.Inertia J(min=0, start=1) "Moment of inertia";
  parameter StateSelect stateSelect=StateSelect.default
    "Priority to use phi and w as states"
    annotation (HideResult=true, Dialog(tab="Advanced"));
  Modelica.Units.SI.Angle phi(stateSelect=stateSelect)
    "Absolute rotation angle of component"
    annotation (Dialog(group="Initialization", showStartAttribute=true));
  Modelica.Units.SI.AngularVelocity w(stateSelect=stateSelect)
    "Absolute angular velocity of component (= der(phi))"
    annotation (Dialog(group="Initialization", showStartAttribute=true));
  Modelica.Units.SI.AngularAcceleration a
    "Absolute angular acceleration of component (= der(w))"
    annotation (Dialog(group="Initialization", showStartAttribute=true));

  type FaultMode=enumeration(Normal "正常", InertiaDrift "转动惯量漂移", InertiaLoss "惯量下降", RotorLock "转子卡死");
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter Modelica.Units.SI.Time faultStartTime=1;
  parameter Modelica.Units.SI.Time faultEndTime=3;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05;
  Real faultActivation(min=0,max=1);
  Real startActivation(min=0,max=1);
  Real endActivation(min=0,max=1);
  parameter Modelica.Units.SI.Inertia JFault=0.5*J;
  parameter Modelica.Units.SI.RotationalDampingConstant lockDamping=1e9;
  Modelica.Units.SI.Inertia J_effective;
  Modelica.Units.SI.Torque lockTorque;

public
  parameter Boolean enabled=true "Enable the selected fault mode";
  parameter Boolean useExternalTrigger=false "Use trigger instead of the internal time window" annotation(Evaluate=true);
  Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger
    annotation(Placement(transformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270),
      iconTransformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270)));
  output Boolean active "Requested or latched fault";
  output Real effectLevel(min=0,max=1) "Current realized fault effect";
  output Integer activationCount "Number of rising activations";
  Modelica.Units.SI.AngularMomentum momentum;
  Modelica.Units.SI.Energy storedEnergy;
  Modelica.Units.SI.Power lossPower,parameterPower,portPower;
protected
  MSLFaults.Utilities.Internal.Activation faultGate(enabled=enabled and faultMode<>FaultMode.Normal,
    permanent=false,severity=severity,startTime=faultStartTime,endTime=faultEndTime,
    transitionTime=transitionTime,useExternalTrigger=useExternalTrigger);
initial equation
  assert(J>=0 and lockDamping>=0,"The nominal inertial and damping terms must be nonnegative.");
  assert(not (faultMode==FaultMode.InertiaDrift or faultMode==FaultMode.InertiaLoss) or (J>0 and JFault>0),"Varying inertia requires positive endpoint parameters.");
equation
  connect(trigger,faultGate.externalTrigger);
  if not useExternalTrigger then faultGate.externalTrigger=false; end if;
  active=faultGate.active;
  effectLevel=faultGate.effectLevel;
  activationCount=faultGate.activationCount;

  startActivation=if severity>0 then effectLevel/severity else 0;
  endActivation=if active or effectLevel>0 then 1 else 0;
  faultActivation=effectLevel;

  phi = flange_a.phi;
  phi = flange_b.phi;
  w = der(phi);
  a = der(w);
  J_effective=if faultMode==FaultMode.InertiaDrift or faultMode==FaultMode.InertiaLoss then J+faultActivation*(JFault-J) else J;
  lockTorque=if faultMode==FaultMode.RotorLock then faultActivation*lockDamping*w else 0;
  der(momentum)+lockTorque=flange_a.tau+flange_b.tau;

  momentum=J_effective*w;
  storedEnergy=0.5*momentum*w;
  lossPower=lockTorque*w;
  portPower=(flange_a.tau+flange_b.tau)*w;
  parameterPower=der(storedEnergy)-portPower+lossPower;
  annotation (Documentation(info="<html><p><b>Purpose</b> 1D-rotational component with inertia</p><p><b>Interfaces</b> Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger; output Boolean active \"Requested or latched fault\"; output Real effectLevel(min=0,max=1) \"Current realized fault effect\"; output Integer activationCount \"Number of rising activations\"</p><p><b>Parameters</b> parameter Modelica.Units.SI.Inertia J(min=0, start=1) \"Moment of inertia\"; parameter StateSelect stateSelect=StateSelect.default; parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter Modelica.Units.SI.Time faultStartTime=1; parameter Modelica.Units.SI.Time faultEndTime=3; parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05; parameter Modelica.Units.SI.Inertia JFault=0.5*J; parameter Modelica.Units.SI.RotationalDampingConstant lockDamping=1e9; parameter Boolean enabled=true \"Enable the selected fault mode\"; parameter Boolean useExternalTrigger=false \"Use trigger instead of the internal time window\"</p><p><b>Behavior</b> 1D-rotational component with inertia</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.RotationalDamping\">MSLFaults.Examples.RotationalDamping</a></p><p><b>Purpose</b> Fault-enhanced Inertia</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> The variable inertial parameter is a behavioral approximation. Momentum is conserved in the absence of external force or torque. Locking adds finite dissipative resistance.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Mechanics.Rotational.Components.Inertia</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.RotationalDamping\">MSLFaults.Examples.RotationalDamping</a></p><p>用法：将 FaultableInertia 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p>
<p>
Rotational component with <strong>inertia</strong> and two rigidly connected flanges.
</p>
</html>"),
       Icon(coordinateSystem(preserveAspectRatio=true,
    extent={{-100.0,-100.0},{100.0,100.0}}),
      graphics={
        Rectangle(lineColor={64,64,64}, fillColor={192,192,192}, fillPattern=FillPattern.HorizontalCylinder, extent={{-100,-10},{-50,10}}, lineThickness=0.5),
        Rectangle(lineColor={64,64,64}, fillColor={192,192,192}, fillPattern=FillPattern.HorizontalCylinder, extent={{50,-10},{100,10}}, lineThickness=0.5),
        Line(points={{-80,-25},{-60,-25}}, color={64,64,64}, thickness=0.5),
        Line(points={{60,-25},{80,-25}}, color={64,64,64}, thickness=0.5),
        Line(points={{-70,-25},{-70,-70}}, color={64,64,64}, thickness=0.5),
        Line(points={{70,-25},{70,-70}}, color={64,64,64}, thickness=0.5),
        Line(points={{-80,25},{-60,25}}, color={64,64,64}, thickness=0.5),
        Line(points={{60,25},{80,25}}, color={64,64,64}, thickness=0.5),
        Line(points={{-70,45},{-70,25}}, color={64,64,64}, thickness=0.5),
        Line(points={{70,45},{70,25}}, color={64,64,64}, thickness=0.5),
        Line(points={{-70,-70},{70,-70}}, color={64,64,64}, thickness=0.5),
        Rectangle(lineColor={64,64,64}, fillColor={255,255,255}, fillPattern=FillPattern.HorizontalCylinder, extent={{-50,-50},{50,50}}, radius=10, lineThickness=0.5),
        Text(textColor={0,0,255}, extent={{-150,155},{150,125}}, textString="%name"),
        Text(extent={{-150,-120},{150,-80}}, textString="J=%J"),
        Rectangle(lineColor={64,64,64}, fillColor={255,255,255}, extent={{-50,-50},{50,50}}, radius=10, lineThickness=0.5),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableInertia;
