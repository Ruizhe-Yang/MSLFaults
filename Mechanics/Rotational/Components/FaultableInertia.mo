within MSLFaults.Mechanics.Rotational.Components;
model FaultableInertia "Rotational inertia with momentum-based parameter variation"
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

  type FaultMode=enumeration(Normal "Nominal component relation", InertiaDrift "Changed moment of inertia", InertiaLoss "Reduced moment of inertia", RotorLock "Finite viscous shaft resistance");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault law";
  parameter Real severity(min=0,max=1)=1 "Scale of the activation envelope";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault request";
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal half-open fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a complete activation or recovery ramp";
  Real faultActivation(min=0,max=1) "Activation envelope used by the fault law";
  Real startActivation(min=0,max=1) "Activation envelope divided by positive severity";
  Real endActivation(min=0,max=1) "One while activation or its recovery envelope remains present";
  parameter Modelica.Units.SI.Inertia JFault=0.5*J "Moment of inertia at full parameter-change effect";
  parameter Modelica.Units.SI.RotationalDampingConstant lockDamping=1e9 "Finite viscous coefficient used by the locking approximation";
  Modelica.Units.SI.Inertia J_effective "Effective moment of inertia";
  Modelica.Units.SI.Torque lockTorque "Viscous torque opposing the shaft motion";

public
  parameter Boolean enabled=true "Enable activation of the selected fault mode";
  parameter Boolean useExternalTrigger=false "Use trigger in place of the internal fault window" annotation(Evaluate=true);
  Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger "External Boolean activation request"
    annotation(Placement(transformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270),
      iconTransformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270)));
  output Boolean active "Requested or latched activation state";
  output Real effectLevel(min=0,max=1) "Severity-scaled activation envelope";
  output Integer activationCount "Number of rising activations, including an initially active request";
  Modelica.Units.SI.AngularMomentum momentum "Momentum associated with the effective inertial parameter";
  Modelica.Units.SI.Energy storedEnergy "Stored mechanical energy from the effective parameters";
  Modelica.Units.SI.Power lossPower "Power dissipated by the finite locking resistance",parameterPower "Energy-rate contribution associated with prescribed parameter changes",portPower "Mechanical power entering through the flanges";
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
  annotation (Documentation(info="<html><p>InertiaDrift and InertiaLoss use the same interpolation from J to JFault. The balance differentiates momentum=J_effective*w. RotorLock adds finite viscous torque. parameterPower=der(storedEnergy)-portPower+lossPower accounts for prescribed parameter variation.</p><p>Set J and the initial shaft state; choose JFault or lockDamping and inspect momentum, storedEnergy, lossPower and parameterPower.</p><p>Varying inertia requires positive endpoint values. Drift follows the activation envelope without a separate ageing clock.</p><p>Nominal reference: <code>Modelica.Mechanics.Rotational.Components.Inertia</code>. Activation uses the configured half-open fault window or the optional trigger; inspect <code>effectLevel</code> during recovery.</p></html>"),
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
