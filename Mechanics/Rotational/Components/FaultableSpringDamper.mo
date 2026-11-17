within MSLFaults.Mechanics.Rotational.Components;
model FaultableSpringDamper "Rotational spring and damper with latched fracture effects"
  parameter Modelica.Units.SI.RotationalSpringConstant c(final min=0, start=1.0e5)
    "Spring constant";
  parameter Modelica.Units.SI.RotationalDampingConstant d(final min=0, start=0)
    "Damping constant";
  parameter Modelica.Units.SI.Angle phi_rel0=0 "Unstretched spring angle";
  extends
    Modelica.Mechanics.Rotational.Interfaces.PartialCompliantWithRelativeStates;
  extends
    Modelica.Thermal.HeatTransfer.Interfaces.PartialElementaryConditionalHeatPortWithoutT;
  type FaultMode=enumeration(Normal "Nominal component relation", StiffnessLoss "Reduced stiffness", DampingLoss "Reduced damping", DampingIncrease "Configured damping increase", Broken "Latched fracture effect", Seizure "Increased finite mechanical resistance");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault law";
  parameter Real severity(min=0,max=1)=1 "Scale of the activation envelope";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault request";
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal half-open fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a complete activation or recovery ramp";
  Real faultActivation(min=0,max=1) "Activation envelope used by the fault law";
  Real startActivation(min=0,max=1) "Activation envelope divided by positive severity";
  Real endActivation(min=0,max=1) "One while activation or its recovery envelope remains present";
  parameter Modelica.Units.SI.RotationalSpringConstant cFault=0.5*c "Stiffness target for the selected fault law";
  parameter Modelica.Units.SI.RotationalDampingConstant dFault=0.5*d "Damping target for the selected fault law";
  parameter Modelica.Units.SI.RotationalDampingConstant dSeized=1e9 "Finite damping at full seizure effect";
  Modelica.Units.SI.RotationalSpringConstant c_effective "Stiffness used by the constitutive relation";
  Modelica.Units.SI.RotationalDampingConstant d_effective "Damping used by the constitutive relation";
protected
  Modelica.Units.SI.Torque tau_c "Spring torque";
  Modelica.Units.SI.Torque tau_d "Damping torque";

public
  parameter Boolean enabled=true "Enable activation of the selected fault mode";
  parameter Boolean useExternalTrigger=false "Use trigger in place of the internal fault window" annotation(Evaluate=true);
  Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger "External Boolean activation request"
    annotation(Placement(transformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270),
      iconTransformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270)));
  output Boolean active "Requested or latched activation state";
  output Real effectLevel(min=0,max=1) "Severity-scaled activation envelope";
  output Integer activationCount "Number of rising activations, including an initially active request";

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
    Documentation(info="<html><p>StiffnessLoss interpolates towards cFault. Both DampingLoss and DampingIncrease interpolate towards dFault. Broken latches and reduces stiffness and damping towards zero; Seizure interpolates towards dSeized. lossPower includes the damping contribution.</p><p>Set c, d and the unstretched coordinate; choose cFault, dFault or dSeized and inspect spring and damping efforts.</p><p>Set dFault&gt;d to obtain increased damping: its default is 0.5*d for both damping modes. Broken persists after the trigger ends and removes both terms at full severity.</p><p>Nominal reference: <code>Modelica.Mechanics.Rotational.Components.SpringDamper</code>. Activation uses the configured half-open fault window or the optional trigger; inspect <code>effectLevel</code> during recovery.</p></html>"),
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
