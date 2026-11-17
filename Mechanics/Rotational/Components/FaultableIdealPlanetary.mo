within MSLFaults.Mechanics.Rotational.Components;
model FaultableIdealPlanetary "Planetary gear with ratio and additional torque faults"
  parameter Real ratio(start=100/50)
    "Number of ring_teeth/sun_teeth (e.g., ratio=100/50)";

  // kinematic relationship
  Modelica.Mechanics.Rotational.Interfaces.Flange_a sun "Flange of sun shaft" annotation (Placement(
        transformation(extent={{-110,-10},{-90,10}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_a carrier "Flange of carrier shaft" annotation (
      Placement(transformation(extent={{-110,30},{-90,50}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b ring "Flange of ring shaft" annotation (Placement(
        transformation(extent={{90,-10},{110,10}})));

  type FaultMode=enumeration(Normal "Nominal component relation", GearRatioError "Changed kinematic transmission ratio", ToothDamage "Sinusoidal tooth-related torque", LockedGear "Declared gear-locking approximation");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault law";
  parameter Real severity(min=0,max=1)=1 "Scale of the activation envelope";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault request";
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal half-open fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a complete activation or recovery ramp";
  Real faultActivation(min=0,max=1) "Activation envelope used by the fault law";
  Real startActivation(min=0,max=1) "Activation envelope divided by positive severity";
  Real endActivation(min=0,max=1) "One while activation or its recovery envelope remains present";
  parameter Real ratioFault=0.9*ratio "Transmission ratio at full ratio-error effect";
  parameter Modelica.Units.SI.Torque toothTorque=1 "Amplitude of the prescribed tooth-torque sinusoid";
  Real ratio_effective "Transmission ratio in the current kinematic relation";
  Modelica.Units.SI.Torque faultTorque "Additional torque in the faulted balance";

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
    Documentation(info="<html><p>GearRatioError modifies the exact planetary angular relation. ToothDamage adds a sinusoidal torque at 20 Hz. LockedGear adds a torque proportional to the absolute carrier speed, with opposite contributions in the ring and carrier torque equations.</p><p>Connect sun, carrier and ring; set the nominal ratio and ratioFault or toothTorque, and inspect all three shaft torques.</p><p>The additional torques do not impose a rigid shaft lock; the model contains no inertia or separate thermal dissipation port.</p><p>Nominal reference: <code>Modelica.Mechanics.Rotational.Components.IdealPlanetary</code>. Activation uses the configured half-open fault window or the optional trigger; inspect <code>effectLevel</code> during recovery.</p></html>"),
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
