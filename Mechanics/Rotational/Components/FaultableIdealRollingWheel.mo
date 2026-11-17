within MSLFaults.Mechanics.Rotational.Components;
model FaultableIdealRollingWheel
  "Ideal rolling wheel with radius and kinematic transmission faults"

  extends Modelica.Mechanics.Rotational.Interfaces.PartialElementaryRotationalToTranslational;
  parameter Modelica.Units.SI.Distance radius(start=0.3) "Wheel radius";

  type FaultMode=enumeration(Normal "Nominal component relation", RadiusError "Changed effective wheel radius", Slip "Reduced kinematic transmission scale", LockedWheel "Declared wheel-locking approximation");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault law";
  parameter Real severity(min=0,max=1)=1 "Scale of the activation envelope";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault request";
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal half-open fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a complete activation or recovery ramp";
  Real faultActivation(min=0,max=1) "Activation envelope used by the fault law";
  Real startActivation(min=0,max=1) "Activation envelope divided by positive severity";
  Real endActivation(min=0,max=1) "One while activation or its recovery envelope remains present";
  parameter Modelica.Units.SI.Distance radiusFault=0.9*radius "Wheel radius at full radius-error effect";
  parameter Real slipRatioFault(min=0,max=1)=0.2 "Reduction of transmission scale at full slip effect";
  Modelica.Units.SI.Distance radius_effective "Wheel radius used by the current relations";
  Real rollingScale "Scale applied to the wheel position relation";

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

  radius_effective=if faultMode==FaultMode.RadiusError then radius+faultActivation*(radiusFault-radius) else radius;
  rollingScale=if faultMode==FaultMode.Slip then 1-faultActivation*slipRatioFault elseif faultMode==FaultMode.LockedWheel then 1-faultActivation else 1;
  (flangeR.phi-internalSupportR.phi)*radius_effective*rollingScale=flangeT.s-internalSupportT.s;
  0=radius_effective*flangeT.f+flangeR.tau;


  annotation (
    Icon(coordinateSystem(preserveAspectRatio=true,
        extent={{-100.0,-100.0},{100.0,100.0}}),
      graphics={
        Rectangle(lineColor={64,64,64}, fillColor={192,192,192}, fillPattern=FillPattern.HorizontalCylinder, extent={{-100,-10},{-46,10}}, lineThickness=0.5),
        Ellipse(lineColor={64,64,64}, fillColor={255,255,255}, fillPattern=FillPattern.HorizontalCylinder, extent={{-50,-80},{10,80}}, lineThickness=0.5),
        Rectangle(lineColor={64,64,64}, fillColor={255,255,255}, pattern=LinePattern.None, fillPattern=FillPattern.HorizontalCylinder, extent={{-20,-80},{12,80}}, lineThickness=0.5),
        Ellipse(lineColor={64,64,64}, fillColor={128,128,128}, fillPattern=FillPattern.Solid, extent={{-16,-80},{44,80}}, lineThickness=0.5),
        Ellipse(lineColor={192,192,192}, fillColor={192,192,192}, fillPattern=FillPattern.Solid, extent={{-2,-52},{34,52}}, lineThickness=0.5),
        Ellipse(lineColor={64,64,64}, fillColor={192,192,192}, fillPattern=FillPattern.HorizontalCylinder, extent={{12,-10},{20,10}}, lineThickness=0.5),
        Rectangle(lineColor={64,64,64}, fillColor={192,192,192}, pattern=LinePattern.None, fillPattern=FillPattern.HorizontalCylinder, extent={{16,-10},{60,10}}, lineThickness=0.5),
        Ellipse(fillColor={192,192,192}, fillPattern=FillPattern.Solid, extent={{56,-10},{64,10}}, lineColor={64,64,64}, lineThickness=0.5),
        Text(textColor={0,0,255}, extent={{-150,155},{150,125}}, textString="%name"),
        Polygon(lineColor={0,127,0}, fillColor={0,127,0}, fillPattern=FillPattern.Solid, points={{80,10},{80,26},{60,26},{60,20},{70,20},{70,-20},{60,-20},{60,-26},{80,-26},{80,-10},{90,-10},{90,10},{80,10}}, lineThickness=0.5),
        Line(points={{-100,-20},{-60,-20}}, color={64,64,64}, thickness=0.5),
        Line(points={{-100,-20},{-100,-100}}, color={64,64,64}, thickness=0.5),
        Line(points={{-100,20},{-60,20}}, color={64,64,64}, thickness=0.5),
        Line(points={{100,-90},{-40,-90}}, color={0,127,0}, thickness=0.5),
        Line(points={{70,-26},{70,-50},{100,-50},{100,-100}}, color={0,127,0}, thickness=0.5),
        Line(origin={-2.5,80}, points={{-17.5,0},{17.5,0}}, color={64,64,64}, thickness=0.5),
        Line(origin={-2.5,-80}, points={{-17.5,0},{17.5,0}}, color={64,64,64}, thickness=0.5),
        Line(origin={38,10}, points={{-22,0},{22,0}}, color={64,64,64}, thickness=0.5),
        Line(origin={38,-10}, points={{-22,0},{22,0}}, color={64,64,64}, thickness=0.5),
        Text(extent={{-150,-120},{150,-150}}, textString="radius=%radius"),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }),
                             Documentation(info="<html><p>RadiusError changes the effective radius. Slip scales the rotational contribution to the position relation by 1-effectLevel*slipRatioFault. LockedWheel uses the scale 1-effectLevel. The torque-force relation uses radius_effective without that scale.</p><p>Connect flangeR and flangeT with appropriate supports; set radiusFault or slipRatioFault and inspect both displacements.</p><p>At full LockedWheel effect the translational displacement relative to support is zero. These modes alter ideal kinematics and do not describe tyre contact or a dissipative slip law.</p><p>Nominal reference: <code>Modelica.Mechanics.Rotational.Components.IdealRollingWheel</code>. Activation uses the configured half-open fault window or the optional trigger; inspect <code>effectLevel</code> during recovery.</p></html>"));
end FaultableIdealRollingWheel;
