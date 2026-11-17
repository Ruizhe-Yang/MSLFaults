within MSLFaults.Mechanics.Translational.Components;
model FaultableIdealRollingWheel "Ideal rolling wheel with radius and kinematic transmission faults"
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


  annotation(
    Documentation(info="<html><p>RadiusError changes the effective radius. Slip scales the rotational contribution to the position relation by 1-effectLevel*slipRatioFault. LockedWheel uses the scale 1-effectLevel. The torque-force relation uses radius_effective without that scale.</p><p>Connect flangeR and flangeT with appropriate supports; set radiusFault or slipRatioFault and inspect both displacements.</p><p>At full LockedWheel effect the translational displacement relative to support is zero. These modes alter ideal kinematics and do not describe tyre contact or a dissipative slip law.</p><p>Nominal reference: <code>Modelica.Mechanics.Translational.Components.IdealRollingWheel</code>. Activation uses the configured half-open fault window or the optional trigger; inspect <code>effectLevel</code> during recovery.</p></html>"),
    Icon(graphics={
        Ellipse(extent={{-61,-54},{61,68}}, lineColor={255,0,0}, fillColor={255,255,255}, fillPattern=FillPattern.None, pattern=LinePattern.Solid, lineThickness=0.85),
        Ellipse(extent={{-8,-1},{8,15}}, lineColor={64,64,64}, fillColor={255,255,255}, fillPattern=FillPattern.None, pattern=LinePattern.Solid, lineThickness=0.6),
        Line(points={{-100,0},{-78,0},{-78,7},{-61,7}}, color={64,64,64}, thickness=0.8, pattern=LinePattern.Solid),
        Line(points={{-75,-59},{91,-59}}, color={64,64,64}, thickness=0.85, pattern=LinePattern.Solid),
        Line(points={{61,7},{80,7},{80,0},{100,0}}, color={64,64,64}, thickness=0.8, pattern=LinePattern.Solid),
        Line(points={{-17,85},{31,85}}, color={64,64,64}, thickness=0.6, pattern=LinePattern.Solid, arrow={Arrow.None,Arrow.Filled}),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}, fontSize=0),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableIdealRollingWheel;
