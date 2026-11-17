within MSLFaults.Mechanics.Rotational.Components;
model FaultableDamper "Rotational damper with bounded damping changes and progressive leakage"
  extends
    Modelica.Mechanics.Rotational.Interfaces.PartialCompliantWithRelativeStates;
  parameter Modelica.Units.SI.RotationalDampingConstant d(final min=0, start=0)
    "Damping constant";
  extends
    Modelica.Thermal.HeatTransfer.Interfaces.PartialElementaryConditionalHeatPortWithoutT;
  type FaultMode=enumeration(Normal "Nominal component relation", DampingLoss "Reduced damping", DampingIncrease "Configured damping increase", Leakage "Leakage-related damping change", Seizure "Increased finite mechanical resistance");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault law";
  parameter Real severity(min=0,max=1)=1 "Scale of the activation envelope";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault request";
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal half-open fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a complete activation or recovery ramp";
  Real faultActivation(min=0,max=1) "Activation envelope used by the fault law";
  Real startActivation(min=0,max=1) "Activation envelope divided by positive severity";
  Real endActivation(min=0,max=1) "One while activation or its recovery envelope remains present";
  parameter Modelica.Units.SI.RotationalDampingConstant dFault=0.5*d
    "Default target for dLoss";
  parameter Modelica.Units.SI.RotationalDampingConstant dLoss=dFault "Damping at full loss effect";
  parameter Modelica.Units.SI.RotationalDampingConstant dIncrease=2*d "Damping at full increase effect";
  parameter Modelica.Units.SI.RotationalDampingConstant dLeak=0.1*d "Damping at completed leakage progression";
  parameter Modelica.Units.SI.Time leakageTime(min=Modelica.Constants.small)=1 "Elapsed activation duration to complete leakage progression";
  parameter Modelica.Units.SI.RotationalDampingConstant dSeized=1e9 "Finite damping at full seizure effect";
  Modelica.Units.SI.RotationalDampingConstant d_effective "Damping used by the constitutive relation";
  Real leakageProgress(min=0,max=1) "Bounded progression based on elapsed activation time";

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

  leakageProgress=min(1,max(0,faultGate.elapsedActiveTime/leakageTime));
  d_effective=if faultMode==FaultMode.DampingLoss then d+faultActivation*(dLoss-d)
    elseif faultMode==FaultMode.DampingIncrease then d+faultActivation*(dIncrease-d)
    elseif faultMode==FaultMode.Leakage then d+effectLevel*leakageProgress*(dLeak-d)
    elseif faultMode==FaultMode.Seizure then d+faultActivation*(dSeized-d) else d;
  tau=d_effective*w_rel;
  lossPower = tau*w_rel;

  assert(d_effective>=0,"A passive damper requires nonnegative damping.");
  annotation (
    Documentation(info="<html><p>The torque is d_effective*w_rel. DampingLoss, DampingIncrease and Seizure interpolate towards their respective targets. Leakage additionally uses elapsed activation time divided by leakageTime. lossPower reports viscous dissipation.</p><p>Connect two rotational flanges and set dLoss, dIncrease, dLeak or dSeized for the selected mode.</p><p>Leakage recovers with the effect envelope; Seizure is large finite damping rather than a rigid constraint. All effective damping must remain non-negative.</p><p>Nominal reference: <code>Modelica.Mechanics.Rotational.Components.Damper</code>. Activation uses the configured half-open fault window or the optional trigger; inspect <code>effectLevel</code> during recovery.</p></html>"),
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
      graphics={
        Line(points={{-90,0},{-60,0}}, color={64,64,64}, thickness=0.5),
        Line(points={{-60,-30},{-60,30}}, color={64,64,64}, thickness=0.5),
        Line(points={{-60,-30},{60,-30}}, color={64,64,64}, thickness=0.5),
        Line(points={{-60,30},{60,30}}, color={64,64,64}, thickness=0.5),
        Rectangle(extent={{-60,30},{30,-30}}, fillColor={192,192,192}, fillPattern=FillPattern.Solid, lineColor={64,64,64}, lineThickness=0.5),
        Line(points={{30,0},{90,0}}, color={64,64,64}, thickness=0.5),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Text(extent={{-150,-50},{150,-90}}, textString="d=%d"),
        Line(visible=useHeatPort, points={{-100,-100},{-100,-40},{-20,-40},{-20,0}}, color={191,0,0}, pattern=LinePattern.Dot, thickness=0.5),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableDamper;
