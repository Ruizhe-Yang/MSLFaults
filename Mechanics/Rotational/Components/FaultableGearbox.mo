within MSLFaults.Mechanics.Rotational.Components;
model FaultableGearbox "Elastic gearbox with an additional fault torque"
  extends Modelica.Mechanics.Rotational.Icons.Gearbox;
  extends Modelica.Mechanics.Rotational.Interfaces.PartialTwoFlangesAndSupport;

  parameter Real ratio(start=1)
    "Transmission ratio (flange_a.phi/flange_b.phi)";
  parameter Real lossTable[:, 5]=[0, 1, 1, 0, 0]
    "Array for mesh efficiencies and bearing friction depending on speed (see docu of LossyGear)";
  parameter Modelica.Units.SI.RotationalSpringConstant c(final min=Modelica.Constants.small,
      start=1.0e5) "Gear elasticity (spring constant)";
  parameter Modelica.Units.SI.RotationalDampingConstant d(final min=0, start=0)
    "Gear damping (relative damping)";
  parameter Modelica.Units.SI.Angle b(final min=0) = 0 "Total backlash";
  parameter StateSelect stateSelect=StateSelect.prefer
    "Priority to use phi_rel and w_rel as states"
    annotation (HideResult=true, Dialog(tab="Advanced"));
  extends Modelica.Thermal.HeatTransfer.Interfaces.PartialConditionalHeatPort(
      final T=293.15);
  Modelica.Units.SI.Angle phi_rel(
    start=0,
    stateSelect=stateSelect,
    nominal=1e-4) = flange_b.phi - lossyGear.flange_b.phi
    "Relative rotation angle over gear elasticity (= flange_b.phi - lossyGear.flange_b.phi)";
  Modelica.Units.SI.AngularVelocity w_rel(
    start=0,
    stateSelect=stateSelect) = der(phi_rel)
    "Relative angular velocity over gear elasticity (= der(phi_rel))";
  Modelica.Units.SI.AngularAcceleration a_rel(start=0) = der(w_rel)
    "Relative angular acceleration over gear elasticity (= der(w_rel))";

  Modelica.Mechanics.Rotational.Components.LossyGear lossyGear(
    final ratio=ratio,
    final lossTable=lossTable,
    final useSupport=true,
    final useHeatPort=true) annotation (Placement(transformation(extent={{-60,
            -20},{-20,20}})));
  Modelica.Mechanics.Rotational.Components.ElastoBacklash elastoBacklash(
    final b=b,
    final c=c,
    final phi_rel0=0,
    final d=d,
    final useHeatPort=true) annotation (Placement(transformation(extent={{
            20,-20},{60,20}})));
  type FaultMode=enumeration(Normal "Nominal component relation", ToothDamage "Sinusoidal tooth-related torque", EfficiencyLoss "Additional opposing torque", LockedGear "Declared gear-locking approximation");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault law";
  parameter Real severity(min=0,max=1)=1 "Scale of the activation envelope";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault request";
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal half-open fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a complete activation or recovery ramp";
  Real faultActivation(min=0,max=1) "Activation envelope used by the fault law";
  Real startActivation(min=0,max=1) "Activation envelope divided by positive severity";
  Real endActivation(min=0,max=1) "One while activation or its recovery envelope remains present";
  parameter Modelica.Units.SI.Torque faultTorqueAmplitude=1 "Amplitude of the additional fault torque";
  parameter Modelica.Units.SI.Frequency toothFrequency=20 "Frequency of the prescribed tooth-torque sinusoid";
  parameter Modelica.Units.SI.RotationalDampingConstant lockDamping=1e9 "Finite viscous coefficient used by the locking approximation";
  Modelica.Mechanics.Rotational.Sources.Torque faultTorqueSource(useSupport=true);
  Modelica.Blocks.Sources.RealExpression faultTorqueCommand(y=if faultMode==FaultMode.ToothDamage then faultActivation*faultTorqueAmplitude*sin(2*Modelica.Constants.pi*toothFrequency*time) elseif faultMode==FaultMode.EfficiencyLoss then -faultActivation*faultTorqueAmplitude*sign(der(flange_a.phi)) elseif faultMode==FaultMode.LockedGear then -faultActivation*lockDamping*der(flange_a.phi) else 0);

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
  connect(faultTorqueCommand.y,faultTorqueSource.tau);
  connect(faultTorqueSource.flange,flange_a);
  connect(faultTorqueSource.support,internalSupport);

  connect(flange_a, lossyGear.flange_a) annotation (Line(points={{-100,0},{
          -90,0},{-90,0},{-80,0},{-80,0},{-60,0}}));
  connect(lossyGear.flange_b, elastoBacklash.flange_a) annotation (Line(
        points={{-20,0},{-10,0},{0,0},{20,0}}));
  connect(elastoBacklash.flange_b, flange_b) annotation (Line(points={{60,0},
          {70,0},{70,0},{80,0},{80,0},{100,0}}));
  connect(elastoBacklash.heatPort, internalHeatPort) annotation (Line(
      points={{20,-20},{20,-60},{-100,-60},{-100,-80}}, color={191,0,0}));
  connect(lossyGear.heatPort, internalHeatPort) annotation (Line(
      points={{-60,-20},{-60,-60},{-100,-60},{-100,-80}}, color={191,0,0}));
  connect(lossyGear.support, internalSupport) annotation (Line(
      points={{-40,-20},{-40,-40},{0,-40},{0,-80}}));


  annotation (
    Documentation(info="<html><p>Combine the nominal LossyGear and ElastoBacklash components with a torque source on flange_a. ToothDamage supplies a sinusoid; EfficiencyLoss opposes the absolute input-shaft velocity; LockedGear adds finite viscous resistance to that velocity.</p><p>Configure ratio, lossTable, c, d and b; connect the shafts and support, then select the added torque mode.</p><p>The fault source does not change the native gear ratio or tooth stiffness. Its work is not included in the heat flow from the nominal gear and backlash components.</p><p>Nominal reference: <code>Modelica.Mechanics.Rotational.Components.Gearbox</code>. Activation uses the configured half-open fault window or the optional trigger; inspect <code>effectLevel</code> during recovery.</p></html>"),
       Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
            {100,100}}),
      graphics={
        Text(extent={{-150,155},{150,125}}, textColor={0,0,255}, textString="%name"),
        Text(extent={{-150,70},{150,100}}, textString="ratio=%ratio, c=%c"),
        Line(visible=useHeatPort, points={{-100,-100},{-100,-30},{0,-30}}, color={191,0,0}, pattern=LinePattern.Dot, thickness=0.5),
        Text(extent={{108,94},{139,62}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableGearbox;
