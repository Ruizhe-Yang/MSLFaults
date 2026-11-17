within MSLFaults.Mechanics.Rotational.Components;
model FaultableClutch "Rotational clutch with separate static and sliding friction faults"
  extends Modelica.Mechanics.Rotational.Icons.Clutch;
  extends
    Modelica.Mechanics.Rotational.Interfaces.PartialCompliantWithRelativeStates;

  parameter Real mu_pos[:, 2]=[0, 0.5]
    "Positive sliding friction coefficient [-] as function of w_rel [rad/s] (w_rel>=0)";
  parameter Real peak(final min=1) = 1
    "Peak for maximum value of mu at w==0 (mu0_max = peak*mu_pos[1,2])";
  parameter Real cgeo(final min=0) = 1
    "Geometry constant containing friction distribution assumption";
  parameter Modelica.Units.SI.Force fn_max(final min=0, start=1) "Maximum normal force";

  extends Modelica.Mechanics.Rotational.Interfaces.PartialFriction;
  extends
    Modelica.Thermal.HeatTransfer.Interfaces.PartialElementaryConditionalHeatPortWithoutT;

  Real mu0 "Friction coefficient for w=0 and forward sliding";
  Modelica.Units.SI.Force fn "Normal force (fn=fn_max*f_normalized)";
  Modelica.Blocks.Interfaces.RealInput f_normalized
    "Normalised force signal 0..1 (normal force = fn_max*f_normalized; clutch is engaged if > 0)"
    annotation (Placement(transformation(
        origin={0,110},
        extent={{20,-20},{-20,20}},
        rotation=90)));

  type FaultMode=enumeration(Normal "Nominal component relation", FrictionLoss "Reduced friction", FrictionIncrease "Increased friction", Slipping "Reduced sliding friction", Sticking "Increased static resistance");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault law";
  parameter Real severity(min=0,max=1)=1 "Scale of the activation envelope";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault request";
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal half-open fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a complete activation or recovery ramp";
  Real faultActivation(min=0,max=1) "Activation envelope used by the fault law";
  Real startActivation(min=0,max=1) "Activation envelope divided by positive severity";
  Real endActivation(min=0,max=1) "One while activation or its recovery envelope remains present";
  parameter Real frictionScaleFault(min=0,max=1)=0.5
    "Default target for frictionLossScale";
  parameter Real frictionLossScale(min=0,max=1)=frictionScaleFault "Friction multiplier at full loss effect";
  parameter Real frictionIncreaseScale(min=1)=2 "Friction multiplier at full increase effect";
  parameter Real slipDynamicScale(min=0,max=1)=0.2 "Sliding-friction multiplier at full slip effect";
  parameter Real stickingStaticScale(min=1)=1000 "Static-friction multiplier at full sticking effect";
  Real dynamicScale "Multiplier on the sliding-friction law";
  Real staticScale "Multiplier on the static-friction threshold";

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

  // Constant auxiliary variable
  mu0 = Modelica.Math.Vectors.interpolate(mu_pos[:,1], mu_pos[:,2], 0, 1);

  // Relative quantities
  w_relfric = w_rel;
  a_relfric = a_rel;

  // Normal force and friction torque for w_rel=0
  dynamicScale=if faultMode==FaultMode.FrictionLoss then 1+faultActivation*(frictionLossScale-1)
    elseif faultMode==FaultMode.FrictionIncrease then 1+faultActivation*(frictionIncreaseScale-1)
    elseif faultMode==FaultMode.Slipping then 1+faultActivation*(slipDynamicScale-1) else 1;
  staticScale=if faultMode==FaultMode.FrictionLoss then 1+faultActivation*(frictionLossScale-1)
    elseif faultMode==FaultMode.FrictionIncrease then 1+faultActivation*(frictionIncreaseScale-1)
    elseif faultMode==FaultMode.Sticking then 1+faultActivation*(stickingStaticScale-1) else 1;
  fn=fn_max*f_normalized;
  free = fn <= 0;
  tau0 = mu0*cgeo*fn*staticScale;
  tau0_max = peak*tau0;

  // Friction torque
  tau = if locked then sa*unitTorque else if free then 0 else cgeo*fn*dynamicScale*(
    if startForward then
      Modelica.Math.Vectors.interpolate(mu_pos[:,1], mu_pos[:,2], w_rel, 1)
    else if startBackward then
      -Modelica.Math.Vectors.interpolate(mu_pos[:,1], mu_pos[:,2], w_rel, 1)
    else if pre(mode) == Forward then
      Modelica.Math.Vectors.interpolate(mu_pos[:,1], mu_pos[:,2], w_rel, 1)
    else
      -Modelica.Math.Vectors.interpolate(mu_pos[:,1], mu_pos[:,2], -w_rel, 1));
  lossPower = tau*w_relfric;


  annotation (Icon(coordinateSystem(preserveAspectRatio=true,
        extent={{-100,-100},{100,100}}),
      graphics={
        Text(extent={{-150,-110},{150,-70}}, textString="%name", textColor={0,0,255}),
        Line(visible=useHeatPort, points={{-100,-100},{-100,-40},{0,-40}}, color={191,0,0}, pattern=LinePattern.Dot, thickness=0.5),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }), Documentation(info="<html><p>Transmit torque according to the relative shaft motion and the Coulomb friction state. FrictionLoss and FrictionIncrease affect static and sliding friction; Slipping changes the sliding scale, while Sticking changes the static threshold.</p><p>Connect the two shafts, supply f_normalized and set fn_max, mu_pos and the selected friction scale.</p><p>The clutch remains free for non-positive normal force. Inspect w_rel, tau and lossPower when distinguishing sliding from a locked friction state.</p><p>Nominal reference: <code>Modelica.Mechanics.Rotational.Components.Clutch</code>. Activation uses the configured half-open fault window or the optional trigger; inspect <code>effectLevel</code> during recovery.</p></html>"));
end FaultableClutch;
