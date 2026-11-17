within MSLFaults.Mechanics.Rotational.Components;
model FaultableOneWayClutch "One-way clutch with a fault-adjusted friction level"
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
  parameter Modelica.Units.SI.AngularVelocity w_small=1e10
    "Relative angular velocity near to zero if jumps due to a reinit(..) of the velocity can occur (set to low value only if such impulses can occur)"
    annotation (Dialog(tab="Advanced"));
  extends
    Modelica.Thermal.HeatTransfer.Interfaces.PartialElementaryConditionalHeatPortWithoutT;

  Real u "Normalised force input signal (0..1)";
  Modelica.Units.SI.Force fn "Normal force (fn=fn_max*inPort.signal)";
  Boolean startForward(start=false)
    "= true, if w_rel=0 and start of forward sliding or w_rel > w_small";
  Boolean locked(start=false) "= true, if w_rel=0 and not sliding";
  Boolean stuck(start=false) "w_rel=0 (locked or start forward sliding)";

  type FaultMode=enumeration(Normal "Nominal component relation", FrictionLoss "Reduced friction", FrictionIncrease "Increased friction", Slipping "Reduced sliding friction", Sticking "Increased static resistance");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault law";
  parameter Real severity(min=0,max=1)=1 "Scale of the activation envelope";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault request";
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal half-open fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a complete activation or recovery ramp";
  Real faultActivation(min=0,max=1) "Activation envelope used by the fault law";
  Real startActivation(min=0,max=1) "Activation envelope divided by positive severity";
  Real endActivation(min=0,max=1) "One while activation or its recovery envelope remains present";
  parameter Real frictionScaleFault(min=0)=0.5 "Normal-force multiplier at full friction-loss or slip effect";
  parameter Real stickingScale(min=1)=1000 "Friction multiplier at full sticking effect";
  Real frictionScale "Effective friction multiplier";
protected
  Modelica.Units.SI.Torque tau0 "Friction torque for w=0 and sliding";
  Modelica.Units.SI.Torque tau0_max "Maximum friction torque for w=0 and locked";
  Real mu0 "Friction coefficient for w=0 and sliding";
  Boolean free "= true, if frictional element is not active";
  Real sa(final unit="1")
    "Path parameter of tau = f(a_rel) Friction characteristic";
  constant Real eps0=1.0e-4 "Relative hysteresis epsilon";
  Modelica.Units.SI.Torque tau0_max_low "Lowest value for tau0_max";
  parameter Real peak2=max([peak, 1 + eps0]);
  constant Modelica.Units.SI.AngularAcceleration unitAngularAcceleration=1;
  constant Modelica.Units.SI.Torque unitTorque=1;
public
  Modelica.Blocks.Interfaces.RealInput f_normalized
    "Normalised force signal 0..1 (normal force = fn_max*f_normalized; clutch is engaged if > 0)"
    annotation (Placement(transformation(
        origin={0,110},
        extent={{20,-20},{-20,20}},
        rotation=90)));


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
  tau0_max_low = eps0*mu0*cgeo*fn_max;

  // Normal force and friction torque for w_rel=0
  u = f_normalized;
  free = u <= 0;
  frictionScale=if faultMode==FaultMode.FrictionLoss or faultMode==FaultMode.Slipping then 1+faultActivation*(frictionScaleFault-1)
    elseif faultMode==FaultMode.FrictionIncrease then 1+faultActivation
    elseif faultMode==FaultMode.Sticking then 1+faultActivation*(stickingScale-1) else 1;
  fn=if free then 0 else fn_max*u*frictionScale;
  tau0 = mu0*cgeo*fn;
  tau0_max = if free then tau0_max_low else peak2*tau0;

  /* Friction characteristic
       (locked is introduced to help the Modelica translator determining
       the different structural configurations, if for each configuration
       special code shall be generated)
    */
  startForward = pre(stuck) and (sa > tau0_max/unitTorque or pre(
    startForward) and sa > tau0/unitTorque or w_rel > w_small) or initial()
     and (w_rel > 0);
  locked = pre(stuck) and not startForward;

  // acceleration and friction torque
  a_rel = unitAngularAcceleration*(if locked then 0 else sa - tau0/
    unitTorque);
  tau = if locked then sa*unitTorque else (if free then 0 else cgeo*fn*
    Modelica.Math.Vectors.interpolate(mu_pos[:,1], mu_pos[:,2], w_rel, 1));

  // Determine configuration
  stuck = locked or w_rel <= 0;

  lossPower = if stuck then 0 else tau*w_rel;


  annotation (
    Icon(coordinateSystem(preserveAspectRatio=true,
          extent={{-100,-100},{100,100}}),
      graphics={
        Text(extent={{-150,-110},{150,-70}}, textString="%name", textColor={0,0,255}),
        Polygon(points={{-10,30},{50,0},{-10,-30},{-10,30}}, fillPattern=FillPattern.Solid, lineColor={64,64,64}, lineThickness=0.5),
        Line(visible=useHeatPort, points={{-100,-99},{-100,-40},{0,-40}}, color={191,0,0}, pattern=LinePattern.Dot, thickness=0.5),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }), Documentation(info="<html><p>The freewheel and clutch state machine uses a scaled normal force. FrictionLoss and Slipping share frictionScaleFault; FrictionIncrease uses the factor 1+effectLevel; Sticking uses stickingScale.</p><p>Connect the two shafts, supply f_normalized and set the friction table and relevant scale; inspect w_rel and tau.</p><p>The nominal one-way constraint is retained. A non-positive input releases the frictional clutch contribution; Sticking scales friction rather than introducing a separate rigid lock.</p><p>Nominal reference: <code>Modelica.Mechanics.Rotational.Components.OneWayClutch</code>. Activation uses the configured half-open fault window or the optional trigger; inspect <code>effectLevel</code> during recovery.</p></html>"));
end FaultableOneWayClutch;
