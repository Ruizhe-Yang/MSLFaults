within MSLFaults.Mechanics.Rotational.Components;
model FaultableLossyGear
  "Gear with nominal mesh losses and an additional fault torque"

  extends Modelica.Mechanics.Rotational.Icons.Gear;
  extends Modelica.Mechanics.Rotational.Interfaces.PartialElementaryTwoFlangesAndSupport2;

  parameter Real ratio(start=1)
    "Transmission ratio (flange_a.phi/flange_b.phi)";
  parameter Real lossTable[:, 5]=[0, 1, 1, 0, 0]
    "Array for mesh efficiencies and bearing friction depending on speed";
  extends Modelica.Thermal.HeatTransfer.Interfaces.PartialElementaryConditionalHeatPortWithoutT;
  Modelica.Units.SI.Angle phi_a
    "Angle between left shaft flange and support";
  Modelica.Units.SI.Angle phi_b
    "Angle between right shaft flange and support";

  Real sa(final unit="1") "Path parameter for acceleration and torque loss";
  Modelica.Units.SI.AngularVelocity w_a
    "Angular velocity of flange_a with respect to support";
  Modelica.Units.SI.AngularAcceleration a_a
    "Angular acceleration of flange_a with respect to support";

  Real interpolation_result[1, 4]
    "Result of interpolation in lossTable (= [eta_mf1, eta_mf2, tau_bf1, tau_bf2])";
  Real eta_mf1(unit="1") "Mesh efficiency in case that flange_a is driving";
  Real eta_mf2(unit="1") "Mesh efficiency in case that flange_b is driving";
  Modelica.Units.SI.Torque tau_bf_a "Bearing friction torque on flange_a side";
  Modelica.Units.SI.Torque tau_eta
    "Torque that determines the driving side (= if forwardSliding then flange_a.tau-tau_bf_a else if backwardSliding then flange_a.tau+tau_bf_a else flange_a.tau)";

  Modelica.Units.SI.Torque tau_bf1
    "Absolute resultant bearing friction torque with respect to flange_a in case that flange_a is driving (= |tau_bf_a*eta_mf1 + tau_bf_b/i|)";
  Modelica.Units.SI.Torque tau_bf2
    "Absolute resultant bearing friction torque with respect to flange_a in case that flange_b is driving (= |tau_bf_a/eta_mf2 + tau_bf_b/i|)";

  Modelica.Units.SI.Torque quadrant1 "Torque loss if w_a > 0 and flange_a.tau >= 0";
  Modelica.Units.SI.Torque quadrant2 "Torque loss if w_a > 0 and flange_a.tau < 0";
  Modelica.Units.SI.Torque quadrant3 "Torque loss if w_a < 0 and flange_a.tau >= 0";
  Modelica.Units.SI.Torque quadrant4 "Torque loss if w_a < 0 and flange_a.tau < 0";

  // Resultant friction torques at quadrant values for angular velocities near zero
  Modelica.Units.SI.Torque quadrant1_p
    "Torque loss at w_a = 0+ to determine driving side (flange_a.tau >= 0)";
  Modelica.Units.SI.Torque quadrant2_p
    "Torque loss at w_a = 0+ to determine driving side (flange_a.tau < 0)";
  Modelica.Units.SI.Torque quadrant3_m
    "Torque loss at w_a = 0- to determine driving side (flange_a.tau >=0)";
  Modelica.Units.SI.Torque quadrant4_m
    "Torque loss at w_a = 0- to determine driving side (flange_a.tau < 0)";

  Modelica.Units.SI.Torque tauLoss
    "Torque loss due to friction in the gear teeth and in the bearings";
  Modelica.Units.SI.Torque tauLossMax "Torque loss for positive speed";
  Modelica.Units.SI.Torque tauLossMin "Torque loss for negative speed";

  Modelica.Units.SI.Torque tauLossMax_p "Torque loss for positive speed";
  Modelica.Units.SI.Torque tauLossMin_m "Torque loss for negative speed";

  Boolean tau_aPos(start=true)
    "Alias of the inferred driving-torque sign";
  Boolean tau_etaPos(start=true) "= true, if torque tau_eta is not negative";
  Boolean startForward(start=false) "= true, if starting to roll forward";
  Boolean startBackward(start=false) "= true, if starting to roll backward";
  Boolean locked(start=false) "= true, if gear is locked";

  Boolean ideal
    "= true, if losses are neglected (that is lossTable = [0, 1, 1, 0, 0])";

  constant Integer Unknown=3 "Value of mode is not known";
  constant Integer Free=2 "Element is not active";
  constant Integer Forward=1 "w_a > 0 (forward rolling)";
  constant Integer Stuck=0
    "w_a = 0 (forward rolling, locked or backward rolling)";
  constant Integer Backward=-1 "w_a < 0 (backward rolling)";
  Integer mode(
    final min=Backward,
    final max=Unknown,
    start=Free,
    fixed=true)
    "Mode of friction element (unknown, not active, forward/backward rolling, stuck)";

  Modelica.Units.SI.Torque tau_eta_p "tau_eta assuming positive omega";
  Modelica.Units.SI.Torque tau_eta_m "tau_eta assuming negative omega";
  type FaultMode=enumeration(Normal "Nominal component relation", GearRatioError "Changed kinematic transmission ratio", EfficiencyLoss "Additional opposing torque", ToothDamage "Sinusoidal tooth-related torque", LockedGear "Declared gear-locking approximation");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault law";
  parameter Real severity(min=0,max=1)=1 "Scale of the activation envelope";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault request";
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal half-open fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a complete activation or recovery ramp";
  Real faultActivation(min=0,max=1) "Activation envelope used by the fault law";
  Real startActivation(min=0,max=1) "Activation envelope divided by positive severity";
  Real endActivation(min=0,max=1) "One while activation or its recovery envelope remains present";
  parameter Real ratioFault=0.9*ratio "Transmission ratio at full ratio-error effect";
  parameter Modelica.Units.SI.Torque lossTorqueFault=1 "Amplitude of the additional gear-loss or tooth torque";
  parameter Modelica.Units.SI.RotationalDampingConstant lockDamping=1e9 "Finite viscous coefficient used by the locking approximation";
  parameter Modelica.Units.SI.Frequency toothFrequency=20 "Frequency of the prescribed tooth-torque sinusoid";
  parameter Modelica.Units.SI.AngularVelocity wRegularization=1e-3
    "Speed scale for smooth efficiency-loss torque reversal";
  Real ratio_effective "Transmission ratio in the current kinematic relation";
  Modelica.Units.SI.Torque addedLossTorque "Additional torque beyond the nominal gear-loss law";
protected
  constant Modelica.Units.SI.AngularAcceleration unitAngularAcceleration=1;
  constant Modelica.Units.SI.Torque unitTorque=1;

  // get friction and eta information for omega=0
  parameter Real eta_mf1_0(unit="1")=Modelica.Math.Vectors.interpolate(lossTable[:,1], lossTable[:,2], 0, 1);
  parameter Real eta_mf2_0(unit="1")=Modelica.Math.Vectors.interpolate(lossTable[:,1], lossTable[:,3], 0, 1);
  parameter Modelica.Units.SI.Torque tau_bf1_0=abs(Modelica.Math.Vectors.interpolate(lossTable[:,1], lossTable[:,4], 0, 1));
  parameter Modelica.Units.SI.Torque tau_bf2_0=abs(Modelica.Math.Vectors.interpolate(lossTable[:,1], lossTable[:,5], 0, 1));
  parameter Modelica.Units.SI.Torque tau_bf_a_0=if Modelica.Math.isEqual(
          eta_mf1_0,
          1.0,
          Modelica.Constants.eps) and Modelica.Math.isEqual(
          eta_mf2_0,
          1.0,
          Modelica.Constants.eps) then tau_bf1_0/2 else (tau_bf1_0 -
      tau_bf2_0)/(eta_mf1_0 - 1.0/eta_mf2_0);
  // For eta_mf1_0=eta_mf2_0=1 the given bearing
  // friction can not be separated into a part
  // on side A or B, so it is done arbitrarily.
  // Calculate tau_bf_a_0 from the following equation
  // Friction separation equations are implemented below.
  //  tau_bf1_0=eta_mf1_0*tau_bf_a_0 + 1/ratio a_0
  //  tau_bf2_0=1/eta_mf2*tau_bf_a_0 + 1/ratio tau_bf_a_0

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
  assert(abs(ratio) > 0,
    "Error in initialization of LossyGear: ratio may not be zero");

  ideal = Modelica.Math.Matrices.isEqual(
        lossTable,
        [0, 1, 1, 0, 0],
        Modelica.Constants.eps);

  if ideal then
    interpolation_result = [1, 1, 0, 0];
    eta_mf1 = 1;
    eta_mf2 = 1;
    tau_bf1 = 0;
    tau_bf2 = 0;
  else
    interpolation_result = [
      Modelica.Math.Vectors.interpolate(lossTable[:,1], lossTable[:,2], noEvent(abs(w_a)), 1),
      Modelica.Math.Vectors.interpolate(lossTable[:,1], lossTable[:,3], noEvent(abs(w_a)), 1),
      Modelica.Math.Vectors.interpolate(lossTable[:,1], lossTable[:,4], noEvent(abs(w_a)), 1),
      Modelica.Math.Vectors.interpolate(lossTable[:,1], lossTable[:,5], noEvent(abs(w_a)), 1)];
    eta_mf1 = interpolation_result[1, 1];
    eta_mf2 = interpolation_result[1, 2];
    tau_bf1 = noEvent(abs(interpolation_result[1, 3]));
    tau_bf2 = noEvent(abs(interpolation_result[1, 4]));
  end if;

  if Modelica.Math.isEqual(
          eta_mf1,
          1.0,
          Modelica.Constants.eps) and Modelica.Math.isEqual(
          eta_mf2,
          1.0,
          Modelica.Constants.eps) then
    // For eta_mf1=eta_mf2=1 the given bearing friction can not be
    // separated into a part on side A or B, so it is done arbitrarily.
    tau_bf_a = tau_bf1/2;
  else
    //calculate tau_bf_a from the following equations
    //tau_bf1 = eta_mf1*tau_bf_a + tau_bf_b / ratio
    //tau_bf2 = 1/eta_mf2*tau_bf_a + tau_bf_b / ratio
    tau_bf_a = (tau_bf1 - tau_bf2)/(eta_mf1 - 1.0/eta_mf2);
  end if;

  phi_a = flange_a.phi - phi_support;
  phi_b = flange_b.phi - phi_support;
  ratio_effective=if faultMode==FaultMode.GearRatioError then ratio+faultActivation*(ratioFault-ratio) else ratio;
  addedLossTorque=if faultMode==FaultMode.EfficiencyLoss then
      faultActivation*lossTorqueFault*tanh(w_a/wRegularization)
    elseif faultMode==FaultMode.ToothDamage then faultActivation*lossTorqueFault*sin(2*Modelica.Constants.pi*toothFrequency*time)
    elseif faultMode==FaultMode.LockedGear then faultActivation*lockDamping*w_a else 0;
  phi_a=ratio_effective*phi_b;

  // Torque balance (no inertias)
  0=flange_b.tau+ratio_effective*(flange_a.tau-tauLoss-addedLossTorque);

  // Speed for friction element
  w_a = der(phi_a);
  a_a = der(w_a);

  // Determine driving side
  //assuming positive omega
  tau_eta_p = flange_a.tau - tau_bf_a_0;
  //assuming negative omega
  tau_eta_m = flange_a.tau + tau_bf_a_0;

  // assuming w>=0, take value at w=0 to decide rolling/stuck mode
  quadrant1_p = (1 - eta_mf1_0)*flange_a.tau + tau_bf1_0;
  quadrant2_p = (1 - 1/eta_mf2_0)*flange_a.tau + tau_bf2_0;
  tauLossMax_p = if noEvent(tau_eta_p > 0) then quadrant1_p else
    quadrant2_p;

  // assuming w<=0, take value at w=0 to decide rolling/stuck mode
  quadrant4_m = (1 - 1/eta_mf2_0)*flange_a.tau - tau_bf2_0;
  quadrant3_m = (1 - eta_mf1_0)*flange_a.tau - tau_bf1_0;
  tauLossMin_m = if noEvent(tau_eta_m > 0) then quadrant4_m else
    quadrant3_m;

  quadrant1 = (1 - eta_mf1)*flange_a.tau + tau_bf1;
  quadrant2 = (1 - 1/eta_mf2)*flange_a.tau + tau_bf2;
  quadrant4 = (1 - 1/eta_mf2)*flange_a.tau - tau_bf2;
  quadrant3 = (1 - eta_mf1)*flange_a.tau - tau_bf1;

  //tau eta: only for determination of driving side for calculation of tauloss
  tau_eta = if ideal then flange_a.tau else (if locked then flange_a.tau
     else (if (startForward or pre(mode) == Forward) then flange_a.tau -
    tau_bf_a else flange_a.tau + tau_bf_a));

  // Torque Losses
  tau_etaPos = tau_eta >= 0;
  tau_aPos = tau_etaPos;
  tauLossMax = if tau_etaPos then quadrant1 else quadrant2;
  tauLossMin = if tau_etaPos then quadrant4 else quadrant3;

  // Determine rolling/stuck mode when w_rel = 0
  startForward = pre(mode) == Stuck and sa > tauLossMax_p/unitTorque or
    initial() and w_a > 0;
  startBackward = pre(mode) == Stuck and sa < tauLossMin_m/unitTorque or
    initial() and w_a < 0;
  locked = not (ideal or pre(mode) == Forward or startForward or pre(mode)
     == Backward or startBackward);

  /* Parametrised curve description a_a = f1(sa), tauLoss = f2(sa)
       In comparison to Modelica.Mechanics.Rotational.FrictionBase it is possible
       to simplify the following expression as mode == Stuck is assured in case
       of startForward or startBackward */
  tauLoss = if ideal then 0 else (if locked then sa*unitTorque else (if (
    startForward or pre(mode) == Forward) then tauLossMax else tauLossMin));

  a_a = unitAngularAcceleration*(if locked then 0 else sa - tauLoss/
    unitTorque);

  /* Finite state machine to fix configuration after the computation above
       The above equations are only dependent on pre(mode) and not on the actual
       value of mode. This prevents loops. So mode can be determined in one step. */
  mode = if ideal then Free else (if (pre(mode) == Forward or startForward)
     and w_a > 0 then Forward else if (pre(mode) == Backward or
    startBackward) and w_a < 0 then Backward else Stuck);

  lossPower = tauLoss*w_a;


  annotation (
    Documentation(info="<html><p>GearRatioError changes the ideal angular relation. EfficiencyLoss adds a torque with tanh-regularised speed reversal; ToothDamage adds a sinusoidal torque; LockedGear adds viscous torque. The nominal mesh-efficiency and bearing-friction state machine remains in use.</p><p>Set ratio and lossTable, then select ratioFault, lossTorqueFault or lockDamping; inspect addedLossTorque separately from lossPower.</p><p>lossPower contains only tauLoss*w_a, so additional fault-torque work is not included in that output or its heat port. Keep wRegularization positive.</p><p>Nominal reference: <code>Modelica.Mechanics.Rotational.Components.LossyGear</code>. Activation uses the configured half-open fault window or the optional trigger; inspect <code>effectLevel</code> during recovery.</p></html>"),
       Icon(coordinateSystem(preserveAspectRatio=true,
        extent={{-100.0,-100.0},{100.0,100.0}}),
      graphics={
        Polygon(fillColor={161,35,41}, pattern=LinePattern.None, fillPattern=FillPattern.Solid, points={{-110,50},{-80,50},{-80,80},{-90,80},{-70,100},{-50,80},{-60,80},{-60,30},{-110,30},{-110,50}}, lineColor={64,64,64}, lineThickness=0.5),
        Line(points={{-80,20},{-60,20}}, color={64,64,64}, thickness=0.5),
        Text(textColor={0,0,255}, extent={{-148,155},{152,125}}, textString="%name"),
        Text(extent={{-145,-79},{155,-49}}, textString="ratio=%ratio"),
        Line(visible=useHeatPort, points={{-100,-100},{-100,-30},{0,-30},{0,0}}, color={191,0,0}, pattern=LinePattern.Dot, thickness=0.5),
        Text(extent={{108,94},{139,62}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableLossyGear;
