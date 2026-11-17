within MSLFaults.Mechanics.Translational.Components;
model FaultableMassWithStopAndFriction
  "Sliding mass with friction faults and movable hard stops"
  extends PartialFrictionWithStop;
  Modelica.Units.SI.Velocity v(start=0, stateSelect=StateSelect.always)
    "Absolute velocity of flange_a and flange_b";
  Modelica.Units.SI.Acceleration a(start=0)
    "Absolute acceleration of flange_a and flange_b";
  parameter Modelica.Units.SI.Mass m(start=1) "Mass";
  parameter Real F_prop(
    final unit="N.s/m",
    final min=0,
    start=1) "Velocity dependent friction";
  parameter Modelica.Units.SI.Force F_Coulomb(start=5)
    "Constant friction: Coulomb force";
  parameter Modelica.Units.SI.Force F_Stribeck(start=10) "Stribeck effect";
  parameter Real fexp(
    final unit="s/m",
    final min=0,
    start=2) "Exponential decay";
  extends Modelica.Thermal.HeatTransfer.Interfaces.PartialElementaryConditionalHeatPortWithoutT;
  Integer stopped
    "Mode of stop (-1: hard stop at flange_a, 0: no stop, +1: hard stop at flange_b";
  type FaultMode=enumeration(Normal "Nominal component relation", MassDrift "Changed effective mass", FrictionLoss "Reduced friction", FrictionIncrease "Increased friction", StopShift "Translated hard-stop locations", Seizure "Increased finite mechanical resistance");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault law";
  parameter Real severity(min=0,max=1)=1 "Scale of the activation envelope";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault request";
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal half-open fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a complete activation or recovery ramp";
  Real faultActivation(min=0,max=1) "Activation envelope used by the fault law";
  Real startActivation(min=0,max=1) "Activation envelope divided by positive severity";
  Real endActivation(min=0,max=1) "One while activation or its recovery envelope remains present";
  parameter Modelica.Units.SI.Mass mFault=0.5*m "Mass at full parameter-change effect";
  parameter Real frictionScaleFault(min=0)=2 "Friction multiplier at full configured increase effect";
  parameter Modelica.Units.SI.Position stopShift=0.1 "Offset applied to both hard-stop locations at full effect";
  Modelica.Units.SI.Mass m_effective "Effective mass used by the force relation";
  Real frictionScale "Effective friction multiplier";
  encapsulated partial model PartialFrictionWithStop
    "Base model of Coulomb friction elements with stop"

    import Modelica;
    import Modelica.Mechanics.Translational.Interfaces.PartialRigid;
    parameter Modelica.Units.SI.Position smax(start=25)
      "Right stop for (right end of) sliding mass";
    parameter Modelica.Units.SI.Position smin(start=-25)
      "Left stop for (left end of) sliding mass";
    Modelica.Units.SI.Position smax_effective
      "Fault-adjusted right stop used by the hybrid contact state machine";
    Modelica.Units.SI.Position smin_effective
      "Fault-adjusted left stop used by the hybrid contact state machine";
    parameter Modelica.Units.SI.Velocity v_small=1e-3
      "Relative velocity near to zero (see model info text)"
      annotation (Dialog(tab="Advanced"));
    // Equations to define the following variables have to be defined in subclasses
    Modelica.Units.SI.Velocity v_relfric "Relative velocity between frictional surfaces";
    Modelica.Units.SI.Acceleration a_relfric
      "Relative acceleration between frictional surfaces";
    Modelica.Units.SI.Force f
      "Friction force (positive, if directed in opposite direction of v_rel)";
    Modelica.Units.SI.Force f0 "Friction force for v=0 and forward sliding";
    Modelica.Units.SI.Force f0_max "Maximum friction force for v=0 and locked";
    Boolean free "= true, if frictional element is not active";
    // Equations to define the following variables are given in this class
    Real sa(unit="1")
      "Path parameter of friction characteristic f = f(a_relfric)";
    Boolean startForward(start=false, fixed=true)
      "= true, if v_rel=0 and start of forward sliding or v_rel > v_small";
    Boolean startBackward(start=false, fixed=true)
      "= true, if v_rel=0 and start of backward sliding or v_rel < -v_small";
    Boolean locked(start=false) "= true, if v_rel=0 and not sliding";
    extends PartialRigid(s(start=0, stateSelect=StateSelect.always));
    constant Integer Unknown=3 "Value of mode is not known";
    constant Integer Free=2 "Element is not active";
    constant Integer Forward=1 "v_rel > 0 (forward sliding)";
    constant Integer Stuck=0
      "v_rel = 0 (forward sliding, locked or backward sliding)";
    constant Integer Backward=-1 "v_rel < 0 (backward sliding)";
    Integer mode(
      final min=Backward,
      final max=Unknown,
      start=Unknown,
      fixed=true)
      "Mode of friction (-1: backward sliding, 0: stuck, 1: forward sliding, 2: inactive, 3: unknown)";
  protected
    constant Modelica.Units.SI.Acceleration unitAcceleration=1 annotation (HideResult=true);
    constant Modelica.Units.SI.Force unitForce=1 annotation (HideResult=true);
  equation
    /* Friction characteristic
   (locked is introduced to help the Modelica translator determining
   the different structural configurations,
   if for each configuration special code shall be generated)
*/
    startForward = pre(mode) == Stuck and (sa > f0_max/unitForce and s < (
      smax_effective - L/2) or pre(startForward) and sa > f0/unitForce and s < (smax_effective
       - L/2)) or pre(mode) == Backward and v_relfric > v_small or initial()
       and (v_relfric > 0);
    startBackward = pre(mode) == Stuck and (sa < -f0_max/unitForce and s >
      (smin_effective + L/2) or pre(startBackward) and sa < -f0/unitForce and s > (
      smin_effective + L/2)) or pre(mode) == Forward and v_relfric < -v_small or
      initial() and (v_relfric < 0);
    locked = not free and not (pre(mode) == Forward or startForward or pre(
      mode) == Backward or startBackward);

    a_relfric/unitAcceleration = if locked then 0 else if free then sa
       else if startForward then sa - f0_max/unitForce else if
      startBackward then sa + f0_max/unitForce else if pre(mode) == Forward
       then sa - f0_max/unitForce else sa + f0_max/unitForce;

    /* Friction torque has to be defined in a subclass. Example for a clutch:
   f = if locked then sa else
       if free then   0 else
       cgeo*fn*(if startForward then          Modelica.Math.Vectors.interpolate(mu_pos[:,1], mu_pos[:,2], v_relfric, 1) else
                if startBackward then        -Modelica.Math.Vectors.interpolate(mu_pos[:,1], mu_pos[:,2], -v_relfric, 1) else
                if pre(mode) == Forward then  Modelica.Math.Vectors.interpolate(mu_pos[:,1], mu_pos[:,2], v_relfric, 1) else
                                             -Modelica.Math.Vectors.interpolate(mu_pos[:,1], mu_pos[:,2], -v_relfric, 1));
*/
    // finite state machine to determine configuration
    mode = if free then Free else (if (pre(mode) == Forward or pre(mode)
       == Free or startForward) and v_relfric > 0 and s < (smax_effective - L/2)
       then Forward else if (pre(mode) == Backward or pre(mode) == Free or
      startBackward) and v_relfric < 0 and s > (smin_effective + L/2) then Backward
       else Stuck);
    annotation (Documentation(info="<html><p>Internal Coulomb-friction and hard-stop state machine for the surrounding sliding-mass model. Subclasses supply the force law and effective stop positions. The state transitions distinguish forward motion, backward motion and sticking at the configured boundaries.</p></html>"));
  end PartialFrictionWithStop;

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
  smin_effective=if faultMode==FaultMode.StopShift then smin+faultActivation*stopShift else smin;
  smax_effective=if faultMode==FaultMode.StopShift then smax+faultActivation*stopShift else smax;
  m_effective=if faultMode==FaultMode.MassDrift then m+faultActivation*(mFault-m) else m;
  frictionScale=if faultMode==FaultMode.FrictionLoss then 1-faultActivation elseif faultMode==FaultMode.FrictionIncrease then 1+faultActivation*(frictionScaleFault-1) elseif faultMode==FaultMode.Seizure then 1+faultActivation*1000 else 1;

  // Constant auxiliary variables
  f0=frictionScale*(F_Coulomb+F_Stribeck);
  f0_max = f0*1.001;
  free = f0 <= 0 and F_prop <= 0 and s > smin_effective + L/2 and s < smax_effective - L/2;
  // Velocity and acceleration of flanges
  v = der(s);
  a = der(v);
  v_relfric = v;
  a_relfric = a;
  // Equilibrium of forces
  0=flange_a.f+flange_b.f-f-m_effective*der(v);
  // Friction force
  f = if locked then sa*unitForce else if free then 0 else (if startForward
     then frictionScale*(F_prop*v + F_Coulomb + F_Stribeck) else if startBackward then
    frictionScale*(F_prop*v - F_Coulomb - F_Stribeck) else if pre(mode) == Forward then
    frictionScale*(F_prop*v + F_Coulomb + F_Stribeck*Modelica.Math.exp(-fexp*abs(v))) else frictionScale*(F_prop*v -
    F_Coulomb - F_Stribeck*Modelica.Math.exp(-fexp*abs(v))));
  lossPower = f*v_relfric;
  when (initial()) then
    assert(s > smin_effective + L/2 or s >= smin_effective + L/2 and v >= 0,
      "Error in initialization of hard stop. (s - L/2) must be >= smin\n"
       + "(s=" + String(s) + ", L=" + String(L) + ", smin=" + String(smin)
       + ")");
    assert(s < smax_effective - L/2 or s <= smax_effective - L/2 and v <= 0,
      "Error in initialization of hard stop. (s + L/2) must be <= smax\n"
       + "(s=" + String(s) + ", L=" + String(L) + ", smax=" + String(smax)
       + ")");
  end when;

  // Define events for hard stops and reinitialize the state variables velocity v and position s
  stopped = if s <= smin_effective + L/2 then -1 else if s >= smax_effective - L/2 then +1
     else 0;
  when stopped <> 0 then
    reinit(s, if stopped < 0 then smin_effective + L/2 else smax_effective - L/2);
    reinit(v, 0);
  end when;
  /*
Version 1:

  when not (s < smax - L/2) then
    reinit(s, smax - L/2);
    if (not initial() or v>0) then
      reinit(v, 0);
    end if;
  end when;

  when not (s > smin + L/2) then
    reinit(s, smin + L/2);
    if (not initial() or v<0) then
      reinit(v, 0);
    end if;
  end when;

Version 2:
        stopped := if s <= smin + L/2 then -1 else if s >= smax - L/2 then +1 else 0;
      when (initial()) then
        assert(s > smin + L/2 or s >= smin + L/2 and v >= 0,
          "Error in initialisation of hard stop. (s - L/2) must be >= smin\n"+
          "(s=" + String(s) + ", L=" + String(L) + ", smin=" + String(smin) + ")");
        assert(s < smax - L/2 or s <= smax - L/2 and v <= 0,
          "Error in initialisation of hard stop. (s + L/2) must be <= smax\n"+
          "(s=" + String(s) + ", L=" + String(L) + ", smax=" + String(smax) + ")");
      end when;
      when stopped <> 0 then
        reinit(s, if stopped < 0 then smin + L/2 else smax - L/2);
        if (not initial() or stopped*v>0) then
           reinit(v, 0);
        end if;
      end when;
*/


  annotation (
    Documentation(info="<html><p>MassDrift changes the coefficient of acceleration. FrictionLoss, FrictionIncrease and Seizure scale the friction law. StopShift translates both stop locations by the same activated offset. Reaching a stop reinitialises the position to that boundary and the velocity to zero.</p><p>Set m, L, smin, smax and the friction parameters; initialise the body within its stops and inspect s, v, stopped and lossPower.</p><p>The mass law uses m_effective*der(v), without a variable-mass momentum term. lossPower records frictional power and excludes kinetic energy removed by a hard-stop reset.</p><p>Nominal reference: <code>Modelica.Mechanics.Translational.Components.MassWithStopAndFriction</code>. Activation uses the configured half-open fault window or the optional trigger; inspect <code>effectLevel</code> during recovery.</p></html>"),
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
      graphics={
        Line(points={{-100,0},{100,0}}, color={0,127,0}, thickness=0.5),
        Polygon(points={{80,-100},{50,-90},{50,-110},{80,-100}}, lineColor={95,127,95}, fillColor={95,127,95}, fillPattern=FillPattern.Solid, lineThickness=0.5),
        Line(points={{-40,-100},{50,-100}}, color={95,127,95}, thickness=0.5),
        Rectangle(extent={{-30,30},{30,-30}}, fillPattern=FillPattern.Sphere, fillColor={166,221,166}, lineColor={0,127,0}, lineThickness=0.5),
        Rectangle(extent={{-64,-16},{-56,-46}}, fillPattern=FillPattern.Solid, lineColor={0,127,0}, fillColor={0,127,0}, lineThickness=0.5),
        Rectangle(extent={{56,-16},{64,-46}}, fillPattern=FillPattern.Solid, lineColor={0,127,0}, fillColor={0,127,0}, lineThickness=0.5),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Line(points={{-50,-90},{-28,-68}}, color={0,127,0}, thickness=0.5),
        Line(points={{-30,-90},{-8,-68}}, color={0,127,0}, thickness=0.5),
        Line(points={{-10,-90},{12,-68}}, color={0,127,0}, thickness=0.5),
        Line(points={{10,-90},{32,-68}}, color={0,127,0}, thickness=0.5),
        Text(extent={{-150,-110},{150,-140}}, textString="m=%m"),
        Line(visible=useHeatPort, points={{-100,-100},{-100,-40},{3,-40}}, color={191,0,0}, pattern=LinePattern.Dot, thickness=0.5),
        Rectangle(extent={{-70,-46},{70,-70}}, fillColor={160,215,160}, fillPattern=FillPattern.Solid, lineColor={0,127,0}, lineThickness=0.5),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableMassWithStopAndFriction;
