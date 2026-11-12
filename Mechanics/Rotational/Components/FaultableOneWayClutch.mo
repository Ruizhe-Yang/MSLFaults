within MSLFaults.Mechanics.Rotational.Components;
model FaultableOneWayClutch "Parallel connection of freewheel and clutch"
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

  Real u "Normalized force input signal (0..1)";
  Modelica.Units.SI.Force fn "Normal force (fn=fn_max*inPort.signal)";
  Boolean startForward(start=false)
    "= true, if w_rel=0 and start of forward sliding or w_rel > w_small";
  Boolean locked(start=false) "= true, if w_rel=0 and not sliding";
  Boolean stuck(start=false) "w_rel=0 (locked or start forward sliding)";

  type FaultMode=enumeration(Normal "正常", FrictionLoss "摩擦力下降", FrictionIncrease "摩擦力增加", Slipping "单向离合打滑", Sticking "单向离合卡死");
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter Modelica.Units.SI.Time faultStartTime=1;
  parameter Modelica.Units.SI.Time faultEndTime=3;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05;
  Real faultActivation(min=0,max=1);
  Real startActivation(min=0,max=1);
  Real endActivation(min=0,max=1);
  parameter Real frictionScaleFault(min=0)=0.5;
  parameter Real stickingScale(min=1)=1000;
  Real frictionScale;
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
    "Normalized force signal 0..1 (normal force = fn_max*f_normalized; clutch is engaged if > 0)"
    annotation (Placement(transformation(
        origin={0,110},
        extent={{20,-20},{-20,20}},
        rotation=90)));


public
  parameter Boolean enabled=true "Enable the selected fault mode";
  parameter Boolean useExternalTrigger=false "Use trigger instead of the internal time window" annotation(Evaluate=true);
  Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger
    annotation(Placement(transformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270),
      iconTransformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270)));
  output Boolean active "Requested or latched fault";
  output Real effectLevel(min=0,max=1) "Current realized fault effect";
  output Integer activationCount "Number of rising activations";

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
      }), Documentation(info="<html><p><b>Purpose</b> Parallel connection of freewheel and clutch</p><p><b>Interfaces</b> See the named arguments, record fields, or inherited connectors.</p><p><b>Parameters</b> parameter Real mu_pos[:, 2]=[0, 0.5]; parameter Real peak(final min=1) = 1; parameter Real cgeo(final min=0) = 1; parameter Modelica.Units.SI.Force fn_max(final min=0, start=1) \"Maximum normal force\"; parameter Modelica.Units.SI.AngularVelocity w_small=1e10; parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter Modelica.Units.SI.Time faultStartTime=1; parameter Modelica.Units.SI.Time faultEndTime=3; parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05; parameter Real frictionScaleFault(min=0)=0.5; parameter Real stickingScale(min=1)=1000</p><p><b>Behavior</b> Parallel connection of freewheel and clutch</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.RotationalDamping\">MSLFaults.Examples.RotationalDamping</a></p><p><b>Purpose</b> Fault-enhanced OneWayClutch</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> The established physical equations and fault modes use the common activation and severity law.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Mechanics.Rotational.Components.OneWayClutch</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.RotationalDamping\">MSLFaults.Examples.RotationalDamping</a></p><p>用法：将 FaultableOneWayClutch 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p>
<p>
This component models a <strong>one-way clutch</strong>, i.e., a component with
two flanges where friction is present between the two flanges
and these flanges are pressed together via a normal force. These
flanges may be sliding with respect to each other.
</p>
<p>
A one-way-clutch is an element where a&nbsp;clutch is connected in parallel
to a&nbsp;free wheel. This special element is provided, because such
a parallel connection introduces an ambiguity into the model
(the constraint torques are not uniquely defined when both
elements are stuck) and this element resolves it by introducing
<strong>one</strong> constraint torque only instead of two constraints.
</p>
<p>
Note, initial values have to be chosen for the model such that the
relative speed of the one-way-clutch &ge;&nbsp;0. Otherwise, the configuration
is physically not possible and an error occurs.
</p>
<p>
The normal force&nbsp;fn has to be provided as input signal f_normalized in a normalized form
(0&nbsp;&le;&nbsp;f_normalized&nbsp;&le;&nbsp;1),
fn&nbsp;=&nbsp;fn_max&nbsp;*&nbsp;f_normalized, where fn_max has to be provided as parameter.
</p>
<p>
The friction in the clutch is modeled in the following way.
When the relative angular velocity is positive, the friction torque is a
function of the velocity dependent friction coefficient mu(w_rel), of
the normal force&nbsp;fn, and of a geometry constant cgeo which takes into
account the geometry of the device and the assumptions on the friction
distributions:
</p>

<blockquote><pre>
frictional_torque = <strong>cgeo</strong> * <strong>mu</strong>(w_rel) * <strong>fn</strong>
</pre></blockquote>

<p>
Typical values of coefficients of friction <strong>mu</strong>:
</p>
<ul>
  <li>0.2&nbsp;&hellip;&nbsp;0.4 for dry operation,</li>
  <li>0.05&nbsp;&hellip;&nbsp;0.1 when operating in oil.</li>
</ul>

<p>
The geometry constant is calculated - under the
assumption of a uniform rate of wear at the friction surfaces - in the following way:
</p>

<blockquote><pre>
<strong>cgeo</strong> = <strong>N</strong>*(<strong>r0</strong> + <strong>ri</strong>)/2
</pre></blockquote>

<p>
where <strong>ri</strong> is the inner radius,
<strong>ro</strong> is the outer radius and&nbsp;<strong>N</strong> is the number of friction interfaces,
</p>

<p>
The positive part of the friction characteristic <strong>mu</strong>(w_rel),
w_rel&nbsp;>=&nbsp;0, is defined via table mu_pos (first column = w_rel,
second column = mu). Currently, only linear interpolation in
the table is supported.
</p>
<p>
When the relative angular velocity w_rel becomes zero, the elements
connected by the friction element become stuck, i.e., the relative
angle remains constant. In this phase the friction torque is
calculated from a torque balance due to the requirement that
the relative acceleration shall be zero.  The elements begin
to slide when the friction torque exceeds a threshold value,
called the  maximum static friction torque, computed via:
</p>

<blockquote><pre>
frictional_torque = <strong>peak</strong> * <strong>cgeo</strong> * <strong>mu</strong>(w_rel=0) * <strong>fn</strong>,   (<strong>peak</strong> >= 1)
</pre></blockquote>

<p>
This procedure is implemented in a \"clean\" way by state events and
leads to continuous/discrete systems of equations if friction elements
are dynamically coupled. The method is described in
(see also a short sketch in <a href=\"modelica://Modelica.Mechanics.Rotational.UsersGuide.ModelingOfFriction\">UsersGuide.ModelingOfFriction</a>):
</p>
<dl>
<dt>Otter M., Elmqvist H., and Mattsson S.E. (1999):</dt>
<dd><strong>Hybrid Modeling in Modelica based on the Synchronous
    Data Flow Principle</strong>. CACSD'99, Aug. 22.-26, Hawaii.</dd>
</dl>

<p>
See also the discussion
<a href=\"modelica://Modelica.Mechanics.Rotational.UsersGuide.StateSelection\">State Selection</a>
in the User's Guide of the Rotational library.
</p>
</html>"));
end FaultableOneWayClutch;
