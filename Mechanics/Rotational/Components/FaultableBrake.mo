within MSLFaults.Mechanics.Rotational.Components;
model FaultableBrake "Brake based on Coulomb friction"
  extends
    Modelica.Mechanics.Rotational.Interfaces.PartialElementaryTwoFlangesAndSupport2;

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

  Modelica.Units.SI.Angle phi
    "Angle between shaft flanges (flange_a, flange_b) and support";
  Modelica.Units.SI.Torque tau "Brake friction torque";
  Modelica.Units.SI.AngularVelocity w "Absolute angular velocity of flange_a and flange_b";
  Modelica.Units.SI.AngularAcceleration a
    "Absolute angular acceleration of flange_a and flange_b";

  Real mu0 "Friction coefficient for w=0 and forward sliding";
  Modelica.Units.SI.Force fn "Normal force (=fn_max*f_normalized)";

  // Constant auxiliary variable
  Modelica.Blocks.Interfaces.RealInput f_normalized
    "Normalized force signal 0..1 (normal force = fn_max*f_normalized; brake is active if > 0)"
    annotation (Placement(transformation(
        origin={0,110},
        extent={{20,-20},{-20,20}},
        rotation=90)));

  type FaultMode=enumeration(Normal "正常", FrictionLoss "摩擦力下降", FrictionIncrease "摩擦力增加", Slipping "打滑", Sticking "粘滞卡死");
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter Modelica.Units.SI.Time faultStartTime=1;
  parameter Modelica.Units.SI.Time faultEndTime=3;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05;
  Real faultActivation(min=0,max=1);
  Real startActivation(min=0,max=1);
  Real endActivation(min=0,max=1);
  parameter Real frictionScaleFault(min=0,max=1)=0.5
    "Compatibility alias for earlier replacement scenarios";
  parameter Real frictionLossScale(min=0,max=1)=frictionScaleFault;
  parameter Real frictionIncreaseScale(min=1)=2;
  parameter Real slipDynamicScale(min=0,max=1)=0.2;
  parameter Real stickingStaticScale(min=1)=1000;
  Real dynamicScale;
  Real staticScale;

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

  mu0 = Modelica.Math.Vectors.interpolate(mu_pos[:,1], mu_pos[:,2], 0, 1);

  phi = flange_a.phi - phi_support;
  flange_b.phi = flange_a.phi;

  // Angular velocity and angular acceleration of flanges flange_a and flange_b
  w = der(phi);
  a = der(w);
  w_relfric = w;
  a_relfric = a;

  // Friction torque, normal force and friction torque for w_rel=0
  flange_a.tau + flange_b.tau - tau = 0;
  dynamicScale=if faultMode==FaultMode.FrictionLoss then 1+faultActivation*(frictionLossScale-1)
    elseif faultMode==FaultMode.FrictionIncrease then 1+faultActivation*(frictionIncreaseScale-1)
    elseif faultMode==FaultMode.Slipping then 1+faultActivation*(slipDynamicScale-1) else 1;
  staticScale=if faultMode==FaultMode.FrictionLoss then 1+faultActivation*(frictionLossScale-1)
    elseif faultMode==FaultMode.FrictionIncrease then 1+faultActivation*(frictionIncreaseScale-1)
    elseif faultMode==FaultMode.Sticking then 1+faultActivation*(stickingStaticScale-1) else 1;
  fn=fn_max*f_normalized;
  tau0 = mu0*cgeo*fn*staticScale;
  tau0_max = peak*tau0;
  free = fn <= 0;

  // Friction torque
  tau = if locked then sa*unitTorque else if free then 0 else cgeo*fn*dynamicScale*(
    if startForward then
      Modelica.Math.Vectors.interpolate(mu_pos[:,1], mu_pos[:,2], w, 1)
    else if startBackward then
      -Modelica.Math.Vectors.interpolate(mu_pos[:,1], mu_pos[:,2], -w, 1)
    else if pre(mode) == Forward then
      Modelica.Math.Vectors.interpolate(mu_pos[:,1], mu_pos[:,2], w, 1)
    else
      -Modelica.Math.Vectors.interpolate(mu_pos[:,1], mu_pos[:,2], -w, 1));
  lossPower = tau*w_relfric;


  annotation (Icon(coordinateSystem(preserveAspectRatio=true,
        extent={{-100.0,-100.0},{100.0,100.0}}),
      graphics={
        Polygon(lineColor={192,192,192}, fillColor={192,192,192}, fillPattern=FillPattern.Solid, points={{-37,-55},{-37,-90},{37,-90},{37,-55},{33,-55},{33,-86},{-33,-86},{-33,-55},{-37,-55}}, lineThickness=0.5),
        Rectangle(lineColor={64,64,64}, fillColor={192,192,192}, fillPattern=FillPattern.HorizontalCylinder, extent={{-100,-10},{-20,10}}, lineThickness=0.5),
        Rectangle(lineColor={64,64,64}, fillColor={255,255,255}, fillPattern=FillPattern.HorizontalCylinder, extent={{-20,-60},{20,60}}, radius=10, lineThickness=0.5),
        Rectangle(lineColor={64,64,64}, extent={{-20,-60},{20,60}}, radius=10, lineThickness=0.5),
        Rectangle(lineColor={64,64,64}, fillColor={192,192,192}, fillPattern=FillPattern.HorizontalCylinder, extent={{20,-10},{100,10}}, lineThickness=0.5),
        Polygon(lineColor={0,0,127}, fillColor={0,0,127}, fillPattern=FillPattern.Solid, points={{40,-40},{70,-30},{70,-50},{40,-40}}, lineThickness=0.5),
        Rectangle(fillPattern=FillPattern.Solid, extent={{30,-55},{40,-25}}, lineColor={64,64,64}, lineThickness=0.5),
        Polygon(lineColor={0,0,127}, fillColor={0,0,127}, fillPattern=FillPattern.Solid, points={{-40,-40},{-70,-30},{-70,-50},{-40,-40}}, lineThickness=0.5),
        Rectangle(fillPattern=FillPattern.Solid, extent={{-40,-55},{-30,-25}}, lineColor={64,64,64}, lineThickness=0.5),
        Line(points={{0,90},{80,70},{80,-40},{70,-40}}, color={0,0,127}, thickness=0.5),
        Line(points={{0,90},{-80,70},{-80,-40},{-70,-40}}, color={0,0,127}, thickness=0.5),
        Text(textColor={0,0,255}, extent={{-150,-180},{150,-140}}, textString="%name"),
        Line(visible=useHeatPort, points={{-100,-98},{-100,-70},{0,-70},{0,-40}}, color={191,0,0}, pattern=LinePattern.Dot, thickness=0.5),
        Text(extent={{108,94},{139,62}}, textString="F", textColor={255,0,0}, fontSize=0)
      }), Documentation(info="<html><p><b>Purpose</b> Brake based on Coulomb friction</p><p><b>Interfaces</b> Modelica.Blocks.Interfaces.RealInput f_normalized; Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger; output Boolean active \"Requested or latched fault\"; output Real effectLevel(min=0,max=1) \"Current realized fault effect\"; output Integer activationCount \"Number of rising activations\"</p><p><b>Parameters</b> parameter Real mu_pos[:, 2]=[0, 0.5]; parameter Real peak(final min=1) = 1; parameter Real cgeo(final min=0) = 1; parameter Modelica.Units.SI.Force fn_max(final min=0, start=1) \"Maximum normal force\"; parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter Modelica.Units.SI.Time faultStartTime=1; parameter Modelica.Units.SI.Time faultEndTime=3; parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05; parameter Real frictionScaleFault(min=0,max=1)=0.5; parameter Real frictionLossScale(min=0,max=1)=frictionScaleFault; parameter Real frictionIncreaseScale(min=1)=2</p><p><b>Behavior</b> Brake based on Coulomb friction</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.RotationalDamping\">MSLFaults.Examples.RotationalDamping</a></p><p><b>Purpose</b> Fault-enhanced Brake</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> The established physical equations and fault modes use the common activation and severity law.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Mechanics.Rotational.Components.Brake</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.RotationalDamping\">MSLFaults.Examples.RotationalDamping</a></p><p>用法：将 FaultableBrake 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p>
<p>
This component models a <strong>brake</strong>, i.e., a component where a frictional
torque is acting between the housing and a flange and a controlled normal
force presses the flange to the housing in order to increase friction.
The normal force fn has to be provided as input signal f_normalized in a normalized form
(0 &le; f_normalized &le; 1),
fn = fn_max*f_normalized, where fn_max has to be provided as parameter.
Friction in the brake is modelled in the following way:
</p>
<p>
When the absolute angular velocity \"w\" is not zero, the friction torque
is a function of the velocity dependent friction coefficient mu(w), of
the normal force \"fn\", and of a geometry constant \"cgeo\" which takes into
account the geometry of the device and the assumptions on the friction
distributions:
</p>
<blockquote><pre>
frictional_torque = <strong>cgeo</strong> * <strong>mu</strong>(w) * <strong>fn</strong>
</pre></blockquote>
<p>
   Typical values of coefficients of friction <strong>mu</strong>:
</p>
<ul>
  <li>0.2&nbsp;&hellip;&nbsp;0.4 for dry operation,</li>
  <li>0.05&nbsp;&hellip;&nbsp;0.1 when operating in oil.</li>
</ul>
<p>
   When plates are pressed together, where  <strong>ri</strong>  is the inner radius,
   <strong>ro</strong> is the outer radius and <strong>N</strong> is the number of friction interfaces,
   the geometry constant is calculated in the following way under the
   assumption of a uniform rate of wear at the interfaces:
</p>
<blockquote><pre>
<strong>cgeo</strong> = <strong>N</strong>*(<strong>r0</strong> + <strong>ri</strong>)/2
</pre></blockquote>
<p>
    The positive part of the friction characteristic <strong>mu</strong>(w),
    w >= 0, is defined via table mu_pos (first column = w,
    second column = mu). Currently, only linear interpolation in
    the table is supported.
</p>
<p>
   When the absolute angular velocity becomes zero, the elements
   connected by the friction element become stuck, i.e., the absolute
   angle remains constant. In this phase the friction torque is
   calculated from a torque balance due to the requirement, that
   the absolute acceleration shall be zero.  The elements begin
   to slide when the friction torque exceeds a threshold value,
   called the  maximum static friction torque, computed via:
</p>
<blockquote><pre>
frictional_torque = <strong>peak</strong> * <strong>cgeo</strong> * <strong>mu</strong>(w=0) * <strong>fn</strong>   (<strong>peak</strong> >= 1)
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
More precise friction models take into account the elasticity of the
material when the two elements are \"stuck\", as well as other effects,
like hysteresis. This has the advantage that the friction element can
be completely described by a differential equation without events. The
drawback is that the system becomes stiff (about 10-20 times slower
simulation) and that more material constants have to be supplied which
requires more sophisticated identification. For more details, see the
following references, especially (Armstrong and Canudas de Wit 1996):
</p>
<dl>
<dt>Armstrong B. (1991):</dt>
<dd><strong>Control of Machines with Friction</strong>. Kluwer Academic
    Press, Boston MA.<br></dd>
<dt>Armstrong B., and Canudas de Wit C. (1996):</dt>
<dd><strong>Friction Modeling and Compensation.</strong>
    The Control Handbook, edited by W.S.Levine, CRC Press,
    pp. 1369-1382.<br></dd>
<dt>Canudas de Wit C., Olsson H., &Aring;str&ouml;m K.J., and Lischinsky P. (1995):</dt>
<dd><strong>A new model for control of systems with friction.</strong>
    IEEE Transactions on Automatic Control, Vol. 40, No. 3, pp. 419-425.</dd>
</dl>

<p>
See also the discussion
<a href=\"modelica://Modelica.Mechanics.Rotational.UsersGuide.StateSelection\">State Selection</a>
in the User's Guide of the Rotational library.
</p>
</html>"));
end FaultableBrake;
