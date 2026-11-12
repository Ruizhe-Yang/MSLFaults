within MSLFaults.Mechanics.Translational.Components;
model FaultableSupportFriction "Coulomb friction in support"

  extends Modelica.Mechanics.Translational.Interfaces.PartialElementaryTwoFlangesAndSupport2;
  extends Modelica.Thermal.HeatTransfer.Interfaces.PartialElementaryConditionalHeatPortWithoutT;

  parameter Real f_pos[:, 2]=[0, 1]
    "Positive sliding friction characteristic [N] as function of v [m/s] (v>=0)";
  parameter Real peak(final min=1) = 1
    "Peak for maximum friction force at v==0 (f0_max = peak*f_pos[1,2])";
  extends Modelica.Mechanics.Translational.Interfaces.PartialFriction;

  Modelica.Units.SI.Position s "= flange_a.s - support.s";
  Modelica.Units.SI.Force f "Friction force";
  Modelica.Units.SI.Velocity v "Absolute velocity of flange_a and flange_b";
  Modelica.Units.SI.Acceleration a "Absolute acceleration of flange_a and flange_b";
  type FaultMode=enumeration(Normal "正常", FrictionLoss "支承摩擦下降", FrictionIncrease "支承摩擦增加", Wear "磨损", Seizure "抱死");
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter Modelica.Units.SI.Time faultStartTime=1;
  parameter Modelica.Units.SI.Time faultEndTime=3;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05;
  Real faultActivation(min=0,max=1);
  Real startActivation(min=0,max=1);
  Real endActivation(min=0,max=1);
  parameter Real frictionScaleFault(min=0)=2;
  parameter Real seizureScale(min=1)=1000;
  Real frictionScale;

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
  // Constant auxiliary variables
  frictionScale=if faultMode==FaultMode.FrictionLoss then 1-faultActivation elseif faultMode==FaultMode.FrictionIncrease or faultMode==FaultMode.Wear then 1+faultActivation*(frictionScaleFault-1) elseif faultMode==FaultMode.Seizure then 1+faultActivation*(seizureScale-1) else 1;
  f0=frictionScale*Modelica.Math.Vectors.interpolate(f_pos[:,1],f_pos[:,2],0,1);
  f0_max = peak*f0;
  free = false;

  s = flange_a.s - s_support;
  flange_a.s = flange_b.s;

  // velocity and acceleration of flanges
  v = der(s);
  a = der(v);
  v_relfric = v;
  a_relfric = a;

  // Friction force
  flange_a.f + flange_b.f - f = 0;

  // Friction force
  f = frictionScale*(if locked then sa*unitForce else (
    if startForward then
      Modelica.Math.Vectors.interpolate(f_pos[:, 1], f_pos[:, 2], v, 1)
    else if startBackward then
      -Modelica.Math.Vectors.interpolate(f_pos[:, 1], f_pos[:, 2], -v, 1)
    else if pre(mode) == Forward then
      Modelica.Math.Vectors.interpolate(f_pos[:, 1], f_pos[:, 2], v, 1)
    else
      -Modelica.Math.Vectors.interpolate(f_pos[:, 1], f_pos[:, 2], -v, 1)));

  lossPower = f*v_relfric;


  annotation (
    Documentation(info="<html><p><b>Purpose</b> Coulomb friction in support</p><p><b>Interfaces</b> Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger; output Boolean active \"Requested or latched fault\"; output Real effectLevel(min=0,max=1) \"Current realized fault effect\"; output Integer activationCount \"Number of rising activations\"</p><p><b>Parameters</b> parameter Real f_pos[:, 2]=[0, 1]; parameter Real peak(final min=1) = 1; parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter Modelica.Units.SI.Time faultStartTime=1; parameter Modelica.Units.SI.Time faultEndTime=3; parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05; parameter Real frictionScaleFault(min=0)=2; parameter Real seizureScale(min=1)=1000; parameter Boolean enabled=true \"Enable the selected fault mode\"; parameter Boolean useExternalTrigger=false \"Use trigger instead of the internal time window\"</p><p><b>Behavior</b> Coulomb friction in support</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.TranslationalStiffness\">MSLFaults.Examples.TranslationalStiffness</a></p><p><b>Purpose</b> Fault-enhanced SupportFriction</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> The established physical equations and fault modes use the common activation and severity law.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Mechanics.Translational.Components.SupportFriction</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.TranslationalStiffness\">MSLFaults.Examples.TranslationalStiffness</a></p><p>用法：将 FaultableSupportFriction 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p>
<p>
This element describes <strong>Coulomb friction</strong> in <strong>support</strong>,
i.e., a frictional force acting between a flange and the housing.
The positive sliding friction force \"f\" has to be defined
by table \"f_pos\" as function of the absolute velocity \"v\".
E.g.
</p>
<blockquote><pre>
 v |   f
---+-----
 0 |   0
 1 |   2
 2 |   5
 3 |   8
</pre></blockquote>
<p>
gives the following table:
</p>
<blockquote><pre>
f_pos = [0, 0; 1, 2; 2, 5; 3, 8];
</pre></blockquote>
<p>
Currently, only linear interpolation in the table is supported.
Outside of the table, extrapolation through the last
two table entries is used. It is assumed that the negative
sliding friction force has the same characteristic with negative
values. Friction is modelled in the following way:
</p>
<p>
When the absolute velocity \"v\" is not zero, the friction force
is a function of v and of a constant normal force. This dependency
is defined via table f_pos and can be determined by measurements,
e.g., by driving the gear with constant velocity and measuring the
needed driving force (= friction force).
</p>
<p>
When the absolute velocity becomes zero, the elements
connected by the friction element become stuck, i.e., the absolute
position remains constant. In this phase the friction force is
calculated from a force balance due to the requirement, that
the absolute acceleration shall be zero.  The elements begin
to slide when the friction force exceeds a threshold value,
called the maximum static friction force, computed via:
</p>
<blockquote><pre>
maximum_static_friction = <strong>peak</strong> * sliding_friction(v=0)  (<strong>peak</strong> >= 1)
</pre></blockquote>
<p>
This procedure is implemented in a \"clean\" way by state events and
leads to continuous/discrete systems of equations if friction elements
are dynamically coupled which have to be solved by appropriate
numerical methods. The method is described in
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
    Press, Boston MA.<br><br></dd>
<dt>Armstrong B., and Canudas de Wit C. (1996):</dt>
<dd><strong>Friction Modeling and Compensation.</strong>
    The Control Handbook, edited by W.S.Levine, CRC Press,
    pp. 1369-1382.<br><br></dd>
<dt>Canudas de Wit C., Olsson H., &Aring;str&ouml;m K.J., and Lischinsky P. (1995):</dt>
<dd><strong>A new model for control of systems with friction.</strong>
    IEEE Transactions on Automatic Control, Vol. 40, No. 3, pp. 419-425.<br><br></dd>
</dl>

</html>"),
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
      graphics={
        Polygon(points={{-50,-50},{-10,-90},{-10,-100},{10,-100},{10,-90},{50,-50},{-50,-50}}, lineColor={95,95,95}, fillColor={131,175,131}, fillPattern=FillPattern.Solid, lineThickness=0.5),
        Ellipse(extent={{-48,-10},{-28,-30}}, lineColor={0,127,0}, fillPattern=FillPattern.Sphere, fillColor={255,255,255}, lineThickness=0.5),
        Ellipse(extent={{-10,-10},{10,-30}}, lineColor={0,127,0}, fillPattern=FillPattern.Sphere, fillColor={255,255,255}, lineThickness=0.5),
        Ellipse(extent={{30,-10},{50,-30}}, lineColor={0,127,0}, fillPattern=FillPattern.Sphere, fillColor={255,255,255}, lineThickness=0.5),
        Ellipse(extent={{-50,30},{-30,10}}, lineColor={0,127,0}, fillPattern=FillPattern.Sphere, fillColor={255,255,255}, lineThickness=0.5),
        Ellipse(extent={{-10,30},{10,10}}, lineColor={0,127,0}, fillPattern=FillPattern.Sphere, fillColor={255,255,255}, lineThickness=0.5),
        Ellipse(extent={{30,30},{50,10}}, lineColor={0,127,0}, fillPattern=FillPattern.Sphere, fillColor={255,255,255}, lineThickness=0.5),
        Rectangle(extent={{-90,10},{90,-10}}, lineColor={0,127,0}, fillColor={160,215,160}, fillPattern=FillPattern.Solid, lineThickness=0.5),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Polygon(points={{-60,30},{60,30},{60,20},{80,20},{80,50},{-80,50},{-80,20},{-60,20},{-60,30}}, lineColor={95,95,95}, fillPattern=FillPattern.Solid, fillColor={131,175,131}, lineThickness=0.5),
        Line(visible=useHeatPort, points={{-100,-100},{-100,-20},{0,-20}}, color={191,0,0}, pattern=LinePattern.Dot, thickness=0.5),
        Polygon(points={{-60,-30},{60,-30},{60,-20},{80,-20},{80,-50},{-80,-50},{-80,-20},{-60,-20},{-60,-30}}, lineColor={95,95,95}, fillPattern=FillPattern.Solid, fillColor={131,175,131}, lineThickness=0.5),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableSupportFriction;
