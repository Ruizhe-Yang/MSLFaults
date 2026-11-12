within MSLFaults.Mechanics.Rotational.Components;
model FaultableBearingFriction "Coulomb friction in bearings"
  extends
    Modelica.Mechanics.Rotational.Interfaces.PartialElementaryTwoFlangesAndSupport2;

  parameter Real tau_pos[:, 2]=[0, 1]
    "Positive sliding friction characteristic [N.m] as function of w [rad/s] (w>=0)";
  parameter Real peak(final min=1) = 1
    "Peak for maximum friction torque at w==0 (tau0_max = peak*tau_pos[1,2])";

  extends Modelica.Mechanics.Rotational.Interfaces.PartialFriction;
  extends
    Modelica.Thermal.HeatTransfer.Interfaces.PartialElementaryConditionalHeatPortWithoutT;

  Modelica.Units.SI.Angle phi
    "Angle between shaft flanges (flange_a, flange_b) and support";
  Modelica.Units.SI.Torque tau "Friction torque";
  Modelica.Units.SI.AngularVelocity w "Absolute angular velocity of flange_a and flange_b";
  Modelica.Units.SI.AngularAcceleration a
    "Absolute angular acceleration of flange_a and flange_b";

  type FaultMode=enumeration(Normal "正常", FrictionLoss "轴承摩擦下降", FrictionIncrease "轴承摩擦增加", Wear "轴承磨损", Seizure "轴承抱死");
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter Modelica.Units.SI.Time faultStartTime=1;
  parameter Modelica.Units.SI.Time faultEndTime=3;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05;
  Real faultActivation(min=0,max=1);
  Real startActivation(min=0,max=1);
  Real endActivation(min=0,max=1);
  parameter Real frictionLossScale(min=0,max=1)=0.5;
  parameter Real frictionIncreaseScale(min=1)=2;
  parameter Real wearScale(min=1)=3;
  parameter Modelica.Units.SI.Time wearTime(min=Modelica.Constants.small)=10;
  parameter Real seizureScale(min=1)=1000;
  Real frictionScale;
  Real wearProgress(min=0,max=1);

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
  wearProgress=if time<=faultStartTime then 0 else min(1,max(0,(min(time,faultEndTime)-faultStartTime)/wearTime));
  frictionScale=if faultMode==FaultMode.FrictionLoss then 1+faultActivation*(frictionLossScale-1)
    elseif faultMode==FaultMode.FrictionIncrease then 1+faultActivation*(frictionIncreaseScale-1)
    elseif faultMode==FaultMode.Wear then 1+severity*endActivation*wearProgress*(wearScale-1)
    elseif faultMode==FaultMode.Seizure then 1+faultActivation*(seizureScale-1) else 1;
  tau0=frictionScale*Modelica.Math.Vectors.interpolate(tau_pos[:,1],tau_pos[:,2],0,1);
  tau0_max = peak*tau0;
  free = false;

  phi = flange_a.phi - phi_support;
  flange_b.phi = flange_a.phi;

  // Angular velocity and angular acceleration of flanges
  w = der(phi);
  a = der(w);
  w_relfric = w;
  a_relfric = a;

  // Friction torque
  flange_a.tau + flange_b.tau - tau = 0;

  // Friction torque
  tau = frictionScale*(if locked then sa*unitTorque else (
    if startForward then
      Modelica.Math.Vectors.interpolate(tau_pos[:,1], tau_pos[:,2], w, 1)
    else if startBackward then
      -Modelica.Math.Vectors.interpolate(tau_pos[:,1], tau_pos[:,2], -w, 1)
    else if pre(mode) == Forward then
      Modelica.Math.Vectors.interpolate(tau_pos[:,1], tau_pos[:,2], w, 1)
    else
      -Modelica.Math.Vectors.interpolate(tau_pos[:,1], tau_pos[:,2], -w, 1)));
  lossPower = tau*w_relfric;


  annotation (Documentation(info="<html><p><b>Purpose</b> Coulomb friction in bearings</p><p><b>Interfaces</b> Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger; output Boolean active \"Requested or latched fault\"; output Real effectLevel(min=0,max=1) \"Current realized fault effect\"; output Integer activationCount \"Number of rising activations\"</p><p><b>Parameters</b> parameter Real tau_pos[:, 2]=[0, 1]; parameter Real peak(final min=1) = 1; parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter Modelica.Units.SI.Time faultStartTime=1; parameter Modelica.Units.SI.Time faultEndTime=3; parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05; parameter Real frictionLossScale(min=0,max=1)=0.5; parameter Real frictionIncreaseScale(min=1)=2; parameter Real wearScale(min=1)=3; parameter Modelica.Units.SI.Time wearTime(min=Modelica.Constants.small)=10; parameter Real seizureScale(min=1)=1000</p><p><b>Behavior</b> Coulomb friction in bearings</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.RotationalDamping\">MSLFaults.Examples.RotationalDamping</a></p><p><b>Purpose</b> Fault-enhanced BearingFriction</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> The established physical equations and fault modes use the common activation and severity law.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Mechanics.Rotational.Components.BearingFriction</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.RotationalDamping\">MSLFaults.Examples.RotationalDamping</a></p><p>用法：将 FaultableBearingFriction 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p>
<p>
This element describes <strong>Coulomb friction</strong> in <strong>bearings</strong>,
i.e., a frictional torque acting between a flange and the housing.
The positive sliding friction torque \"tau\" has to be defined
by table \"tau_pos\" as function of the absolute angular velocity \"w\".
E.g.
</p>
<blockquote><pre>
 w | tau
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
tau_pos = [0, 0; 1, 2; 2, 5; 3, 8];
</pre></blockquote>
<p>
Currently, only linear interpolation in the table is supported.
Outside of the table, extrapolation through the last
two table entries is used. It is assumed that the negative
sliding friction force has the same characteristic with negative
values. Friction is modelled in the following way:
</p>
<p>
When the absolute angular velocity \"w\" is not zero, the friction torque
is a function of w and of a constant normal force. This dependency
is defined via table tau_pos and can be determined by measurements,
e.g., by driving the gear with constant velocity and measuring the
needed motor torque (= friction torque).
</p>
<p>
When the absolute angular velocity becomes zero, the elements
connected by the friction element become stuck, i.e., the absolute
angle remains constant. In this phase the friction torque is
calculated from a torque balance due to the requirement, that
the absolute acceleration shall be zero.  The elements begin
to slide when the friction torque exceeds a threshold value,
called the maximum static friction torque, computed via:
</p>
<blockquote><pre>
maximum_static_friction = <strong>peak</strong> * sliding_friction(w=0)  (<strong>peak</strong> >= 1)
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
    Press, Boston MA.<br></dd>
<dt>Armstrong B., and Canudas de Wit C. (1996):</dt>
<dd><strong>Friction Modeling and Compensation.</strong>
    The Control Handbook, edited by W.S.Levine, CRC Press,
    pp. 1369-1382.<br></dd>
<dt>Canudas de Wit C., Olsson H., &Aring;str&ouml;m K.J., and Lischinsky P. (1995):</dt>
<dd><strong>A new model for control of systems with friction.</strong>
    IEEE Transactions on Automatic Control, Vol. 40, No. 3, pp. 419-425.</dd>
</dl>
</html>"),
       Icon(coordinateSystem(preserveAspectRatio=true,
      extent={{-100.0,-100.0},{100.0,100.0}}),
      graphics={
        Rectangle(lineColor={64,64,64}, fillColor={192,192,192}, fillPattern=FillPattern.HorizontalCylinder, extent={{-100,-10},{100,10}}, lineThickness=0.5),
        Rectangle(extent={{-60,-60},{60,-10}}, lineColor={64,64,64}, lineThickness=0.5),
        Rectangle(fillColor={192,192,192}, fillPattern=FillPattern.Solid, extent={{-60,-25},{60,-10}}, lineColor={64,64,64}, lineThickness=0.5),
        Rectangle(fillColor={192,192,192}, fillPattern=FillPattern.Solid, extent={{-60,-61},{60,-45}}, lineColor={64,64,64}, lineThickness=0.5),
        Rectangle(fillColor={255,255,255}, fillPattern=FillPattern.Solid, extent={{-50,-50},{50,-18}}, lineColor={64,64,64}, lineThickness=0.5),
        Polygon(fillColor={160,160,164}, fillPattern=FillPattern.Solid, points={{60,-60},{60,-70},{75,-70},{75,-80},{-75,-80},{-75,-70},{-60,-70},{-60,-60},{60,-60}}, lineColor={64,64,64}, lineThickness=0.5),
        Line(points={{-75,-10},{-75,-70}}, color={64,64,64}, thickness=0.5),
        Line(points={{75,-10},{75,-70}}, color={64,64,64}, thickness=0.5),
        Rectangle(extent={{-60,10},{60,60}}, lineColor={64,64,64}, lineThickness=0.5),
        Rectangle(fillColor={192,192,192}, fillPattern=FillPattern.Solid, extent={{-60,45},{60,60}}, lineColor={64,64,64}, lineThickness=0.5),
        Rectangle(fillColor={192,192,192}, fillPattern=FillPattern.Solid, extent={{-60,10},{60,25}}, lineColor={64,64,64}, lineThickness=0.5),
        Rectangle(fillColor={255,255,255}, fillPattern=FillPattern.Solid, extent={{-50,19},{50,51}}, lineColor={64,64,64}, lineThickness=0.5),
        Line(points={{-75,70},{-75,10}}, color={64,64,64}, thickness=0.5),
        Polygon(fillColor={160,160,164}, fillPattern=FillPattern.Solid, points={{60,60},{60,70},{75,70},{75,80},{-75,80},{-75,70},{-60,70},{-60,60},{60,60}}, lineColor={64,64,64}, lineThickness=0.5),
        Line(points={{75,70},{75,10}}, color={64,64,64}, thickness=0.5),
        Text(textColor={0,0,255}, extent={{-150,155},{150,125}}, textString="%name"),
        Line(points={{0,-80},{0,-100}}, color={64,64,64}, thickness=0.5),
        Line(visible=useHeatPort, points={{-100,-100},{-100,-35},{2,-35}}, color={191,0,0}, pattern=LinePattern.Dot, thickness=0.5),
        Text(extent={{108,94},{139,62}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableBearingFriction;
