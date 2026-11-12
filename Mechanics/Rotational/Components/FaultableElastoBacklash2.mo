within MSLFaults.Mechanics.Rotational.Components;
model FaultableElastoBacklash2
  "Backlash connected in series to linear spring and damper (backlash is modeled with elasticity; at start of contact the flange torque can jump, contrary to the ElastoBacklash model)"

  parameter Modelica.Units.SI.RotationalSpringConstant c(final min=Modelica.Constants.small,
      start=1.0e5) "Spring constant (c > 0 required)";
  parameter Modelica.Units.SI.RotationalDampingConstant d(final min=0, start=0)
    "Damping constant";
  parameter Modelica.Units.SI.Angle b(final min=0)=0 "Total backlash";
  parameter Modelica.Units.SI.Angle phi_rel0=0 "Unstretched spring angle";

  extends
    Modelica.Mechanics.Rotational.Interfaces.PartialCompliantWithRelativeStates;
  extends
    Modelica.Thermal.HeatTransfer.Interfaces.PartialElementaryConditionalHeatPortWithoutT;
  type FaultMode=enumeration(Normal "正常", StiffnessLoss "刚度下降", DampingLoss "阻尼下降", BacklashIncrease "侧隙增加", ToothBroken "断齿", Seizure "卡死");
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter Modelica.Units.SI.Time faultStartTime=1;
  parameter Modelica.Units.SI.Time faultEndTime=3;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05;
  Real faultActivation(min=0,max=1);
  Real startActivation(min=0,max=1);
  Real endActivation(min=0,max=1);
  parameter Modelica.Units.SI.RotationalSpringConstant cFault=0.5*c;
  parameter Modelica.Units.SI.RotationalDampingConstant dFault=0.5*d;
  parameter Modelica.Units.SI.Angle bFault=b+0.1;
  parameter Modelica.Units.SI.RotationalDampingConstant dSeized=1e9;
  Modelica.Units.SI.RotationalSpringConstant c_effective;
  Modelica.Units.SI.RotationalDampingConstant d_effective;
  Modelica.Units.SI.Angle b_effective;
  Modelica.Units.SI.Angle bMax_effective;
  Modelica.Units.SI.Angle bMin_effective;
protected
  final parameter Modelica.Units.SI.Angle bMax=b/2
    "Backlash in range bMin <= phi_rel - phi_rel0 <= bMax";
  final parameter Modelica.Units.SI.Angle bMin=-bMax
    "Backlash in range bMin <= phi_rel - phi_rel0 <= bMax";
  Modelica.Units.SI.Torque tau_c;
  Modelica.Units.SI.Torque tau_d;
  Modelica.Units.SI.Angle phi_diff=phi_rel - phi_rel0;

  // A minimum backlash is defined in order to avoid an infinite
  // number of state events if backlash b is set to zero.
  constant Modelica.Units.SI.Angle bEps=1e-10 "Minimum backlash";


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
    permanent=faultMode==FaultMode.ToothBroken,severity=severity,startTime=faultStartTime,endTime=faultEndTime,
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
  c_effective=if faultMode==FaultMode.StiffnessLoss then c+faultActivation*(cFault-c) elseif faultMode==FaultMode.ToothBroken then c*(1-faultActivation) else c;
  d_effective=if faultMode==FaultMode.DampingLoss then d+faultActivation*(dFault-d) elseif faultMode==FaultMode.ToothBroken then d*(1-faultActivation) elseif faultMode==FaultMode.Seizure then d+faultActivation*(dSeized-d) else d;
  b_effective=if faultMode==FaultMode.BacklashIncrease then b+faultActivation*(bFault-b) else b;
  bMax_effective=b_effective/2;
  bMin_effective=-b_effective/2;

  if initial() then
    /* During initialization the characteristic is modified, in order that
        it is a strict monotone rising function. Otherwise, initialization might
        result in a singular system when the characteristic has to be
        inverted. The characteristic is modified in the range 1.5*bMin_effective <= phi_rel - phi_rel0 <= 1.5 bMax_effective,
        so that in this range a linear characteristic is present that approaches the original
        function continuously at its limits, e.g.,
          original:  tau(1.5*bMax_effective) = c_effective*(phi_diff - bMax_effective)
                                   = c_effective*(0.5*bMax_effective)
          initial :  tau(1.5*bMax_effective) = (c_effective/3)*phi_diff
                                   = (c_effective/3)*(3/2)*bMax_effective
                                   = (c_effective/2)*bMax_effective
     */
    tau_c = if phi_diff > 1.5*bMax_effective then c_effective*(phi_diff - bMax_effective) else if
      phi_diff < 1.5*bMin_effective then c_effective*(phi_diff - bMin_effective) else (c_effective/3)*phi_diff;
    tau_d = d_effective*w_rel;
    tau = tau_c + tau_d;
    lossPower = tau_d*w_rel;
   else
     /*
     if abs(b_effective) <= bEps then
        tau_c = c_effective*phi_diff;
        tau_d = d_effective*w_rel;
        tau   = tau_c + tau_d;
     elseif phi_diff > bMax_effective then
        tau_c = c_effective*(phi_diff - bMax_effective);
        tau_d = d_effective*w_rel;
        tau   = if tau_c + tau_d <= 0 then 0 else tau_c + tau_d;
     elseif phi_diff < bMin_effective then
        tau_c = c_effective*(phi_diff - bMin_effective);
        tau_d = d_effective*w_rel;
        tau   = if tau_c + tau_d >= 0 then 0 else tau_c + tau_d;
     else
        tau_c = 0;
        tau_d = 0;
        tau   = 0;
     end if;

     This is written in the form below, in order that parameter "b"
     is not evaluated during translation (i.e., in the above form
     it cannot be changed anymore after translation).
   */
    tau_c = if abs(b_effective) <= bEps then
               c_effective*phi_diff
            else if phi_diff > bMax_effective then
               c_effective*(phi_diff - bMax_effective)
            else if phi_diff < bMin_effective then
               c_effective*(phi_diff - bMin_effective)
            else 0;
    tau_d = d_effective*w_rel;

    tau = if abs(b_effective) <= bEps then
             tau_c + tau_d
          else if phi_diff > bMax_effective then
             (if tau_c + tau_d <= 0 then 0 else tau_c + tau_d)
          else if phi_diff < bMin_effective then
             (if tau_c + tau_d >= 0 then 0 else tau_c + tau_d)
          else 0;

    lossPower = if abs(b_effective) <= bEps then
                   tau_d*w_rel
                else if phi_diff > bMax_effective then
                   (if tau_c + tau_d <= 0 then 0 else tau_d*w_rel)
                else if phi_diff < bMin_effective then
                   (if tau_c + tau_d >= 0 then 0 else tau_d*w_rel)
                else 0;
  end if;


  annotation (defaultComponentName="elastoBacklash",
    Documentation(info="<html><p><b>Purpose</b> FaultableElastoBacklash2</p><p><b>Interfaces</b> See the named arguments, record fields, or inherited connectors.</p><p><b>Parameters</b> parameter Modelica.Units.SI.RotationalSpringConstant c(final min=Modelica.Constants.small,; parameter Modelica.Units.SI.RotationalDampingConstant d(final min=0, start=0); parameter Modelica.Units.SI.Angle b(final min=0)=0 \"Total backlash\"; parameter Modelica.Units.SI.Angle phi_rel0=0 \"Unstretched spring angle\"; parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter Modelica.Units.SI.Time faultStartTime=1; parameter Modelica.Units.SI.Time faultEndTime=3; parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05; parameter Modelica.Units.SI.RotationalSpringConstant cFault=0.5*c; parameter Modelica.Units.SI.RotationalDampingConstant dFault=0.5*d; parameter Modelica.Units.SI.Angle bFault=b+0.1</p><p><b>Behavior</b> FaultableElastoBacklash2</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.RotationalDamping\">MSLFaults.Examples.RotationalDamping</a></p><p><b>Purpose</b> Fault-enhanced ElastoBacklash2</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> The established physical equations and fault modes use the common activation and severity law.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery. ToothBroken latch until a new simulation. faultEndTime does not repair a latched fault.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Mechanics.Rotational.Components.ElastoBacklash2</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.RotationalDamping\">MSLFaults.Examples.RotationalDamping</a></p><p>用法：将 FaultableElastoBacklash2 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p>
<p>
This element consists of a <strong>backlash</strong> element <strong>connected in series</strong>
to a <strong>spring</strong> and <strong>damper</strong> element which are <strong>connected in parallel</strong>.
The spring constant shall be non-zero, otherwise the component cannot be used.
</p>

<p>
In combination with components IdealGear, the ElastoBacklash2 model
can be used to model a gear box with backlash, elasticity and damping.
</p>

<p>
During initialization, the backlash characteristic is replaced by a continuous
approximation in the backlash region, in order to reduce problems during
initialization, especially for inverse models.
</p>

<p>
If the backlash b is smaller as 1e-10 rad (especially, if b=0),
then the backlash is ignored and the component reduces to a spring/damper
element in parallel.
</p>

<p>
In the backlash region (-b/2 &le; flange_b.phi - flange_a.phi - phi_rel0 &le; b/2) no torque
is exerted (flange_b.tau = 0). Outside of this region, contact is present and
the contact torque is basically computed with a linear
spring/damper characteristic:
</p>

<blockquote><pre>
desiredContactTorque = c_effective*phi_contact + d_effective*<strong>der</strong>(phi_contact)

         phi_contact = phi_rel - phi_rel0 - b/2 <strong>if</strong> phi_rel - phi_rel0 &gt;  b/2
                     = phi_rel - phi_rel0 + b/2 <strong>if</strong> phi_rel - phi_rel0 &lt; -b/2

         phi_rel     = flange_b.phi - flange_a.phi;
</pre></blockquote>

<p>
This torque characteristic leads to the following difficulty:
</p>

<ul>
<li> If the damper torque becomes larger as the spring torque and with opposite sign,
     the contact torque would be \"pulling/sticking\" which is unphysical, since during
     contact only pushing torques can occur.</li>
</ul>

<p>
In the literature this issue seems to be not discussed. For this reason, the most simple
approach is used in the ElastoBacklash2 model, by slightly changing
the linear spring/damper characteristic to:
</p>

<blockquote><pre>
// Torque characteristic when phi_rel > phi_rel0
<strong>if</strong> phi_rel - phi_rel0 &lt; b/2 <strong>then</strong>
   tau_c = 0;          // spring torque
   tau_d = 0;          // damper torque
   flange_b.tau = 0;
<strong>else</strong>
   tau_c = c_effective*(phi_rel - phi_rel0);    // spring torque
   tau_d = d_effective*<strong>der</strong>(phi_rel);            // damper torque
   flange_b.tau = <strong>if</strong> tau_c + tau_d &le; 0 <strong>then</strong> 0 <strong>else</strong> tau_c + tau_d;
<strong>end if</strong>;
</pre></blockquote>

<p>
Note, when sticking would occur (tau_c + tau_d &le; 0), then the contact torque
is explicitly set to zero.
</p>

<p>
This model of backlash is slightly different to the
<a href=\"modelica://Modelica.Mechanics.Rotational.Components.ElastoBacklash\">ElastoBacklash</a>
component:
</p>

<ul>
<li> An event occurs when contact occurs or when contact is released (contrary to the ElastoBacklash component).</li>
<li> When contact occurs, the torque changes discontinuously, due to the damping.
     The damping is larger as for the ElastoBacklash component (for the same damping coefficient),
    because the ElastoBacklash component has a heuristic to avoid the discontinuity of the torque when contact occurs.</li>
<li> For some models, the ElastoBacklash2 component leads to faster simulations
      (as compared when using the ElastBacklash component).</li>
</ul>

<p>
See also the discussion
<a href=\"modelica://Modelica.Mechanics.Rotational.UsersGuide.StateSelection\">State Selection</a>
in the User's Guide of the Rotational library.
</p>
</html>"),
    Icon(coordinateSystem(preserveAspectRatio=false,
      extent={{-100,-100},{100,100}}),
      graphics={
        Line(points={{-80,32},{-58,32},{-48,0},{-34,61},{-20,0},{-8,60},{0,30},{20,30}}, color={64,64,64}, thickness=0.5),
        Rectangle(extent={{-60,-10},{-10,-50}}, fillColor={192,192,192}, fillPattern=FillPattern.Solid, lineColor={64,64,64}, lineThickness=0.5),
        Line(points={{-60,-50},{0,-50}}, color={64,64,64}, thickness=0.5),
        Line(points={{-60,-10},{0,-10}}, color={64,64,64}, thickness=0.5),
        Line(points={{-10,-30},{20,-30}}, color={64,64,64}, thickness=0.5),
        Line(points={{-80,-30},{-60,-30}}, color={64,64,64}, thickness=0.5),
        Line(points={{-80,32},{-80,-30}}, color={64,64,64}, thickness=0.5),
        Line(points={{20,30},{20,-30}}, color={64,64,64}, thickness=0.5),
        Line(points={{-90,0},{-80,0}}, color={64,64,64}, thickness=0.5),
        Line(points={{90,0},{80,0}}, color={64,64,64}, thickness=0.5),
        Line(points={{20,0},{60,0},{60,-30}}, color={64,64,64}, thickness=0.5),
        Line(points={{40,-12},{40,-40},{80,-40},{80,0}}, color={64,64,64}, thickness=0.5),
        Text(extent={{-150,-130},{150,-90}}, textString="b=%b"),
        Text(extent={{-150,155},{150,125}}, textColor={0,0,255}, textString="%name"),
        Text(extent={{-152,-92},{148,-52}}, textString="c_effective=%c_effective"),
        Line(visible=useHeatPort, points={{-100,-100},{-100,-43},{-34,-43}}, color={191,0,0}, pattern=LinePattern.Dot, thickness=0.5),
        Text(extent={{20,48},{80,10}}, textColor={95,95,95}, textString="2"),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableElastoBacklash2;
