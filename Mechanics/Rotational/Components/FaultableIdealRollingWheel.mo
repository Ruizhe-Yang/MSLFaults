within MSLFaults.Mechanics.Rotational.Components;
model FaultableIdealRollingWheel
  "Simple 1-dim. model of an ideal rolling wheel without inertia"

  extends Modelica.Mechanics.Rotational.Interfaces.PartialElementaryRotationalToTranslational;
  parameter Modelica.Units.SI.Distance radius(start=0.3) "Wheel radius";

  type FaultMode=enumeration(Normal "正常", RadiusError "半径误差", Slip "滚动打滑", LockedWheel "车轮锁死");
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter Modelica.Units.SI.Time faultStartTime=1;
  parameter Modelica.Units.SI.Time faultEndTime=3;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05;
  Real faultActivation(min=0,max=1);
  Real startActivation(min=0,max=1);
  Real endActivation(min=0,max=1);
  parameter Modelica.Units.SI.Distance radiusFault=0.9*radius;
  parameter Real slipRatioFault(min=0,max=1)=0.2;
  Modelica.Units.SI.Distance radius_effective;
  Real rollingScale;

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

  radius_effective=if faultMode==FaultMode.RadiusError then radius+faultActivation*(radiusFault-radius) else radius;
  rollingScale=if faultMode==FaultMode.Slip then 1-faultActivation*slipRatioFault elseif faultMode==FaultMode.LockedWheel then 1-faultActivation else 1;
  (flangeR.phi-internalSupportR.phi)*radius_effective*rollingScale=flangeT.s-internalSupportT.s;
  0=radius_effective*flangeT.f+flangeR.tau;


  annotation (
    Icon(coordinateSystem(preserveAspectRatio=true,
        extent={{-100.0,-100.0},{100.0,100.0}}),
      graphics={
        Rectangle(lineColor={64,64,64}, fillColor={192,192,192}, fillPattern=FillPattern.HorizontalCylinder, extent={{-100,-10},{-46,10}}, lineThickness=0.5),
        Ellipse(lineColor={64,64,64}, fillColor={255,255,255}, fillPattern=FillPattern.HorizontalCylinder, extent={{-50,-80},{10,80}}, lineThickness=0.5),
        Rectangle(lineColor={64,64,64}, fillColor={255,255,255}, pattern=LinePattern.None, fillPattern=FillPattern.HorizontalCylinder, extent={{-20,-80},{12,80}}, lineThickness=0.5),
        Ellipse(lineColor={64,64,64}, fillColor={128,128,128}, fillPattern=FillPattern.Solid, extent={{-16,-80},{44,80}}, lineThickness=0.5),
        Ellipse(lineColor={192,192,192}, fillColor={192,192,192}, fillPattern=FillPattern.Solid, extent={{-2,-52},{34,52}}, lineThickness=0.5),
        Ellipse(lineColor={64,64,64}, fillColor={192,192,192}, fillPattern=FillPattern.HorizontalCylinder, extent={{12,-10},{20,10}}, lineThickness=0.5),
        Rectangle(lineColor={64,64,64}, fillColor={192,192,192}, pattern=LinePattern.None, fillPattern=FillPattern.HorizontalCylinder, extent={{16,-10},{60,10}}, lineThickness=0.5),
        Ellipse(fillColor={192,192,192}, fillPattern=FillPattern.Solid, extent={{56,-10},{64,10}}, lineColor={64,64,64}, lineThickness=0.5),
        Text(textColor={0,0,255}, extent={{-150,155},{150,125}}, textString="%name"),
        Polygon(lineColor={0,127,0}, fillColor={0,127,0}, fillPattern=FillPattern.Solid, points={{80,10},{80,26},{60,26},{60,20},{70,20},{70,-20},{60,-20},{60,-26},{80,-26},{80,-10},{90,-10},{90,10},{80,10}}, lineThickness=0.5),
        Line(points={{-100,-20},{-60,-20}}, color={64,64,64}, thickness=0.5),
        Line(points={{-100,-20},{-100,-100}}, color={64,64,64}, thickness=0.5),
        Line(points={{-100,20},{-60,20}}, color={64,64,64}, thickness=0.5),
        Line(points={{100,-90},{-40,-90}}, color={0,127,0}, thickness=0.5),
        Line(points={{70,-26},{70,-50},{100,-50},{100,-100}}, color={0,127,0}, thickness=0.5),
        Line(origin={-2.5,80}, points={{-17.5,0},{17.5,0}}, color={64,64,64}, thickness=0.5),
        Line(origin={-2.5,-80}, points={{-17.5,0},{17.5,0}}, color={64,64,64}, thickness=0.5),
        Line(origin={38,10}, points={{-22,0},{22,0}}, color={64,64,64}, thickness=0.5),
        Line(origin={38,-10}, points={{-22,0},{22,0}}, color={64,64,64}, thickness=0.5),
        Text(extent={{-150,-120},{150,-150}}, textString="radius=%radius"),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }),
                             Documentation(info="<html><p><b>Purpose</b> FaultableIdealRollingWheel</p><p><b>Interfaces</b> Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger; output Boolean active \"Requested or latched fault\"; output Real effectLevel(min=0,max=1) \"Current realized fault effect\"; output Integer activationCount \"Number of rising activations\"</p><p><b>Parameters</b> parameter Modelica.Units.SI.Distance radius(start=0.3) \"Wheel radius\"; parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter Modelica.Units.SI.Time faultStartTime=1; parameter Modelica.Units.SI.Time faultEndTime=3; parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05; parameter Modelica.Units.SI.Distance radiusFault=0.9*radius; parameter Real slipRatioFault(min=0,max=1)=0.2; parameter Boolean enabled=true \"Enable the selected fault mode\"; parameter Boolean useExternalTrigger=false \"Use trigger instead of the internal time window\"</p><p><b>Behavior</b> FaultableIdealRollingWheel</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.RotationalDamping\">MSLFaults.Examples.RotationalDamping</a></p><p><b>Purpose</b> Fault-enhanced IdealRollingWheel</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> The established physical equations and fault modes use the common activation and severity law.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Mechanics.Rotational.Components.IdealRollingWheel</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.RotationalDamping\">MSLFaults.Examples.RotationalDamping</a></p><p>用法：将 FaultableIdealRollingWheel 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p>
<p>
A simple kinematic model of a rolling wheel which has no inertia and
no rolling resistance. This component defines the kinematic constraint:
</p>

<blockquote><pre>
(flangeR.phi - internalSupportR.phi) * radius = (flangeT.s - internalSupportT.s);
</pre></blockquote>
</html>"));
end FaultableIdealRollingWheel;
