within MSLFaults.Mechanics.Translational.Components;
model FaultableSupportFriction "Support friction with configurable friction scaling"

  extends Modelica.Mechanics.Translational.Interfaces.PartialElementaryTwoFlangesAndSupport2;
  extends Modelica.Thermal.HeatTransfer.Interfaces.PartialElementaryConditionalHeatPortWithoutT;

  parameter Real f_pos[:, 2]=[0, 1]
    "Positive sliding friction characteristic [N] as function of v [m/s] (v>=0)";
  parameter Real peak(final min=1) = 1
    "Peak for maximum friction force at v==0 (f0_max = peak*f_pos[1,2])";
  extends Modelica.Mechanics.Translational.Interfaces.PartialFriction;

  Modelica.Units.SI.Position s "= flange_a.s - support.s";
  Modelica.Units.SI.Force f "Friction force";
  Modelica.Units.SI.Velocity v "Velocity used by the support-friction law";
  Modelica.Units.SI.Acceleration a "Acceleration used by the support-friction law";
  type FaultMode=enumeration(Normal "Nominal component relation", FrictionLoss "Reduced friction", FrictionIncrease "Increased friction", Wear "Configured wear-related friction change", Seizure "Increased finite mechanical resistance");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault law";
  parameter Real severity(min=0,max=1)=1 "Scale of the activation envelope";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault request";
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal half-open fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a complete activation or recovery ramp";
  Real faultActivation(min=0,max=1) "Activation envelope used by the fault law";
  Real startActivation(min=0,max=1) "Activation envelope divided by positive severity";
  Real endActivation(min=0,max=1) "One while activation or its recovery envelope remains present";
  parameter Real frictionScaleFault(min=0)=2 "Friction multiplier at full configured increase effect";
  parameter Real seizureScale(min=1)=1000 "Finite friction multiplier at full seizure effect";
  Real frictionScale "Effective friction multiplier";

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
    Documentation(info="<html><p>FrictionLoss scales the characteristic towards zero. FrictionIncrease and Wear share frictionScaleFault; Seizure uses seizureScale. The static threshold and force characteristic use the same scale, and lossPower reports frictional power.</p><p>Connect the rigidly linked flanges and optional support; set f_pos, peak and the selected friction scale.</p><p>Wear follows the activation envelope without an accumulated wear state. Seizure increases finite friction rather than imposing a new position constraint.</p><p>Nominal reference: <code>Modelica.Mechanics.Translational.Components.SupportFriction</code>. Activation uses the configured half-open fault window or the optional trigger; inspect <code>effectLevel</code> during recovery.</p></html>"),
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
