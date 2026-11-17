within MSLFaults.Mechanics.Rotational.Components;
model FaultableBrake "Rotational brake with separate static and sliding friction faults"
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
  Modelica.Units.SI.AngularVelocity w "Angular velocity relative to the support";
  Modelica.Units.SI.AngularAcceleration a
    "Angular acceleration relative to the support";

  Real mu0 "Friction coefficient for w=0 and forward sliding";
  Modelica.Units.SI.Force fn "Normal force (=fn_max*f_normalized)";

  // Constant auxiliary variable
  Modelica.Blocks.Interfaces.RealInput f_normalized
    "Normalised force signal 0..1 (normal force = fn_max*f_normalized; brake is active if > 0)"
    annotation (Placement(transformation(
        origin={0,110},
        extent={{20,-20},{-20,20}},
        rotation=90)));

  type FaultMode=enumeration(Normal "Nominal component relation", FrictionLoss "Reduced friction", FrictionIncrease "Increased friction", Slipping "Reduced sliding friction", Sticking "Increased static resistance");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault law";
  parameter Real severity(min=0,max=1)=1 "Scale of the activation envelope";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault request";
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal half-open fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a complete activation or recovery ramp";
  Real faultActivation(min=0,max=1) "Activation envelope used by the fault law";
  Real startActivation(min=0,max=1) "Activation envelope divided by positive severity";
  Real endActivation(min=0,max=1) "One while activation or its recovery envelope remains present";
  parameter Real frictionScaleFault(min=0,max=1)=0.5
    "Default target for frictionLossScale";
  parameter Real frictionLossScale(min=0,max=1)=frictionScaleFault "Friction multiplier at full loss effect";
  parameter Real frictionIncreaseScale(min=1)=2 "Friction multiplier at full increase effect";
  parameter Real slipDynamicScale(min=0,max=1)=0.2 "Sliding-friction multiplier at full slip effect";
  parameter Real stickingStaticScale(min=1)=1000 "Static-friction multiplier at full sticking effect";
  Real dynamicScale "Multiplier on the sliding-friction law";
  Real staticScale "Multiplier on the static-friction threshold";

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
      }), Documentation(info="<html><p>FrictionLoss and FrictionIncrease scale both static and sliding friction. Slipping reduces only sliding friction through slipDynamicScale; Sticking raises only the static threshold through stickingStaticScale. Normal force remains fn_max*f_normalized.</p><p>Supply f_normalized, set fn_max and mu_pos, and configure the relevant static or dynamic scale.</p><p>A non-positive normal force releases the brake even in Sticking mode; an increased static threshold is a finite friction approximation.</p><p>Nominal reference: <code>Modelica.Mechanics.Rotational.Components.Brake</code>. Activation uses the configured half-open fault window or the optional trigger; inspect <code>effectLevel</code> during recovery.</p></html>"));
end FaultableBrake;
