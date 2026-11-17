within MSLFaults.Mechanics.Translational.Components;
model FaultableBrake "Translational brake with a fault-adjusted normal force"

  extends Modelica.Mechanics.Translational.Interfaces.PartialElementaryTwoFlangesAndSupport2;
  extends Modelica.Thermal.HeatTransfer.Interfaces.PartialElementaryConditionalHeatPortWithoutT;
  parameter Real mu_pos[:, 2]=[0, 0.5]
    "Positive sliding friction coefficient [-] as function of v_rel [m/s] (v_rel>=0)";
  parameter Real peak(final min=1) = 1
    "Peak for maximum value of mu at w==0 (mu0_max = peak*mu_pos[1,2])";
  parameter Real cgeo(final min=0) = 1
    "Geometry constant containing friction distribution assumption";
  parameter Modelica.Units.SI.Force fn_max(final min=0, start=1) "Maximum normal force";
  extends Modelica.Mechanics.Translational.Interfaces.PartialFriction;

  Modelica.Units.SI.Position s "Absolute position of flange_a and of flange_b";
  Modelica.Units.SI.Force f "Brake friction force";
  Modelica.Units.SI.Velocity v "Velocity used by the support-friction law";
  Modelica.Units.SI.Acceleration a "Acceleration used by the support-friction law";

  Real mu0 "Friction coefficient for v=0 and forward sliding";
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
  parameter Real frictionScaleFault(min=0)=0.5 "Normal-force multiplier at full friction-loss or slip effect";
  parameter Real stickingScale(min=1)=1000 "Friction multiplier at full sticking effect";
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
  mu0 = Modelica.Math.Vectors.interpolate(
        mu_pos[:, 1],
        mu_pos[:, 2],
        0,
        1);

  s = s_a;
  s = s_b;

  // velocity and acceleration of flanges flange_a and flange_b
  v = der(s);
  a = der(v);
  v_relfric = v;
  a_relfric = a;

  // Friction force, normal force and friction force for v_rel=0
  flange_a.f + flange_b.f - f = 0;
  frictionScale=if faultMode==FaultMode.FrictionLoss or faultMode==FaultMode.Slipping then 1+faultActivation*(frictionScaleFault-1) elseif faultMode==FaultMode.FrictionIncrease then 1+faultActivation elseif faultMode==FaultMode.Sticking then 1+faultActivation*(stickingScale-1) else 1;
  fn=fn_max*f_normalized*frictionScale;
  f0 = mu0*cgeo*fn;
  f0_max = peak*f0;
  free = fn <= 0;

  // Friction force
  f = if locked then sa*unitForce else if free then 0 else cgeo*fn*(if
    startForward then Modelica.Math.Vectors.interpolate(
        mu_pos[:, 1],
        mu_pos[:, 2],
        v,
        1) else if startBackward then -Modelica.Math.Vectors.interpolate(
        mu_pos[:, 1],
        mu_pos[:, 2],
        -v,
        1) else if pre(mode) == Forward then
    Modelica.Math.Vectors.interpolate(
        mu_pos[:, 1],
        mu_pos[:, 2],
        v,
        1) else -Modelica.Math.Vectors.interpolate(
        mu_pos[:, 1],
        mu_pos[:, 2],
        -v,
        1));

  lossPower = f*v_relfric;


  annotation (Documentation(info="<html><p>FrictionLoss and Slipping share a multiplier on the commanded normal force. FrictionIncrease uses 1+effectLevel; Sticking approaches stickingScale. The scaled force affects both static and sliding friction.</p><p>Supply f_normalized, set fn_max and mu_pos, and select frictionScaleFault or stickingScale; inspect f and lossPower.</p><p>A non-positive normal force releases the brake. Sticking is a finite friction increase, not an imposed position constraint.</p><p>Nominal reference: <code>Modelica.Mechanics.Translational.Components.Brake</code>. Activation uses the configured half-open fault window or the optional trigger; inspect <code>effectLevel</code> during recovery.</p></html>"),
       Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
            {100,100}}),
      graphics={
        Rectangle(extent={{-90,10},{90,-10}}, lineColor={0,127,0}, fillColor={160,215,160}, fillPattern=FillPattern.Solid, lineThickness=0.5),
        Polygon(points={{0,-30},{10,-50},{-10,-50},{0,-30}}, lineColor={0,0,127}, fillColor={0,0,127}, fillPattern=FillPattern.Solid, lineThickness=0.5),
        Polygon(points={{10,50},{-10,50},{0,30},{10,50}}, lineColor={0,0,127}, fillColor={0,0,127}, fillPattern=FillPattern.Solid, lineThickness=0.5),
        Line(points={{0,90},{0,50}}, color={0,0,127}, thickness=0.5),
        Rectangle(extent={{20,28},{30,22}}, lineColor={175,190,175}, fillColor={175,190,175}, fillPattern=FillPattern.Solid, lineThickness=0.5),
        Rectangle(extent={{20,-22},{30,-28}}, lineColor={175,190,175}, fillColor={175,190,175}, fillPattern=FillPattern.Solid, lineThickness=0.5),
        Rectangle(extent={{30,28},{36,-102}}, lineColor={175,190,175}, fillColor={175,190,175}, fillPattern=FillPattern.Solid, lineThickness=0.5),
        Rectangle(extent={{14,-96},{30,-102}}, lineColor={175,190,175}, fillColor={175,190,175}, fillPattern=FillPattern.Solid, lineThickness=0.5),
        Line(points={{0,-50},{0,-60},{-40,-50},{-40,48},{0,60},{0,90}}, color={0,0,127}, thickness=0.5),
        Text(extent={{-150,-120},{150,-160}}, textString="%name", textColor={0,0,255}),
        Line(visible=useHeatPort, points={{-100,-102},{-100,-16},{0,-16}}, color={191,0,0}, pattern=LinePattern.Dot, thickness=0.5),
        Rectangle(extent={{-20,30},{20,20}}, fillPattern=FillPattern.Solid, lineColor={64,64,64}, lineThickness=0.5),
        Rectangle(extent={{-20,-20},{20,-30}}, fillPattern=FillPattern.Solid, lineColor={64,64,64}, lineThickness=0.5),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableBrake;
