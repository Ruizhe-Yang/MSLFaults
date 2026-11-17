within MSLFaults.Mechanics.Translational.Components;
model FaultableRollingResistance "Rolling resistance with amplitude and periodic-force faults"
  extends Modelica.Mechanics.Translational.Interfaces.PartialForce;
  import Modelica.Constants.pi;
  parameter Modelica.Units.SI.Force fWeight(start=0) "Wheel load due to gravity";
  parameter Boolean useCrInput=false "Enable signal input for Cr";
  parameter Real CrConstant=0.01 "Constant rolling resistance coefficient"
    annotation(Dialog(enable=not useCrInput));
  parameter Boolean useInclinationInput=false "Enable signal input for inclination";
  parameter Real inclinationConstant=0 "Constant inclination = tan(angle)"
    annotation(Dialog(enable=not useInclinationInput));
  parameter Modelica.Blocks.Types.Regularization reg=Modelica.Blocks.Types.Regularization.Exp
    "Type of regularisation" annotation(Evaluate=true);
  parameter Modelica.Units.SI.Velocity v0(final min=Modelica.Constants.eps)=0.1
    "Regularisation below v0";
  Modelica.Units.SI.Velocity v
    "Velocity of flange with respect to support (= der(s))";
  Modelica.Units.SI.Force f_nominal "Nominal rolling resistance without regularisation";
  Modelica.Blocks.Interfaces.RealInput inclination = inclination_internal if useInclinationInput
    "Inclination=tan(angle)"
    annotation (Placement(transformation(extent={{-20,-20},{20,20}},
        origin={-120,60})));
  Modelica.Blocks.Interfaces.RealInput cr = Cr_internal if useCrInput
    "Rolling resistance coefficient"
    annotation (Placement(transformation(extent={{-20,-20},{20,20}},
        origin={-120,-60})));
  type FaultMode=enumeration(Normal "Nominal component relation", ResistanceLoss "Reduced rolling resistance", ResistanceIncrease "Increased rolling resistance", TireDamage "Periodic rolling-force amplitude", LockedWheel "Declared wheel-locking approximation");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault law";
  parameter Real severity(min=0,max=1)=1 "Scale of the activation envelope";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault request";
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal half-open fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a complete activation or recovery ramp";
  Real faultActivation(min=0,max=1) "Activation envelope used by the fault law";
  Real startActivation(min=0,max=1) "Activation envelope divided by positive severity";
  Real endActivation(min=0,max=1) "One while activation or its recovery envelope remains present";
  parameter Real resistanceScaleFault(min=0)=2 "Rolling-resistance multiplier at full increase effect";
  parameter Modelica.Units.SI.Force damageForce=10 "Amplitude of the periodic rolling-force term";
  parameter Modelica.Units.SI.Frequency damageFrequency=10 "Frequency of the prescribed damage-related sinusoid";
  Real resistanceScale "Multiplier on the nominal rolling-resistance amplitude";
protected
  Real Cr_internal "Rolling resistance coefficient";
  Real inclination_internal "Inclination";

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
  if not useCrInput then
    Cr_internal = CrConstant;
  end if;
  if not useInclinationInput then
    inclination_internal = inclinationConstant;
  end if;
  v = der(s);
  resistanceScale=if faultMode==FaultMode.ResistanceLoss then 1-faultActivation elseif faultMode==FaultMode.ResistanceIncrease then 1+faultActivation*(resistanceScaleFault-1) elseif faultMode==FaultMode.LockedWheel then 1+faultActivation*1000 else 1;
  f_nominal=-resistanceScale*Cr_internal*fWeight*cos(atan(inclination_internal)) + (if faultMode==FaultMode.TireDamage then faultActivation*damageForce*sin(2*Modelica.Constants.pi*damageFrequency*time) else 0);
  if reg==Modelica.Blocks.Types.Regularization.Exp then
    f = -f_nominal*(2/(1 + Modelica.Math.exp(-v/(0.01*v0)))-1);
  elseif reg==Modelica.Blocks.Types.Regularization.Sine then
    f = -f_nominal*smooth(1, (if abs(v)>=v0 then sign(v) else Modelica.Math.sin(pi/2*v/v0)));
  elseif reg==Modelica.Blocks.Types.Regularization.Linear then
    f = -f_nominal*(if abs(v)>=v0 then sign(v) else (v/v0));
  else//if reg==Modelica.Blocks.Types.Regularisation.CoSine
    f = -f_nominal*(if abs(v)>=v0 then sign(v) else sign(v)*(1 - Modelica.Math.cos(pi/2*v/v0)));
  end if;


  annotation (
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
      graphics={
        Ellipse(extent={{-60,60},{60,-60}}, lineColor={0,127,0}, fillColor={160,215,160}, fillPattern=FillPattern.Sphere, lineThickness=0.5),
        Ellipse(extent={{-40,40},{40,-40}}, lineColor={0,127,0}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, lineThickness=0.5),
        Text(extent={{-100,70},{-60,50}}, textColor={64,64,64}, textString="inc."),
        Text(extent={{-110,-50},{-70,-70}}, textColor={64,64,64}, textString="cr"),
        Rectangle(extent={{-2,40},{2,-40}}, lineColor={0,127,0}, fillColor={160,215,160}, fillPattern=FillPattern.Solid, lineThickness=0.5),
        Rectangle(extent={{-2,40},{2,-40}}, lineColor={0,127,0}, fillColor={160,215,160}, fillPattern=FillPattern.Solid, rotation=90, lineThickness=0.5),
        Rectangle(extent={{-2,40},{2,-40}}, lineColor={0,127,0}, fillColor={160,215,160}, fillPattern=FillPattern.Solid, rotation=135, lineThickness=0.5),
        Rectangle(extent={{-2,40},{2,-40}}, lineColor={0,127,0}, fillColor={160,215,160}, fillPattern=FillPattern.Solid, rotation=45, lineThickness=0.5),
        Ellipse(extent={{-10,10},{10,-10}}, lineColor={0,127,0}, fillColor={160,215,160}, fillPattern=FillPattern.Solid, lineThickness=0.5),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}, fontSize=0),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }),
    Documentation(info="<html><p>ResistanceLoss reduces the nominal amplitude towards zero; ResistanceIncrease and LockedWheel increase it. TireDamage adds a sinusoidal term before the speed-dependent direction regularisation. The inclination input is the tangent of the slope angle.</p><p>Set wheel load and rolling coefficient, supply optional cr and inclination inputs, and choose the low-speed regularisation.</p><p>LockedWheel raises rolling resistance without constraining wheel rotation. A periodic amplitude may change the force sign if it exceeds the nominal rolling term.</p><p>Nominal reference: <code>Modelica.Mechanics.Translational.Components.RollingResistance</code>. Activation uses the configured half-open fault window or the optional trigger; inspect <code>effectLevel</code> during recovery.</p></html>"));
end FaultableRollingResistance;
