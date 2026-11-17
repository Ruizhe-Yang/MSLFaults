within MSLFaults.Mechanics.Translational.Components;
model FaultableVehicle "Longitudinal vehicle model with additional fault forces"
  parameter Modelica.Units.SI.Mass m "Total mass of vehicle";
  parameter Modelica.Units.SI.Acceleration g=Modelica.Constants.g_n "Constant gravity acceleration";
  parameter Modelica.Units.SI.Inertia J "Total rotational inertia of drive train";
  parameter Modelica.Units.SI.Length R "Wheel radius";
  parameter Modelica.Units.SI.Area A(start=1) "Cross section of vehicle"
    annotation(Dialog(tab="Driving resistances", group="Drag resistance"));
  parameter Real Cd(start=0.5) "Drag resistance coefficient"
    annotation(Dialog(tab="Driving resistances", group="Drag resistance"));
  parameter Modelica.Units.SI.Density rho=1.2 "Density of air"
    annotation(Dialog(tab="Driving resistances", group="Drag resistance"));
  parameter Boolean useWindInput=false "Enable signal input for wind velocity"
    annotation(Dialog(tab="Driving resistances", group="Drag resistance"));
  parameter Modelica.Units.SI.Velocity vWindConstant=0 "Constant wind velocity"
    annotation(Dialog(tab="Driving resistances", group="Drag resistance", enable=not useWindInput));
  parameter Boolean useCrInput=false "Enable signal input for Cr"
    annotation(Dialog(tab="Driving resistances", group="Rolling resistance"));
  parameter Real CrConstant=0.015 "Constant rolling resistance coefficient"
    annotation(Dialog(tab="Driving resistances", group="Rolling resistance", enable=not useCrInput));
  parameter Modelica.Units.SI.Velocity vReg=1e-3 "Velocity for regularisation around 0"
    annotation(Dialog(tab="Driving resistances", group="Rolling resistance"));
  parameter Boolean useInclinationInput=false "Enable signal input for inclination"
    annotation(Dialog(tab="Driving resistances", group="Inclination resistance"));
  parameter Real inclinationConstant=0 "Constant inclination = tan(angle)"
    annotation(Dialog(tab="Driving resistances", group="Inclination resistance", enable=not useInclinationInput));
  Modelica.Units.SI.Position s(displayUnit="km", start=0)=mass.s "Position of vehicle";
  Modelica.Units.SI.Velocity v(displayUnit="km/h", start=0)=mass.v "Velocity of vehicle";
  Modelica.Units.SI.Acceleration a=mass.a "Acceleration of vehicle";
protected
  constant Modelica.Units.SI.Velocity vRef=1 "Reference velocity for air drag";
public
  Modelica.Mechanics.Translational.Sources.QuadraticSpeedDependentForce fDrag(
    final useSupport=true,
    final f_nominal=-Cd*A*rho*vRef^2/2,
    final ForceDirection=false,
    final v_nominal=vRef) "Drag resistance"
    annotation (Placement(transformation(extent={{50,-40},{70,-20}})));
  Modelica.Mechanics.Translational.Components.RollingResistance fRoll(
    final fWeight=m*g,
    final useCrInput=useCrInput,
    final CrConstant=CrConstant,
    final useInclinationInput=useInclinationInput,
    final inclinationConstant=inclinationConstant,
    final reg=Modelica.Blocks.Types.Regularization.Linear,
    final v0=vReg) "Rolling resistance"
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        origin={60,0})));
  Modelica.Mechanics.Translational.Sources.Force fGrav "Inclination resistance"
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        origin={60,30})));
  Modelica.Mechanics.Translational.Interfaces.Flange_b flangeT "Translational flange"
    annotation (Placement(transformation(extent={{90,10},{110,-10}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_a flangeR "Rotational flange"
    annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia(final J=J)
    annotation (Placement(transformation(extent={{-50,50},{-30,70}})));
  Modelica.Mechanics.Translational.Components.IdealRollingWheel idealRollingWheel(final radius=R)
    annotation (Placement(transformation(extent={{-10,50},{10,70}})));
  Modelica.Mechanics.Translational.Components.Mass mass(final m=m) annotation (Placement(transformation(extent={{30,50},
            {50,70}})));
  Modelica.Blocks.Interfaces.RealInput inclination if useInclinationInput
    "Inclination=tan(angle)"
    annotation (Placement(transformation(extent={{-20,-20},{20,20}},
        rotation=90,
        origin={-60,-120})));
  Modelica.Blocks.Interfaces.RealInput cr if useCrInput
    "Rolling resistance coefficient"
    annotation (Placement(transformation(extent={{-20,-20},{20,20}},
        rotation=90,
        origin={0,-120})));
  Modelica.Blocks.Interfaces.RealInput vWind(unit="m/s") if useWindInput
    "Wind velocity"
    annotation (Placement(transformation(extent={{-20,-20},{20,20}},
        rotation=90,
        origin={60,-120})));
  Modelica.Mechanics.Translational.Sources.Speed windSpeed(s(fixed=true))  annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={60,-60})));
  Modelica.Blocks.Math.Gain gravForceGain(final k=-m*g) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        origin={20,30})));
protected
  Modelica.Blocks.Sources.Constant constInclination(k=inclinationConstant) if not useInclinationInput
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-80,-90})));
  Modelica.Blocks.Sources.Constant constWindSpeed(k=vWindConstant) if not useWindInput
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={40,-90})));
public
  Modelica.Blocks.Math.Atan atan annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        origin={-40,30})));
  Modelica.Blocks.Math.Sin sin annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        origin={-10,30})));

  type FaultMode=enumeration(Normal "Nominal component relation", MassLoadIncrease "Additional acceleration-proportional load", DragIncrease "Additional aerodynamic drag", RollingResistanceIncrease "Additional rolling resistance", DrivetrainLock "Finite viscous vehicle resistance");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault law";
  parameter Real severity(min=0,max=1)=1 "Scale of the activation envelope";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault request";
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal half-open fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a complete activation or recovery ramp";
  Real faultActivation(min=0,max=1) "Activation envelope used by the fault law";
  Real startActivation(min=0,max=1) "Activation envelope divided by positive severity";
  Real endActivation(min=0,max=1) "One while activation or its recovery envelope remains present";
  parameter Modelica.Units.SI.Mass addedMass=0.2*m "Additional acceleration-proportional inertial load at full effect";
  parameter Real addedDragArea=0.2*Cd*A "Additional drag coefficient times area at full effect";
  parameter Modelica.Units.SI.Force addedRollingForce=0.2*CrConstant*m*g "Magnitude of additional rolling resistance at full effect";
  parameter Modelica.Units.SI.TranslationalDampingConstant drivetrainLockDamping=1e10 "Finite damping for the drivetrain-lock approximation";
  Modelica.Units.SI.Velocity relativeAirSpeed "Vehicle speed relative to the prescribed air motion";
  Modelica.Mechanics.Translational.Sources.Force faultForce(useSupport=false);
  Modelica.Blocks.Sources.RealExpression faultForceCommand(y=
    if faultMode==FaultMode.MassLoadIncrease then -faultActivation*addedMass*a
    elseif faultMode==FaultMode.DragIncrease then -faultActivation*rho*addedDragArea*relativeAirSpeed*abs(relativeAirSpeed)/2
    elseif faultMode==FaultMode.RollingResistanceIncrease then -faultActivation*addedRollingForce*tanh(v/max(vReg,Modelica.Constants.small))
    elseif faultMode==FaultMode.DrivetrainLock then -faultActivation*drivetrainLockDamping*v else 0);


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
  relativeAirSpeed=der(mass.flange_b.s-windSpeed.flange.s);
  connect(faultForceCommand.y,faultForce.f);
  connect(faultForce.flange,flangeT);
  connect(idealRollingWheel.flangeT, mass.flange_a)
    annotation (Line(points={{10,60},{30,60}},
                                             color={0,127,0}));
  connect(constWindSpeed.y, windSpeed.v_ref)
    annotation (Line(points={{51,-90},{60,-90},{60,-72}}, color={0,0,127}));
  connect(vWind, windSpeed.v_ref)
    annotation (Line(points={{60,-120},{60,-72}}, color={0,0,127}));
  connect(fDrag.support, windSpeed.flange)
    annotation (Line(points={{60,-40},{60,-50}}, color={0,127,0}));
  connect(mass.flange_b, flangeT) annotation (Line(points={{50,60},{80,60},{80,0},{100,0}},
                    color={0,127,0}));
  connect(sin.u, atan.y)
    annotation (Line(points={{-22,30},{-29,30}},   color={0,0,127}));
  connect(gravForceGain.u, sin.y) annotation (Line(points={{8,30},{1,30}},     color={0,0,127}));
  connect(gravForceGain.y, fGrav.f)
    annotation (Line(points={{31,30},{48,30}}, color={0,0,127}));
  connect(mass.flange_b, fDrag.flange) annotation (Line(points={{50,60},{80,60},{80,-30},{70,-30}},
                                  color={0,127,0}));
  connect(mass.flange_b, fRoll.flange) annotation (Line(points={{50,60},{80,60},{80,0},{70,0}},
                              color={0,127,0}));
  connect(mass.flange_b, fGrav.flange) annotation (Line(points={{50,60},{80,60},{80,30},{70,30}},
                                color={0,127,0}));
  connect(inertia.flange_b, idealRollingWheel.flangeR)
    annotation (Line(points={{-30,60},{-10,60}}, color={0,0,0}));
  connect(flangeR, inertia.flange_a) annotation (Line(points={{-100,0},{-80,0},{
          -80,60},{-50,60}}, color={0,0,0}));
  connect(cr,fRoll.cr)
    annotation (Line(points={{0,-120},{0,-6},{48,-6}}, color={0,0,127}));
  connect(inclination, fRoll.inclination)
    annotation (Line(points={{-60,-120},{-60,6},{48,6}}, color={0,0,127}));
  connect(inclination, atan.u) annotation (Line(points={{-60,-120},{-60,30},
          {-52,30}}, color={0,0,127}));
  connect(atan.u, constInclination.y) annotation (Line(points={{-52,30},{
          -60,30},{-60,-90},{-69,-90}}, color={0,0,127}));


  annotation (Icon(coordinateSystem(preserveAspectRatio=false),
      graphics={
        Line(points={{-80,-70},{80,-70}}, color={0,127,0}, thickness=0.5),
        Line(points={{-80,0},{85.607,-1.19754}}, color={0,127,0}, origin={-3,-49}, rotation=15, thickness=0.5),
        Ellipse(extent={{-130,-120},{-30,-20}}, lineColor={0,127,0}, startAngle=0, endAngle=15, closure=EllipseClosure.None, lineThickness=0.5),
        Polygon(points={{-70,15},{-70,-25},{50,-25},{50,15},{38,25},{-60,25},{-70,15}}, lineColor={0,127,0}, fillColor={160,215,160}, fillPattern=FillPattern.Solid, origin={0,-9}, rotation=15, lineThickness=0.5),
        Line(points={{-40,0},{40,0}}, color={95,127,95}, origin={-40,32}, rotation=15, thickness=0.5),
        Polygon(points={{15,0},{-15,10},{-15,-10},{15,0}}, lineColor={95,127,95}, fillColor={95,127,95}, fillPattern=FillPattern.Solid, origin={11,46}, rotation=15, lineThickness=0.5),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Polygon(points={{-20,0},{0,10},{0,4},{20,4},{20,-4},{0,-4},{0,-10},{-20,0}}, lineColor={0,127,0}, fillColor={160,215,160}, fillPattern=FillPattern.Solid, origin={68,18}, rotation=15, lineThickness=0.5),
        Text(extent={{-80,-80},{-40,-100}}, textColor={64,64,64}, textString="inc."),
        Text(extent={{-20,-80},{20,-100}}, textColor={64,64,64}, textString="cr"),
        Text(extent={{30,-80},{90,-100}}, textColor={64,64,64}, textString="wind"),
        Ellipse(extent={{-50,-34},{-26,-58}}, lineColor={0,127,0}, fillPattern=FillPattern.Sphere, fillColor={160,215,160}, lineThickness=0.5),
        Ellipse(extent={{20,-16},{44,-40}}, lineColor={0,127,0}, fillPattern=FillPattern.Sphere, fillColor={160,215,160}, lineThickness=0.5),
        Ellipse(extent={{26,-22},{38,-34}}, lineColor={0,127,0}, fillPattern=FillPattern.Solid, fillColor={255,255,255}, lineThickness=0.5),
        Ellipse(extent={{-44,-40},{-32,-52}}, lineColor={0,127,0}, fillPattern=FillPattern.Solid, fillColor={255,255,255}, lineThickness=0.5),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }),
    Documentation(info="<html><p>MassLoadIncrease applies an acceleration-proportional opposing force. DragIncrease uses relative air speed; RollingResistanceIncrease uses a tanh-regularised force; DrivetrainLock adds finite damping. The nominal mass, inertia and ideal wheel remain in the assembly.</p><p>Configure m, J, R, drag and rolling data; connect flangeR or flangeT and supply any enabled wind or slope inputs.</p><p>addedMass changes the inertial force only; nominal gravity and rolling-load terms remain based on m. The fault-force source has no explicit support or heat port.</p><p>Nominal reference: <code>Modelica.Mechanics.Translational.Components.Vehicle</code>. Activation uses the configured half-open fault window or the optional trigger; inspect <code>effectLevel</code> during recovery.</p></html>"));
end FaultableVehicle;
