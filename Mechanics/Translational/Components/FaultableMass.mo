within MSLFaults.Mechanics.Translational.Components;
model FaultableMass "Translational mass with momentum-based parameter variation"
  parameter Modelica.Units.SI.Mass m(min=0, start=1) "Mass of the sliding mass";
  parameter StateSelect stateSelect=StateSelect.default
    "Priority to use s and v as states" annotation (Dialog(tab="Advanced"));
  extends Modelica.Mechanics.Translational.Interfaces.PartialRigid(L=0,s(start=0, stateSelect=
          stateSelect));
  Modelica.Units.SI.Velocity v(start=0, stateSelect=stateSelect)
    "Absolute velocity of component";
  Modelica.Units.SI.Acceleration a(start=0) "Absolute acceleration of component";

  type FaultMode=enumeration(Normal "Nominal component relation", MassDrift "Changed effective mass", MassLoss "Reduced effective mass", LockedMass "Finite viscous resistance to mass motion");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault law";
  parameter Real severity(min=0,max=1)=1 "Scale of the activation envelope";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault request";
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal half-open fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a complete activation or recovery ramp";
  Real faultActivation(min=0,max=1) "Activation envelope used by the fault law";
  Real startActivation(min=0,max=1) "Activation envelope divided by positive severity";
  Real endActivation(min=0,max=1) "One while activation or its recovery envelope remains present";
  parameter Modelica.Units.SI.Mass mFault=0.5*m "Mass at full parameter-change effect";
  parameter Modelica.Units.SI.TranslationalDampingConstant lockDamping=1e12 "Finite viscous coefficient used by the locking approximation";
  Modelica.Units.SI.Mass m_effective "Effective mass used by the force relation";
  Modelica.Units.SI.Force lockForce "Viscous force opposing the absolute body velocity";

public
  parameter Boolean enabled=true "Enable activation of the selected fault mode";
  parameter Boolean useExternalTrigger=false "Use trigger in place of the internal fault window" annotation(Evaluate=true);
  Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger "External Boolean activation request"
    annotation(Placement(transformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270),
      iconTransformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270)));
  output Boolean active "Requested or latched activation state";
  output Real effectLevel(min=0,max=1) "Severity-scaled activation envelope";
  output Integer activationCount "Number of rising activations, including an initially active request";
  Modelica.Units.SI.Momentum momentum "Momentum associated with the effective inertial parameter";
  Modelica.Units.SI.Energy storedEnergy "Stored mechanical energy from the effective parameters";
  Modelica.Units.SI.Power lossPower "Power dissipated by the finite locking resistance",parameterPower "Energy-rate contribution associated with prescribed parameter changes",portPower "Mechanical power entering through the flanges";
protected
  MSLFaults.Utilities.Internal.Activation faultGate(enabled=enabled and faultMode<>FaultMode.Normal,
    permanent=false,severity=severity,startTime=faultStartTime,endTime=faultEndTime,
    transitionTime=transitionTime,useExternalTrigger=useExternalTrigger);
initial equation
  assert(m>=0 and lockDamping>=0,"The nominal inertial and damping terms must be nonnegative.");
  assert(not (faultMode==FaultMode.MassDrift or faultMode==FaultMode.MassLoss) or (m>0 and mFault>0),"Varying inertia requires positive endpoint parameters.");
equation
  connect(trigger,faultGate.externalTrigger);
  if not useExternalTrigger then faultGate.externalTrigger=false; end if;
  active=faultGate.active;
  effectLevel=faultGate.effectLevel;
  activationCount=faultGate.activationCount;

  startActivation=if severity>0 then effectLevel/severity else 0;
  endActivation=if active or effectLevel>0 then 1 else 0;
  faultActivation=effectLevel;
  v = der(s);
  a = der(v);
  m_effective=if faultMode==FaultMode.MassDrift or faultMode==FaultMode.MassLoss then m+faultActivation*(mFault-m) else m;
  lockForce=if faultMode==FaultMode.LockedMass then faultActivation*lockDamping*v else 0;
  der(momentum)+lockForce=flange_a.f+flange_b.f;

  momentum=m_effective*v;
  storedEnergy=0.5*momentum*v;
  lossPower=lockForce*v;
  portPower=(flange_a.f+flange_b.f)*v;
  parameterPower=der(storedEnergy)-portPower+lossPower;
  annotation (
    Documentation(info="<html><p>MassDrift and MassLoss use the same interpolation from m to mFault. The force balance differentiates momentum=m_effective*v. LockedMass adds finite viscous resistance. parameterPower=der(storedEnergy)-portPower+lossPower reports the prescribed parameter contribution.</p><p>Set m and initial position and velocity; select mFault or lockDamping and inspect momentum and the energy diagnostics.</p><p>Varying mass requires positive endpoint values. The model provides no inlet or outlet for material flow associated with a mass change.</p><p>Nominal reference: <code>Modelica.Mechanics.Translational.Components.Mass</code>. Activation uses the configured half-open fault window or the optional trigger; inspect <code>effectLevel</code> during recovery.</p></html>"),
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
            100,100}}),
      graphics={
        Line(points={{-100,0},{100,0}}, color={0,127,0}, thickness=0.5),
        Rectangle(extent={{-55,-30},{56,30}}, fillPattern=FillPattern.Sphere, fillColor={160,215,160}, lineColor={0,127,0}, lineThickness=0.5),
        Polygon(points={{50,-90},{20,-80},{20,-100},{50,-90}}, lineColor={95,127,95}, fillColor={95,127,95}, fillPattern=FillPattern.Solid, lineThickness=0.5),
        Line(points={{-60,-90},{20,-90}}, color={95,127,95}, thickness=0.5),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}, fillColor={110,210,110}, fillPattern=FillPattern.Solid),
        Text(extent={{-150,-45},{150,-75}}, textString="m=%m", fillColor={110,221,110}, fillPattern=FillPattern.Solid, fontSize=0),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableMass;
