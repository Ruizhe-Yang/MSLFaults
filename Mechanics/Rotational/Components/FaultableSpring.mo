within MSLFaults.Mechanics.Rotational.Components;
model FaultableSpring "Rotational spring with stiffness, preload and latched fracture effects"
  extends Modelica.Mechanics.Rotational.Interfaces.PartialCompliant;
  parameter Modelica.Units.SI.RotationalSpringConstant c(final min=0, start=1.0e5)
    "Spring constant";
  parameter Modelica.Units.SI.Angle phi_rel0=0 "Unstretched spring angle";

  type FaultMode=enumeration(Normal "Nominal component relation", StiffnessDegradation "Reduced stiffness", StiffnessIncrease "Configured stiffness increase", Broken "Latched fracture effect", PreloadShift "Changed unstretched coordinate", Sticking "Increased finite spring stiffness");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault law";
  parameter Real severity(min=0,max=1)=1 "Scale of the activation envelope";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault request";
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal half-open fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a complete activation or recovery ramp";
  Real faultActivation(min=0,max=1) "Activation envelope used by the fault law";
  Real startActivation(min=0,max=1) "Activation envelope divided by positive severity";
  Real endActivation(min=0,max=1) "One while activation or its recovery envelope remains present";
  parameter Modelica.Units.SI.RotationalSpringConstant cDegraded=0.5*c "Stiffness at full degradation effect";
  parameter Modelica.Units.SI.RotationalSpringConstant cIncreased=2*c "Stiffness at full increase effect";
  parameter Modelica.Units.SI.Angle phiRel0Fault=phi_rel0+0.1 "Unstretched angle at full preload-shift effect";
  parameter Modelica.Units.SI.RotationalSpringConstant cStick=1e9 "Finite stiffness at full sticking effect";
  Modelica.Units.SI.RotationalSpringConstant c_effective "Stiffness used by the constitutive relation";
  Modelica.Units.SI.Angle phi_rel0_effective "Effective unstretched angle";

public
  parameter Boolean enabled=true "Enable activation of the selected fault mode";
  parameter Boolean useExternalTrigger=false "Use trigger in place of the internal fault window" annotation(Evaluate=true);
  Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger "External Boolean activation request"
    annotation(Placement(transformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270),
      iconTransformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270)));
  output Boolean active "Requested or latched activation state";
  output Real effectLevel(min=0,max=1) "Severity-scaled activation envelope";
  output Integer activationCount "Number of rising activations, including an initially active request";
  Modelica.Units.SI.Angle deflection "Relative coordinate measured from its effective unstretched value";
  Modelica.Units.SI.Energy storedEnergy "Stored mechanical energy from the effective parameters";
  Modelica.Units.SI.Power lossPower "Declared dissipation, zero for the elastic element",parameterPower "Energy-rate contribution associated with prescribed parameter changes";
  parameter Real brokenStiffnessFraction(min=0,max=1)=1e-6 "Residual stiffness fraction at full fracture effect";
protected
  MSLFaults.Utilities.Internal.Activation faultGate(enabled=enabled and faultMode<>FaultMode.Normal,
    permanent=faultMode==FaultMode.Broken,severity=severity,startTime=faultStartTime,endTime=faultEndTime,
    transitionTime=transitionTime,useExternalTrigger=useExternalTrigger);
initial equation
  assert(brokenStiffnessFraction>=0 and brokenStiffnessFraction<=1,"Invalid residual stiffness fraction.");
equation
  connect(trigger,faultGate.externalTrigger);
  if not useExternalTrigger then faultGate.externalTrigger=false; end if;
  active=faultGate.active;
  effectLevel=faultGate.effectLevel;
  activationCount=faultGate.activationCount;

  startActivation=if severity>0 then effectLevel/severity else 0;
  endActivation=if active or effectLevel>0 then 1 else 0;
  faultActivation=effectLevel;

  c_effective=if faultMode==FaultMode.StiffnessDegradation then c+faultActivation*(cDegraded-c)
    elseif faultMode==FaultMode.StiffnessIncrease then c+faultActivation*(cIncreased-c)
    elseif faultMode==FaultMode.Broken then c*(1-faultActivation*(1-brokenStiffnessFraction))
    elseif faultMode==FaultMode.Sticking then c+faultActivation*(cStick-c) else c;
  phi_rel0_effective=if faultMode==FaultMode.PreloadShift then phi_rel0+faultActivation*(phiRel0Fault-phi_rel0) else phi_rel0;
  tau=c_effective*(phi_rel-phi_rel0_effective);

  deflection=phi_rel-phi_rel0_effective;
  storedEnergy=0.5*c_effective*deflection^2;
  lossPower=0;
  parameterPower=der(storedEnergy)-tau*der(phi_rel);
  assert(c_effective>=0,"Stiffness must remain nonnegative.");
  annotation (
    Documentation(info="<html><p>Interpolate stiffness towards cDegraded, cIncreased or cStick, or shift the unstretched angle. Broken latches and reduces stiffness towards c*brokenStiffnessFraction at full severity. The stored energy follows the effective stiffness and deflection.</p><p>Set c and phi_rel0, choose the target stiffness or phiRel0Fault, and inspect tau, storedEnergy and parameterPower.</p><p>parameterPower is the energy-rate residual after mechanical port power; it is not dissipative heat. Broken remains active after deactivation.</p><p>Nominal reference: <code>Modelica.Mechanics.Rotational.Components.Spring</code>. Activation uses the configured half-open fault window or the optional trigger; inspect <code>effectLevel</code> during recovery.</p></html>"),
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
      graphics={
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Text(extent={{-150,-40},{150,-80}}, textString="c=%c"),
        Line(points={{-100,0},{-58,0},{-43,-30},{-13,30},{17,-30},{47,30},{62,0},{100,0}}, color={255,0,0}, thickness=0.8),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableSpring;
