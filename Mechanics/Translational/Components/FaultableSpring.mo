within MSLFaults.Mechanics.Translational.Components;
model FaultableSpring "Translational spring with stiffness, preload and latched fracture effects"
  extends Modelica.Mechanics.Translational.Interfaces.PartialCompliant;
  parameter Modelica.Units.SI.TranslationalSpringConstant c(final min=0, start=1)
    "Spring constant";
  parameter Modelica.Units.SI.Distance s_rel0=0 "Unstretched spring length";

  type FaultMode=enumeration(Normal "Nominal component relation", StiffnessLoss "Reduced stiffness", StiffnessIncrease "Configured stiffness increase", Broken "Latched fracture effect", PreloadShift "Changed unstretched coordinate", Sticking "Increased finite spring stiffness");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault law";
  parameter Real severity(min=0,max=1)=1 "Scale of the activation envelope";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault request";
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal half-open fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a complete activation or recovery ramp";
  Real faultActivation(min=0,max=1) "Activation envelope used by the fault law";
  Real startActivation(min=0,max=1) "Activation envelope divided by positive severity";
  Real endActivation(min=0,max=1) "One while activation or its recovery envelope remains present";
  parameter Modelica.Units.SI.TranslationalSpringConstant cFault=0.5*c "Stiffness target for the selected fault law";
  parameter Modelica.Units.SI.Position sRel0Fault=s_rel0+0.01 "Unstretched length or contact coordinate at full shift effect";
  parameter Modelica.Units.SI.TranslationalSpringConstant cStuck=1e12 "Finite stiffness at full sticking effect";
  Modelica.Units.SI.TranslationalSpringConstant c_effective "Stiffness used by the constitutive relation";
  Modelica.Units.SI.Position s_rel0_effective "Effective unstretched length or contact threshold";

public
  parameter Boolean enabled=true "Enable activation of the selected fault mode";
  parameter Boolean useExternalTrigger=false "Use trigger in place of the internal fault window" annotation(Evaluate=true);
  Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger "External Boolean activation request"
    annotation(Placement(transformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270),
      iconTransformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270)));
  output Boolean active "Requested or latched activation state";
  output Real effectLevel(min=0,max=1) "Severity-scaled activation envelope";
  output Integer activationCount "Number of rising activations, including an initially active request";
  Modelica.Units.SI.Distance deflection "Relative coordinate measured from its effective unstretched value";
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
  c_effective=if faultMode==FaultMode.StiffnessLoss or faultMode==FaultMode.StiffnessIncrease then c+faultActivation*(cFault-c) elseif faultMode==FaultMode.Broken then c*(1-faultActivation*(1-brokenStiffnessFraction)) elseif faultMode==FaultMode.Sticking then c+faultActivation*(cStuck-c) else c;
  s_rel0_effective=if faultMode==FaultMode.PreloadShift then s_rel0+faultActivation*(sRel0Fault-s_rel0) else s_rel0;
  f=c_effective*(s_rel-s_rel0_effective);

  deflection=s_rel-s_rel0_effective;
  storedEnergy=0.5*c_effective*deflection^2;
  lossPower=0;
  parameterPower=der(storedEnergy)-f*der(s_rel);
  assert(c_effective>=0,"Stiffness must remain nonnegative.");
  annotation (
    Documentation(info="<html><p>StiffnessLoss and StiffnessIncrease share cFault. PreloadShift changes the unstretched length; Sticking raises stiffness towards cStuck. Broken latches and retains c*brokenStiffnessFraction at full severity. parameterPower is the stored-energy rate minus mechanical port power.</p><p>Set c and s_rel0, configure cFault or sRel0Fault, and inspect f, storedEnergy and parameterPower.</p><p>Set cFault&gt;c to obtain increased stiffness: the default is 0.5*c. The parameter-power residual is not dissipative heat.</p><p>Nominal reference: <code>Modelica.Mechanics.Translational.Components.Spring</code>. Activation uses the configured half-open fault window or the optional trigger; inspect <code>effectLevel</code> during recovery.</p></html>"),
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
            100,100}}),
      graphics={
        Line(points={{-60,-90},{20,-90}}, color={95,127,95}, thickness=0.5),
        Polygon(points={{50,-90},{20,-80},{20,-100},{50,-90}}, lineColor={95,127,95}, fillColor={95,127,95}, fillPattern=FillPattern.Solid, lineThickness=0.5),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Line(points={{-98,0},{-60,0},{-44,-30},{-16,30},{14,-30},{44,30},{60,0},{100,0}}, color={0,127,0}, thickness=0.5),
        Text(extent={{-150,-45},{150,-75}}, textString="c=%c"),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableSpring;
