within MSLFaults.Mechanics.Translational.Components;
model FaultableSpring "Linear 1D translational spring"
  extends Modelica.Mechanics.Translational.Interfaces.PartialCompliant;
  parameter Modelica.Units.SI.TranslationalSpringConstant c(final min=0, start=1)
    "Spring constant";
  parameter Modelica.Units.SI.Distance s_rel0=0 "Unstretched spring length";

  type FaultMode=enumeration(Normal "正常", StiffnessLoss "刚度下降", StiffnessIncrease "刚度增加", Broken "断裂", PreloadShift "自由长度漂移", Sticking "卡死");
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter Modelica.Units.SI.Time faultStartTime=1;
  parameter Modelica.Units.SI.Time faultEndTime=3;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05;
  Real faultActivation(min=0,max=1);
  Real startActivation(min=0,max=1);
  Real endActivation(min=0,max=1);
  parameter Modelica.Units.SI.TranslationalSpringConstant cFault=0.5*c;
  parameter Modelica.Units.SI.Position sRel0Fault=s_rel0+0.01;
  parameter Modelica.Units.SI.TranslationalSpringConstant cStuck=1e12;
  Modelica.Units.SI.TranslationalSpringConstant c_effective;
  Modelica.Units.SI.Position s_rel0_effective;

public
  parameter Boolean enabled=true "Enable the selected fault mode";
  parameter Boolean useExternalTrigger=false "Use trigger instead of the internal time window" annotation(Evaluate=true);
  Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger
    annotation(Placement(transformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270),
      iconTransformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270)));
  output Boolean active "Requested or latched fault";
  output Real effectLevel(min=0,max=1) "Current realized fault effect";
  output Integer activationCount "Number of rising activations";
  Modelica.Units.SI.Distance deflection;
  Modelica.Units.SI.Energy storedEnergy;
  Modelica.Units.SI.Power lossPower,parameterPower;
  parameter Real brokenStiffnessFraction(min=0,max=1)=1e-6 "Residual fraction after full fracture";
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
    Documentation(info="<html><p><b>Purpose</b> Linear 1D translational spring</p><p><b>Interfaces</b> Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger; output Boolean active \"Requested or latched fault\"; output Real effectLevel(min=0,max=1) \"Current realized fault effect\"; output Integer activationCount \"Number of rising activations\"</p><p><b>Parameters</b> parameter Modelica.Units.SI.TranslationalSpringConstant c(final min=0, start=1); parameter Modelica.Units.SI.Distance s_rel0=0 \"Unstretched spring length\"; parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter Modelica.Units.SI.Time faultStartTime=1; parameter Modelica.Units.SI.Time faultEndTime=3; parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05; parameter Modelica.Units.SI.TranslationalSpringConstant cFault=0.5*c; parameter Modelica.Units.SI.Position sRel0Fault=s_rel0+0.01; parameter Modelica.Units.SI.TranslationalSpringConstant cStuck=1e12; parameter Boolean enabled=true \"Enable the selected fault mode\"; parameter Boolean useExternalTrigger=false \"Use trigger instead of the internal time window\"</p><p><b>Behavior</b> Linear 1D translational spring</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.TranslationalStiffness\">MSLFaults.Examples.TranslationalStiffness</a></p><p><b>Purpose</b> Fault-enhanced Spring</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> Broken latches on first activation. Stored energy changes with stiffness or preload. Parameter work accounts for that exchange and is not routed as heat.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery. Broken latch until a new simulation. faultEndTime does not repair a latched fault.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Mechanics.Translational.Components.Spring</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.TranslationalStiffness\">MSLFaults.Examples.TranslationalStiffness</a></p><p>用法：将 FaultableSpring 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p>
<p>
A <em>linear 1D translational spring</em>. The component can be connected either
between two sliding masses, or between
a sliding mass and the housing (model Fixed), to describe
a coupling of the sliding mass with the housing via a spring.
</p>

</html>"),
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
