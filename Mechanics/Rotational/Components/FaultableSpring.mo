within MSLFaults.Mechanics.Rotational.Components;
model FaultableSpring "Linear 1D rotational spring"
  extends Modelica.Mechanics.Rotational.Interfaces.PartialCompliant;
  parameter Modelica.Units.SI.RotationalSpringConstant c(final min=0, start=1.0e5)
    "Spring constant";
  parameter Modelica.Units.SI.Angle phi_rel0=0 "Unstretched spring angle";

  type FaultMode=enumeration(Normal "正常", StiffnessDegradation "刚度下降", StiffnessIncrease "刚度增加", Broken "断裂", PreloadShift "预紧位置漂移", Sticking "卡滞");
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter Modelica.Units.SI.Time faultStartTime=1;
  parameter Modelica.Units.SI.Time faultEndTime=3;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05;
  Real faultActivation(min=0,max=1);
  Real startActivation(min=0,max=1);
  Real endActivation(min=0,max=1);
  parameter Modelica.Units.SI.RotationalSpringConstant cDegraded=0.5*c;
  parameter Modelica.Units.SI.RotationalSpringConstant cIncreased=2*c;
  parameter Modelica.Units.SI.Angle phiRel0Fault=phi_rel0+0.1;
  parameter Modelica.Units.SI.RotationalSpringConstant cStick=1e9;
  Modelica.Units.SI.RotationalSpringConstant c_effective;
  Modelica.Units.SI.Angle phi_rel0_effective;

public
  parameter Boolean enabled=true "Enable the selected fault mode";
  parameter Boolean useExternalTrigger=false "Use trigger instead of the internal time window" annotation(Evaluate=true);
  Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger
    annotation(Placement(transformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270),
      iconTransformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270)));
  output Boolean active "Requested or latched fault";
  output Real effectLevel(min=0,max=1) "Current realized fault effect";
  output Integer activationCount "Number of rising activations";
  Modelica.Units.SI.Angle deflection;
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
    Documentation(info="<html><p><b>Purpose</b> Linear 1D rotational spring</p><p><b>Interfaces</b> Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger; output Boolean active \"Requested or latched fault\"; output Real effectLevel(min=0,max=1) \"Current realized fault effect\"; output Integer activationCount \"Number of rising activations\"</p><p><b>Parameters</b> parameter Modelica.Units.SI.RotationalSpringConstant c(final min=0, start=1.0e5); parameter Modelica.Units.SI.Angle phi_rel0=0 \"Unstretched spring angle\"; parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter Modelica.Units.SI.Time faultStartTime=1; parameter Modelica.Units.SI.Time faultEndTime=3; parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05; parameter Modelica.Units.SI.RotationalSpringConstant cDegraded=0.5*c; parameter Modelica.Units.SI.RotationalSpringConstant cIncreased=2*c; parameter Modelica.Units.SI.Angle phiRel0Fault=phi_rel0+0.1; parameter Modelica.Units.SI.RotationalSpringConstant cStick=1e9; parameter Boolean enabled=true \"Enable the selected fault mode\"</p><p><b>Behavior</b> Linear 1D rotational spring</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.RotationalDamping\">MSLFaults.Examples.RotationalDamping</a></p><p><b>Purpose</b> Fault-enhanced Spring</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> Broken latches on first activation. Stored energy changes with stiffness or preload. Parameter work accounts for that exchange and is not routed as heat.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery. Broken latch until a new simulation. faultEndTime does not repair a latched fault.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Mechanics.Rotational.Components.Spring</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.RotationalDamping\">MSLFaults.Examples.RotationalDamping</a></p><p>用法：将 FaultableSpring 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p>
<p>
A <strong>linear 1D rotational spring</strong>. The component can be connected either
between two inertias/gears to describe the shaft elasticity, or between
a inertia/gear and the housing (component Fixed), to describe
a coupling of the element with the housing via a spring.
</p>

</html>"),
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
      graphics={
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Text(extent={{-150,-40},{150,-80}}, textString="c=%c"),
        Line(points={{-100,0},{-58,0},{-43,-30},{-13,30},{17,-30},{47,30},{62,0},{100,0}}, color={255,0,0}, thickness=0.8),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableSpring;
