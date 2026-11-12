within MSLFaults.Mechanics.Translational.Components;
model FaultableMass "Sliding mass with inertia"
  parameter Modelica.Units.SI.Mass m(min=0, start=1) "Mass of the sliding mass";
  parameter StateSelect stateSelect=StateSelect.default
    "Priority to use s and v as states" annotation (Dialog(tab="Advanced"));
  extends Modelica.Mechanics.Translational.Interfaces.PartialRigid(L=0,s(start=0, stateSelect=
          stateSelect));
  Modelica.Units.SI.Velocity v(start=0, stateSelect=stateSelect)
    "Absolute velocity of component";
  Modelica.Units.SI.Acceleration a(start=0) "Absolute acceleration of component";

  type FaultMode=enumeration(Normal "正常", MassDrift "质量漂移", MassLoss "质量下降", LockedMass "质量块卡死");
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter Modelica.Units.SI.Time faultStartTime=1;
  parameter Modelica.Units.SI.Time faultEndTime=3;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05;
  Real faultActivation(min=0,max=1);
  Real startActivation(min=0,max=1);
  Real endActivation(min=0,max=1);
  parameter Modelica.Units.SI.Mass mFault=0.5*m;
  parameter Modelica.Units.SI.TranslationalDampingConstant lockDamping=1e12;
  Modelica.Units.SI.Mass m_effective;
  Modelica.Units.SI.Force lockForce;

public
  parameter Boolean enabled=true "Enable the selected fault mode";
  parameter Boolean useExternalTrigger=false "Use trigger instead of the internal time window" annotation(Evaluate=true);
  Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger
    annotation(Placement(transformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270),
      iconTransformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270)));
  output Boolean active "Requested or latched fault";
  output Real effectLevel(min=0,max=1) "Current realized fault effect";
  output Integer activationCount "Number of rising activations";
  Modelica.Units.SI.Momentum momentum;
  Modelica.Units.SI.Energy storedEnergy;
  Modelica.Units.SI.Power lossPower,parameterPower,portPower;
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
    Documentation(info="<html><p><b>Purpose</b> Sliding mass with inertia</p><p><b>Interfaces</b> Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger; output Boolean active \"Requested or latched fault\"; output Real effectLevel(min=0,max=1) \"Current realized fault effect\"; output Integer activationCount \"Number of rising activations\"</p><p><b>Parameters</b> parameter Modelica.Units.SI.Mass m(min=0, start=1) \"Mass of the sliding mass\"; parameter StateSelect stateSelect=StateSelect.default; parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter Modelica.Units.SI.Time faultStartTime=1; parameter Modelica.Units.SI.Time faultEndTime=3; parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05; parameter Modelica.Units.SI.Mass mFault=0.5*m; parameter Modelica.Units.SI.TranslationalDampingConstant lockDamping=1e12; parameter Boolean enabled=true \"Enable the selected fault mode\"; parameter Boolean useExternalTrigger=false \"Use trigger instead of the internal time window\"</p><p><b>Behavior</b> Sliding mass with inertia</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.TranslationalStiffness\">MSLFaults.Examples.TranslationalStiffness</a></p><p><b>Purpose</b> Fault-enhanced Mass</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> The variable inertial parameter is a behavioral approximation. Momentum is conserved in the absence of external force or torque. Locking adds finite dissipative resistance.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Mechanics.Translational.Components.Mass</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.TranslationalStiffness\">MSLFaults.Examples.TranslationalStiffness</a></p><p>用法：将 FaultableMass 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p>
<p>
Sliding mass with <em>inertia, without friction</em> and two rigidly connected flanges.
</p>
<p>
The sliding mass has the length L, the position coordinate s is in the middle.
Sign convention: A positive force at flange flange_a moves the sliding mass in the positive direction.
A negative force at flange flange_a moves the sliding mass to the negative direction.
</p>

</html>"),
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
