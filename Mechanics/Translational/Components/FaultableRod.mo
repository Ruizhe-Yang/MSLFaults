within MSLFaults.Mechanics.Translational.Components;
model FaultableRod "Rod without inertia"
  extends Modelica.Mechanics.Translational.Interfaces.PartialRigid;
  type FaultMode=enumeration(Normal "正常", AxialForceDisturbance "轴向力扰动", RodDamage "杆损伤周期力", RodLock "杆卡死");
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter Modelica.Units.SI.Time faultStartTime=1;
  parameter Modelica.Units.SI.Time faultEndTime=3;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05;
  Real faultActivation(min=0,max=1);
  Real startActivation(min=0,max=1);
  Real endActivation(min=0,max=1);
  parameter Modelica.Units.SI.Force forceFault=1;
  parameter Modelica.Units.SI.Frequency damageFrequency=20;
  Modelica.Units.SI.Force faultForceValue "故障引入的等效轴向外力";

public
  parameter Boolean enabled=true "Enable the selected fault mode";
  parameter Boolean useExternalTrigger=false "Use trigger instead of the internal time window" annotation(Evaluate=true);
  Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger
    annotation(Placement(transformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270),
      iconTransformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270)));
  output Boolean active "Requested or latched fault";
  output Real effectLevel(min=0,max=1) "Current realized fault effect";
  output Integer activationCount "Number of rising activations";

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
  faultForceValue=if faultMode==FaultMode.AxialForceDisturbance then faultActivation*forceFault elseif faultMode==FaultMode.RodDamage then faultActivation*forceFault*sin(2*Modelica.Constants.pi*damageFrequency*time) elseif faultMode==FaultMode.RodLock then -faultActivation*1e10*der(flange_a.s) else 0;
  0=flange_a.f+flange_b.f+faultForceValue;


  annotation (
    Documentation(info="<html><p><b>Purpose</b> Rod without inertia</p><p><b>Interfaces</b> Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger; output Boolean active \"Requested or latched fault\"; output Real effectLevel(min=0,max=1) \"Current realized fault effect\"; output Integer activationCount \"Number of rising activations\"</p><p><b>Parameters</b> parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter Modelica.Units.SI.Time faultStartTime=1; parameter Modelica.Units.SI.Time faultEndTime=3; parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05; parameter Modelica.Units.SI.Force forceFault=1; parameter Modelica.Units.SI.Frequency damageFrequency=20; parameter Boolean enabled=true \"Enable the selected fault mode\"; parameter Boolean useExternalTrigger=false \"Use trigger instead of the internal time window\"</p><p><b>Behavior</b> Rod without inertia</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.TranslationalStiffness\">MSLFaults.Examples.TranslationalStiffness</a></p><p><b>Purpose</b> Fault-enhanced Rod</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> The established physical equations and fault modes use the common activation and severity law.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Mechanics.Translational.Components.Rod</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.TranslationalStiffness\">MSLFaults.Examples.TranslationalStiffness</a></p><p>用法：将 FaultableRod 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p>
<p>
A translational rod <strong>without inertia</strong> and two rigidly connected flanges.
</p>
</html>"),
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
            100,100}}),
      graphics={
        Line(points={{-100,0},{100,0}}, color={0,127,0}, thickness=0.5),
        Polygon(points={{50,-90},{20,-80},{20,-100},{50,-90}}, lineColor={95,127,95}, fillColor={95,127,95}, fillPattern=FillPattern.Solid, lineThickness=0.5),
        Line(points={{-60,-90},{20,-90}}, color={95,127,95}, thickness=0.5),
        Rectangle(extent={{-60,10},{60,-10}}, lineColor={0,127,0}, fillColor={160,215,160}, fillPattern=FillPattern.Solid, lineThickness=0.5),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Text(extent={{-150,-30},{150,-60}}, textString="L=%L"),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableRod;
