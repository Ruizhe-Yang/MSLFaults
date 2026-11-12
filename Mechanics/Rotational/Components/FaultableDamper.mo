within MSLFaults.Mechanics.Rotational.Components;
model FaultableDamper "Linear 1D rotational damper"
  extends
    Modelica.Mechanics.Rotational.Interfaces.PartialCompliantWithRelativeStates;
  parameter Modelica.Units.SI.RotationalDampingConstant d(final min=0, start=0)
    "Damping constant";
  extends
    Modelica.Thermal.HeatTransfer.Interfaces.PartialElementaryConditionalHeatPortWithoutT;
  type FaultMode=enumeration(Normal "正常", DampingLoss "阻尼下降", DampingIncrease "阻尼增加", Leakage "阻尼泄漏", Seizure "卡死");
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter Modelica.Units.SI.Time faultStartTime=1;
  parameter Modelica.Units.SI.Time faultEndTime=3;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05;
  Real faultActivation(min=0,max=1);
  Real startActivation(min=0,max=1);
  Real endActivation(min=0,max=1);
  parameter Modelica.Units.SI.RotationalDampingConstant dFault=0.5*d
    "Compatibility alias for earlier replacement scenarios";
  parameter Modelica.Units.SI.RotationalDampingConstant dLoss=dFault;
  parameter Modelica.Units.SI.RotationalDampingConstant dIncrease=2*d;
  parameter Modelica.Units.SI.RotationalDampingConstant dLeak=0.1*d;
  parameter Modelica.Units.SI.Time leakageTime(min=Modelica.Constants.small)=1;
  parameter Modelica.Units.SI.RotationalDampingConstant dSeized=1e9;
  Modelica.Units.SI.RotationalDampingConstant d_effective;
  Real leakageProgress(min=0,max=1);

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

  leakageProgress=min(1,max(0,faultGate.elapsedActiveTime/leakageTime));
  d_effective=if faultMode==FaultMode.DampingLoss then d+faultActivation*(dLoss-d)
    elseif faultMode==FaultMode.DampingIncrease then d+faultActivation*(dIncrease-d)
    elseif faultMode==FaultMode.Leakage then d+effectLevel*leakageProgress*(dLeak-d)
    elseif faultMode==FaultMode.Seizure then d+faultActivation*(dSeized-d) else d;
  tau=d_effective*w_rel;
  lossPower = tau*w_rel;

  assert(d_effective>=0,"A passive damper requires nonnegative damping.");
  annotation (
    Documentation(info="<html><p><b>Purpose</b> Linear 1D rotational damper</p><p><b>Interfaces</b> Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger; output Boolean active \"Requested or latched fault\"; output Real effectLevel(min=0,max=1) \"Current realized fault effect\"; output Integer activationCount \"Number of rising activations\"</p><p><b>Parameters</b> parameter Modelica.Units.SI.RotationalDampingConstant d(final min=0, start=0); parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter Modelica.Units.SI.Time faultStartTime=1; parameter Modelica.Units.SI.Time faultEndTime=3; parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05; parameter Modelica.Units.SI.RotationalDampingConstant dFault=0.5*d; parameter Modelica.Units.SI.RotationalDampingConstant dLoss=dFault; parameter Modelica.Units.SI.RotationalDampingConstant dIncrease=2*d; parameter Modelica.Units.SI.RotationalDampingConstant dLeak=0.1*d; parameter Modelica.Units.SI.Time leakageTime(min=Modelica.Constants.small)=1; parameter Modelica.Units.SI.RotationalDampingConstant dSeized=1e9</p><p><b>Behavior</b> Linear 1D rotational damper</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.RotationalDamping\">MSLFaults.Examples.RotationalDamping</a></p><p><b>Purpose</b> Fault-enhanced Damper</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> Dissipation is the product of the damper load and relative rate. Seizure is a finite high-damping approximation.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Mechanics.Rotational.Components.Damper</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.RotationalDamping\">MSLFaults.Examples.RotationalDamping</a></p><p>用法：将 FaultableDamper 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p>
<p>
<strong>Linear, velocity dependent damper</strong> element. It can be either connected
between an inertia or gear and the housing (component Fixed), or
between two inertia/gear elements.
</p>

<p>
See also the discussion
<a href=\"modelica://Modelica.Mechanics.Rotational.UsersGuide.StateSelection\">State Selection</a>
in the User's Guide of the Rotational library.
</p>
</html>"),
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
      graphics={
        Line(points={{-90,0},{-60,0}}, color={64,64,64}, thickness=0.5),
        Line(points={{-60,-30},{-60,30}}, color={64,64,64}, thickness=0.5),
        Line(points={{-60,-30},{60,-30}}, color={64,64,64}, thickness=0.5),
        Line(points={{-60,30},{60,30}}, color={64,64,64}, thickness=0.5),
        Rectangle(extent={{-60,30},{30,-30}}, fillColor={192,192,192}, fillPattern=FillPattern.Solid, lineColor={64,64,64}, lineThickness=0.5),
        Line(points={{30,0},{90,0}}, color={64,64,64}, thickness=0.5),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Text(extent={{-150,-50},{150,-90}}, textString="d=%d"),
        Line(visible=useHeatPort, points={{-100,-100},{-100,-40},{-20,-40},{-20,0}}, color={191,0,0}, pattern=LinePattern.Dot, thickness=0.5),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableDamper;
