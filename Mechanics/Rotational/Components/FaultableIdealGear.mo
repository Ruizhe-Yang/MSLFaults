within MSLFaults.Mechanics.Rotational.Components;
model FaultableIdealGear "Ideal gear without inertia"
  extends Modelica.Mechanics.Rotational.Icons.Gear;
  extends
    Modelica.Mechanics.Rotational.Interfaces.PartialElementaryTwoFlangesAndSupport2;
  parameter Real ratio(start=1)
    "Transmission ratio (flange_a.phi/flange_b.phi)";
  Modelica.Units.SI.Angle phi_a
    "Angle between left shaft flange and support";
  Modelica.Units.SI.Angle phi_b
    "Angle between right shaft flange and support";

  type FaultMode=enumeration(Normal "正常", GearRatioError "传动比误差", ToothDamage "齿损伤扰动", LockedGear "齿轮锁死");
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter Modelica.Units.SI.Time faultStartTime=1;
  parameter Modelica.Units.SI.Time faultEndTime=3;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05;
  Real faultActivation(min=0,max=1);
  Real startActivation(min=0,max=1);
  Real endActivation(min=0,max=1);
  parameter Real ratioFault=0.9*ratio;
  parameter Modelica.Units.SI.RotationalDampingConstant lockDamping=1e9;
  parameter Modelica.Units.SI.Torque toothTorque=1;
  parameter Modelica.Units.SI.Frequency toothFrequency=20;
  Real ratio_effective;
  Modelica.Units.SI.Torque faultTorque;

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

  phi_a = flange_a.phi - phi_support;
  phi_b = flange_b.phi - phi_support;
  ratio_effective=if faultMode==FaultMode.GearRatioError then ratio+faultActivation*(ratioFault-ratio) else ratio;
  faultTorque=if faultMode==FaultMode.ToothDamage then faultActivation*toothTorque*sin(2*Modelica.Constants.pi*toothFrequency*time)
    elseif faultMode==FaultMode.LockedGear then faultActivation*lockDamping*der(phi_a-ratio_effective*phi_b) else 0;
  phi_a=ratio_effective*phi_b;
  0=ratio_effective*flange_a.tau+flange_b.tau+faultTorque;


  annotation (
    Documentation(info="<html><p><b>Purpose</b> Ideal gear without inertia</p><p><b>Interfaces</b> Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger; output Boolean active \"Requested or latched fault\"; output Real effectLevel(min=0,max=1) \"Current realized fault effect\"; output Integer activationCount \"Number of rising activations\"</p><p><b>Parameters</b> parameter Real ratio(start=1); parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter Modelica.Units.SI.Time faultStartTime=1; parameter Modelica.Units.SI.Time faultEndTime=3; parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05; parameter Real ratioFault=0.9*ratio; parameter Modelica.Units.SI.RotationalDampingConstant lockDamping=1e9; parameter Modelica.Units.SI.Torque toothTorque=1; parameter Modelica.Units.SI.Frequency toothFrequency=20; parameter Boolean enabled=true \"Enable the selected fault mode\"; parameter Boolean useExternalTrigger=false \"Use trigger instead of the internal time window\"</p><p><b>Behavior</b> Ideal gear without inertia</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.RotationalDamping\">MSLFaults.Examples.RotationalDamping</a></p><p><b>Purpose</b> Fault-enhanced IdealGear</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> The established physical equations and fault modes use the common activation and severity law.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Mechanics.Rotational.Components.IdealGear</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.RotationalDamping\">MSLFaults.Examples.RotationalDamping</a></p><p>用法：将 FaultableIdealGear 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p>
<p>
This element characterizes any type of gear box which is fixed in the
ground and which has one driving shaft and one driven shaft.
The gear is <strong>ideal</strong>, i.e., it does not have inertia, elasticity, damping
or backlash. If these effects have to be considered, the gear has to be
connected to other elements in an appropriate way.
</p>

</html>"),
       Icon(coordinateSystem(preserveAspectRatio=true,
      extent={{-100,-100},{100,100}}),
      graphics={
        Text(extent={{-153,155},{147,125}}, textColor={0,0,255}, textString="%name"),
        Text(extent={{-146,-49},{154,-79}}, textString="ratio=%ratio"),
        Text(extent={{108,94},{139,62}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableIdealGear;
