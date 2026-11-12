within MSLFaults.Electrical.Analog.Sources;
model FaultableStepVoltage "StepVoltage 独立故障增强模型"
  type FaultMode=enumeration(Normal "正常", OutputLoss "输出丢失", AmplitudeDrift "幅值漂移", AmplitudeDrop "幅值下降", OffsetDrift "偏置漂移", StuckOutput "输出卡死");
  parameter Modelica.Units.SI.Voltage V(start=1) "Height of step";
  parameter Modelica.Units.SI.Voltage offset=0 "Output offset";
  parameter Modelica.Units.SI.Time startTime=0 "Time offset";

  extends Modelica.Electrical.Analog.Interfaces.OnePort;
  extends Modelica.Electrical.Analog.Icons.VoltageSource;
  parameter FaultMode faultMode=FaultMode.Normal "故障模式";
  parameter Real severity(min=0,max=1)=1 "故障严重度";
  parameter Modelica.Units.SI.Time faultStartTime=1;
  parameter Modelica.Units.SI.Time faultEndTime=3;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05;
  parameter Real amplitudeFault=0.5 "幅值漂移目标比例";
  parameter Real offsetFault=0 "偏置漂移目标值";
  parameter Real frequencyFault=0.5 "频率漂移目标比例";
  parameter Real stuckOutput=0 "输出卡死值";
  Real faultActivation(min=0,max=1);
  Real startActivation(min=0,max=1);
  Real endActivation(min=0,max=1);
  Real nominalOutput;
  Real effectiveOutput;

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
  effectiveOutput = if faultMode == FaultMode.OutputLoss then nominalOutput*(1-faultActivation)
    elseif faultMode == FaultMode.AmplitudeDrift or faultMode == FaultMode.AmplitudeDrop then nominalOutput*(1+faultActivation*(amplitudeFault-1))
    elseif faultMode == FaultMode.OffsetDrift then nominalOutput+faultActivation*offsetFault
    elseif faultMode == FaultMode.StuckOutput then nominalOutput+faultActivation*(stuckOutput-nominalOutput)
    else nominalOutput;
  nominalOutput = offset + (if time < startTime then 0 else V);
  v = effectiveOutput;


  annotation (
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
            100}}),
      graphics={
        Line(points={{-70,-70},{0,-70},{0,70},{69,70}}, color={128,128,128}, thickness=0.65),
        Ellipse(extent={{-70,70},{70,-70}}, lineColor={255,0,0}, lineThickness=0.75),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }),
    Documentation(revisions="<html>
<ul>
<li><em> 1998   </em>
       by Christoph Clauss<br> initially implemented<br>
       </li>
</ul>
</html>",
        info="<html><p>用法：将 FaultableStepVoltage 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p>
<p>This voltage source uses the corresponding signal source of the Modelica.Blocks.Sources package. Care for the meaning of the parameters in the Blocks package. Furthermore, an offset parameter is introduced, which is added to the value calculated by the blocks source. The startTime parameter allows to shift the blocks source behavior on the time axis.</p>
<p>
<img src=\"modelica://Modelica/Resources/Images/Electrical/Analog/Sources/StepVoltage.png\"
     alt=\"StepVoltage.png\">
</p>
</html>"));
  annotation(Documentation(info="<html><p><b>Purpose</b> StepVoltage 独立故障增强模型</p><p><b>Interfaces</b> extends Modelica.Electrical.Analog.Interfaces.OnePort; Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger; output Boolean active \"Requested or latched fault\"; output Real effectLevel(min=0,max=1) \"Current realized fault effect\"; output Integer activationCount \"Number of rising activations\"</p><p><b>Parameters</b> parameter Modelica.Units.SI.Voltage V(start=1) \"Height of step\"; parameter Modelica.Units.SI.Voltage offset=0 \"Output offset\"; parameter Modelica.Units.SI.Time startTime=0 \"Time offset\"; parameter FaultMode faultMode=FaultMode.Normal \"故障模式\"; parameter Real severity(min=0,max=1)=1 \"故障严重度\"; parameter Modelica.Units.SI.Time faultStartTime=1; parameter Modelica.Units.SI.Time faultEndTime=3; parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05; parameter Real amplitudeFault=0.5 \"幅值漂移目标比例\"; parameter Real offsetFault=0 \"偏置漂移目标值\"; parameter Real frequencyFault=0.5 \"频率漂移目标比例\"; parameter Real stuckOutput=0 \"输出卡死值\"</p><p><b>Behavior</b> StepVoltage 独立故障增强模型</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.ResistorReplacement\">MSLFaults.Examples.ResistorReplacement</a></p><p><b>Purpose</b> Fault-enhanced StepVoltage</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> The established physical equations and fault modes use the common activation and severity law.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Electrical.Analog.Sources.StepVoltage</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.ResistorReplacement\">MSLFaults.Examples.ResistorReplacement</a></p></html>"));
end FaultableStepVoltage;
