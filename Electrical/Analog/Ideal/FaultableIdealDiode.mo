within MSLFaults.Electrical.Analog.Ideal;
model FaultableIdealDiode "Ideal diode with independent fault physics"
  extends Modelica.Electrical.Analog.Interfaces.OnePort;
  parameter Modelica.Units.SI.Resistance Ron(final min=0)=1e-5 "Forward state-on differential resistance";
  parameter Modelica.Units.SI.Conductance Goff(final min=0)=1e-5 "Backward state-off conductance";
  parameter Modelica.Units.SI.Voltage Vknee(final min=0)=0 "Forward threshold voltage";
  extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort;
  type FaultMode=enumeration(Normal "正常", OpenCircuit "强制开路", ShortCircuit "强制短路", ForwardVoltageDrift "正向压降漂移", LeakageIncrease "反向泄漏增加");
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter Modelica.Units.SI.Time faultStartTime=1;
  parameter Modelica.Units.SI.Time faultEndTime=3;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05;
  Real faultActivation(min=0,max=1);
  Real startActivation(min=0,max=1);
  Real endActivation(min=0,max=1);
  parameter Modelica.Units.SI.Resistance R_short=1e-6;
  parameter Modelica.Units.SI.Conductance G_open=1e-10;
  parameter Modelica.Units.SI.Conductance G_leakFault=1e-2;
  parameter Modelica.Units.SI.Voltage VkneeFault=1;
  Boolean off(start=true);
  Boolean naturalOff;
  Real s(start=0,final unit="1");
  constant Modelica.Units.SI.Voltage unitVoltage=1;
  constant Modelica.Units.SI.Current unitCurrent=1;
  Modelica.Units.SI.Resistance Ron_effective;
  Modelica.Units.SI.Conductance Goff_effective;
  Modelica.Units.SI.Voltage Vknee_effective;

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
  naturalOff = s < 0;
  off = if faultMode==FaultMode.OpenCircuit and faultActivation>0.5 then true
    elseif faultMode==FaultMode.ShortCircuit and faultActivation>0.5 then false else naturalOff;
  Ron_effective = if faultMode==FaultMode.ShortCircuit then Ron+faultActivation*(R_short-Ron) else Ron;
  Goff_effective = if faultMode==FaultMode.OpenCircuit then Goff+faultActivation*(G_open-Goff)
    elseif faultMode==FaultMode.LeakageIncrease then Goff+faultActivation*(G_leakFault-Goff) else Goff;
  Vknee_effective = if faultMode==FaultMode.ForwardVoltageDrift then Vknee+faultActivation*(VkneeFault-Vknee) else Vknee;
  v=(s*unitCurrent)*(if off then 1 else Ron_effective)+Vknee_effective;
  i=(s*unitVoltage)*(if off then Goff_effective else 1)+Goff_effective*Vknee_effective;
  LossPower=v*i;


  annotation(
    Documentation(info="<html><p><b>Purpose</b> Ideal diode with independent fault physics</p><p><b>Interfaces</b> extends Modelica.Electrical.Analog.Interfaces.OnePort; extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort; Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger; output Boolean active \"Requested or latched fault\"; output Real effectLevel(min=0,max=1) \"Current realized fault effect\"; output Integer activationCount \"Number of rising activations\"</p><p><b>Parameters</b> parameter Modelica.Units.SI.Resistance Ron(final min=0)=1e-5 \"Forward state-on differential resistance\"; parameter Modelica.Units.SI.Conductance Goff(final min=0)=1e-5 \"Backward state-off conductance\"; parameter Modelica.Units.SI.Voltage Vknee(final min=0)=0 \"Forward threshold voltage\"; parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter Modelica.Units.SI.Time faultStartTime=1; parameter Modelica.Units.SI.Time faultEndTime=3; parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05; parameter Modelica.Units.SI.Resistance R_short=1e-6; parameter Modelica.Units.SI.Conductance G_open=1e-10; parameter Modelica.Units.SI.Conductance G_leakFault=1e-2; parameter Modelica.Units.SI.Voltage VkneeFault=1</p><p><b>Behavior</b> Ideal diode with independent fault physics</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.ResistorReplacement\">MSLFaults.Examples.ResistorReplacement</a></p><p><b>Purpose</b> Fault-enhanced IdealDiode</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> The established physical equations and fault modes use the common activation and severity law.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Electrical.Analog.Ideal.IdealDiode</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.ResistorReplacement\">MSLFaults.Examples.ResistorReplacement</a></p><p>用法：将 FaultableIdealDiode 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p></html>"),
    Icon(coordinateSystem(preserveAspectRatio=true,extent={{-100,-100},{100,100}}),
      graphics={
        Polygon(points={{30,0},{-30,40},{-30,-40},{30,0}}, lineColor={255,0,0}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, lineThickness=0.75),
        Line(points={{-90,0},{40,0}}, color={0,0,255}, thickness=0.5),
        Line(points={{40,0},{90,0}}, color={0,0,255}, thickness=0.5),
        Line(points={{30,40},{30,-40}}, color={255,0,0}, thickness=0.8),
        Line(visible=useHeatPort, points={{0,-100},{0,-20}}, color={127,0,0}, pattern=LinePattern.Dot, thickness=0.5),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableIdealDiode;
