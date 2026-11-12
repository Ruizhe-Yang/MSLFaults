within MSLFaults.Electrical.Analog.Ideal;
model FaultableIdealClosingSwitch "IdealClosingSwitch independent fault model"
  extends Modelica.Electrical.Analog.Interfaces.OnePort;
  parameter Modelica.Units.SI.Resistance Ron(final min=0)=1e-5;
  parameter Modelica.Units.SI.Conductance Goff(final min=0)=1e-5;
  extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(final T=293.15);
  Modelica.Blocks.Interfaces.BooleanInput control annotation(Placement(transformation(origin={0,120},extent={{-20,-20},{20,20}},rotation=270)));
  type FaultMode=enumeration(Normal "正常", StuckOpen "卡在开路", StuckClosed "卡在闭合", ContactResistanceIncrease "接触电阻增加", LeakageIncrease "关断泄漏增加");
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter Modelica.Units.SI.Time faultStartTime=1;
  parameter Modelica.Units.SI.Time faultEndTime=3;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05;
  Real faultActivation(min=0,max=1);
  Real startActivation(min=0,max=1);
  Real endActivation(min=0,max=1);
  parameter Modelica.Units.SI.Resistance RonFault=1;
  parameter Modelica.Units.SI.Conductance GoffFault=1e-2;
  Boolean off;
  Boolean naturalOff;
  Real s(final unit="1");
  constant Modelica.Units.SI.Voltage unitVoltage=1;
  constant Modelica.Units.SI.Current unitCurrent=1;
  Modelica.Units.SI.Resistance Ron_effective;
  Modelica.Units.SI.Conductance Goff_effective;

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
  naturalOff=not control;
  off=if faultMode==FaultMode.StuckOpen and faultActivation>0.5 then true elseif faultMode==FaultMode.StuckClosed and faultActivation>0.5 then false else naturalOff;
  Ron_effective=if faultMode==FaultMode.ContactResistanceIncrease then Ron+faultActivation*(RonFault-Ron) else Ron;
  Goff_effective=if faultMode==FaultMode.LeakageIncrease then Goff+faultActivation*(GoffFault-Goff) else Goff;
  v=(s*unitCurrent)*(if off then 1 else Ron_effective);
  i=(s*unitVoltage)*(if off then Goff_effective else 1);
  LossPower=v*i;


  annotation (defaultComponentName="switch",
    Documentation(info="<html><p><b>Purpose</b> IdealClosingSwitch independent fault model</p><p><b>Interfaces</b> extends Modelica.Electrical.Analog.Interfaces.OnePort; extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(final T=293.15); Modelica.Blocks.Interfaces.BooleanInput control; Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger; output Boolean active \"Requested or latched fault\"; output Real effectLevel(min=0,max=1) \"Current realized fault effect\"; output Integer activationCount \"Number of rising activations\"</p><p><b>Parameters</b> parameter Modelica.Units.SI.Resistance Ron(final min=0)=1e-5; parameter Modelica.Units.SI.Conductance Goff(final min=0)=1e-5; parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter Modelica.Units.SI.Time faultStartTime=1; parameter Modelica.Units.SI.Time faultEndTime=3; parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05; parameter Modelica.Units.SI.Resistance RonFault=1; parameter Modelica.Units.SI.Conductance GoffFault=1e-2; parameter Boolean enabled=true \"Enable the selected fault mode\"; parameter Boolean useExternalTrigger=false \"Use trigger instead of the internal time window\"</p><p><b>Behavior</b> IdealClosingSwitch independent fault model</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.ResistorReplacement\">MSLFaults.Examples.ResistorReplacement</a></p><p><b>Purpose</b> Fault-enhanced IdealClosingSwitch</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> The established physical equations and fault modes use the common activation and severity law.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Electrical.Analog.Ideal.IdealClosingSwitch</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.ResistorReplacement\">MSLFaults.Examples.ResistorReplacement</a></p><p>用法：将 FaultableIdealClosingSwitch 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p>
<p>
The switching behaviour of the ideal closing switch is controlled by the input signal control: off = not control.<br>
For further details, see partial model <a href=\"modelica://Modelica.Electrical.Analog.Interfaces.IdealSwitch\">IdealSwitch</a>.
</p>
</html>",
        revisions="<html>
<ul>
<li><em>February 7, 2016   </em>
       by Anton Haumer<br> extending from partial IdealSwitch<br>
       </li>
<li><em> March 11, 2009   </em>
       by Christoph Clauss<br> conditional heat port added<br>
       </li>
<li><em> 1998   </em>
       by Christoph Clauss<br> initially implemented<br>
       </li>
</ul>
</html>"),
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
            100}}),
      graphics={
        Line(visible=useHeatPort, points={{0,-100},{0,25}}, color={127,0,0}, pattern=LinePattern.Dot, thickness=0.5),
        Line(points={{-90,0},{-40,0}}, color={0,0,255}, thickness=0.65, pattern=LinePattern.Solid),
        Line(points={{40,0},{90,0}}, color={0,0,255}, thickness=0.65, pattern=LinePattern.Solid),
        Ellipse(extent={{-44,-4},{-36,4}}, lineColor={255,0,0}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.7),
        Ellipse(extent={{36,-4},{44,4}}, lineColor={255,0,0}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.7),
        Line(points={{-37,2},{37,34}}, color={255,0,0}, thickness=1, pattern=LinePattern.Solid),
        Line(points={{13,52},{13,29}}, color={64,64,64}, thickness=0.6, pattern=LinePattern.Solid, arrow={Arrow.None,Arrow.Filled}),
        Text(extent={{-150,185},{150,155}}, textString="%name", textColor={0,0,255}, fontSize=0),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableIdealClosingSwitch;
