within MSLFaults.Electrical.Analog.Ideal;
model FaultableIdealOpAmp "Ideal operational amplifier (norator-nullator pair)"
  Modelica.Units.SI.Voltage v1 "Voltage drop over the left port";
  Modelica.Units.SI.Voltage v2 "Voltage drop over the right port";
  Modelica.Units.SI.Current i1 "Current flowing from pos. to neg. pin of the left port";
  Modelica.Units.SI.Current i2 "Current flowing from pos. to neg. pin of the right port";
  Modelica.Electrical.Analog.Interfaces.PositivePin p1 "Positive pin of the left port" annotation (
      Placement(transformation(extent={{-110,-70},{-90,-50}}), iconTransformation(extent={{-110,-70},{-90,-50}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin n1 "Negative pin of the left port" annotation (
      Placement(transformation(extent={{-110,50},{-90,70}}), iconTransformation(extent={{-110,50},{-90,70}})));
  Modelica.Electrical.Analog.Interfaces.PositivePin p2 "Positive pin of the right port" annotation (
      Placement(transformation(extent={{90,-10},{110,10}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin n2 "Negative pin of the right port" annotation (
      Placement(transformation(
        origin={0,-100},
        extent={{10,-10},{-10,10}},
        rotation=270)));
  type FaultMode=enumeration(Normal "正常", InputOffset "输入失调", InputLeakage "输入泄漏", InputShort "输入短路");
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter Modelica.Units.SI.Time faultStartTime=1;
  parameter Modelica.Units.SI.Time faultEndTime=3;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05;
  Real faultActivation(min=0,max=1);
  Real startActivation(min=0,max=1);
  Real endActivation(min=0,max=1);
  parameter Modelica.Units.SI.Voltage inputOffsetFault=0.01;
  parameter Modelica.Units.SI.Conductance inputLeakageFault=1e-6;

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

  v1 = p1.v - n1.v;
  v2 = p2.v - n2.v;
  0 = p1.i + n1.i;
  0 = p2.i + n2.i;
  i1 = p1.i;
  i2 = p2.i;
  v1 = if faultMode==FaultMode.InputOffset then faultActivation*inputOffsetFault else 0;
  i1 = if faultMode==FaultMode.InputLeakage then faultActivation*inputLeakageFault*v1
    elseif faultMode==FaultMode.InputShort then faultActivation*1e6*v1 else 0;


  annotation (defaultComponentName="opAmp",
    Documentation(info="<html><p><b>Purpose</b> Ideal operational amplifier (norator-nullator pair)</p><p><b>Interfaces</b> Modelica.Electrical.Analog.Interfaces.PositivePin p1 \"Positive pin of the left port\" annotation (; Modelica.Electrical.Analog.Interfaces.NegativePin n1 \"Negative pin of the left port\" annotation (; Modelica.Electrical.Analog.Interfaces.PositivePin p2 \"Positive pin of the right port\" annotation (; Modelica.Electrical.Analog.Interfaces.NegativePin n2 \"Negative pin of the right port\" annotation (; Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger; output Boolean active \"Requested or latched fault\"; output Real effectLevel(min=0,max=1) \"Current realized fault effect\"; output Integer activationCount \"Number of rising activations\"</p><p><b>Parameters</b> parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter Modelica.Units.SI.Time faultStartTime=1; parameter Modelica.Units.SI.Time faultEndTime=3; parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05; parameter Modelica.Units.SI.Voltage inputOffsetFault=0.01; parameter Modelica.Units.SI.Conductance inputLeakageFault=1e-6; parameter Boolean enabled=true \"Enable the selected fault mode\"; parameter Boolean useExternalTrigger=false \"Use trigger instead of the internal time window\"</p><p><b>Behavior</b> Ideal operational amplifier (norator-nullator pair)</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.ResistorReplacement\">MSLFaults.Examples.ResistorReplacement</a></p><p><b>Purpose</b> Fault-enhanced IdealOpAmp</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> The established physical equations and fault modes use the common activation and severity law.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Electrical.Analog.Ideal.IdealOpAmp</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.ResistorReplacement\">MSLFaults.Examples.ResistorReplacement</a></p><p>用法：将 FaultableIdealOpAmp 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p>
<p>
The ideal OpAmp is a two-port. The left port is fixed to <em>v1=0</em> and <em>i1=0</em>
(nullator). At the right port both any voltage <em>v2</em> and any current <em>i2</em>
are possible (norator).
</p>
</html>",
        revisions="<html>
<ul>
<li><em> 1998   </em>
       by Christoph Clauss<br> initially implemented<br>
       </li>
</ul>
</html>"),
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
            100}}),
      graphics={
        Line(points={{60,0},{90,0}}, color={0,0,255}, thickness=0.5),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Polygon(points={{70,0},{-70,80},{-70,-80},{70,0}}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, lineColor={0,0,255}, lineThickness=0.5),
        Line(points={{0,-40},{0,-100}}, color={0,0,255}, thickness=0.5),
        Line(points={{-100,60},{-70,60}}, color={0,0,255}, thickness=0.5),
        Line(points={{-100,-60},{-70,-60}}, color={0,0,255}, thickness=0.5),
        Line(points={{70,0},{100,0}}, color={0,0,255}, thickness=0.5),
        Line(points={{-60,50},{-40,50}}, color={0,0,255}, thickness=0.5),
        Line(points={{-60,-50},{-40,-50}}, color={0,0,255}, thickness=0.5),
        Line(points={{-50,-40},{-50,-60}}, color={0,0,255}, thickness=0.5),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableIdealOpAmp;
