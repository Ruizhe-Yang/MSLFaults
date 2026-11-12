within MSLFaults.Electrical.Analog.Basic;
model FaultableCCC "Linear current-controlled current source"
  import SI = Modelica.Units.SI;
  extends Modelica.Electrical.Analog.Interfaces.TwoPort;
  parameter Real gain(start=1) "Current gain";

  type FaultMode = enumeration(Normal "正常", GainDrift "增益漂移", OutputLoss "输出损失", StuckGain "增益卡死");
  parameter FaultMode faultMode=FaultMode.Normal "故障模式";
  parameter Real severity(min=0,max=1)=1 "故障严重度";
  parameter SI.Time faultStartTime=1 "故障开始时刻";
  parameter SI.Time faultEndTime=3 "故障结束时刻";
  parameter SI.Time transitionTime(min=0)=0.05 "故障渐变时间";
  Real faultActivation(min=0,max=1) "含严重度的故障激活量";
  Real startActivation(min=0,max=1);
  Real endActivation(min=0,max=1);
  parameter Real gainFault=0.5*gain;
  Real gain_effective;

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
  gain_effective = if faultMode == FaultMode.GainDrift or faultMode == FaultMode.StuckGain then gain + faultActivation*(gainFault - gain)
    elseif faultMode == FaultMode.OutputLoss then gain*(1 - faultActivation) else gain;
  i2 = i1*gain_effective;
  v1 = 0;


  annotation (defaultComponentName="ccc",
    Documentation(info="<html><p><b>Purpose</b> Linear current-controlled current source</p><p><b>Interfaces</b> extends Modelica.Electrical.Analog.Interfaces.TwoPort; Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger; output Boolean active \"Requested or latched fault\"; output Real effectLevel(min=0,max=1) \"Current realized fault effect\"; output Integer activationCount \"Number of rising activations\"</p><p><b>Parameters</b> parameter Real gain(start=1) \"Current gain\"; parameter FaultMode faultMode=FaultMode.Normal \"故障模式\"; parameter Real severity(min=0,max=1)=1 \"故障严重度\"; parameter SI.Time faultStartTime=1 \"故障开始时刻\"; parameter SI.Time faultEndTime=3 \"故障结束时刻\"; parameter SI.Time transitionTime(min=0)=0.05 \"故障渐变时间\"; parameter Real gainFault=0.5*gain; parameter Boolean enabled=true \"Enable the selected fault mode\"; parameter Boolean useExternalTrigger=false \"Use trigger instead of the internal time window\"</p><p><b>Behavior</b> Linear current-controlled current source</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.ResistorReplacement\">MSLFaults.Examples.ResistorReplacement</a></p><p><b>Purpose</b> Fault-enhanced CCC</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> The established physical equations and fault modes use the common activation and severity law.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Electrical.Analog.Basic.CCC</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.ResistorReplacement\">MSLFaults.Examples.ResistorReplacement</a></p><p>用法：将 FaultableCCC 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p>
<p>The linear current-controlled current source is a TwoPort. The right port current i2 is controlled by the left port current i1 via</p>
<blockquote><pre>
i2 = i1 * gain.
</pre></blockquote>
<p>The left port voltage is zero. Any current gain can be chosen.</p>
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
        Rectangle(extent={{-80,80},{80,-80}}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, lineColor={255,0,0}, lineThickness=0.75),
        Line(points={{-90,100},{-40,100},{-40,-100},{-90,-100}}, color={0,0,255}, thickness=0.5),
        Ellipse(extent={{20,20},{60,-20}}, lineColor={0,0,255}, lineThickness=0.5),
        Line(points={{-20,60},{20,60}}, color={0,0,255}, thickness=0.5),
        Polygon(points={{20,60},{10,63},{10,57},{20,60}}, fillColor={0,0,255}, fillPattern=FillPattern.Solid, lineColor={0,0,255}, lineThickness=0.5),
        Line(points={{90,100},{40,100},{40,20}}, color={0,0,255}, thickness=0.5),
        Line(points={{90,-100},{40,-100},{40,-20}}, color={0,0,255}, thickness=0.5),
        Line(points={{20,0},{60,0}}, color={0,0,255}, thickness=0.5),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Text(extent={{108,94},{139,62}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableCCC;
