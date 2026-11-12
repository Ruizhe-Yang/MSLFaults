within MSLFaults.Electrical.Analog.Basic;
model FaultableTransformer "Transformer with two ports"
  import SI = Modelica.Units.SI;
  extends Modelica.Electrical.Analog.Interfaces.TwoPort(i1(start=0),i2(start=0));
  parameter SI.Inductance L1(start=1) "Primary inductance";
  parameter SI.Inductance L2(start=1) "Secondary inductance";
  parameter SI.Inductance M(start=1) "Coupling inductance";
  Real dv "Difference between voltage drop over primary inductor and voltage drop over secondary inductor";
  type FaultMode = enumeration(Normal "正常", PrimaryInductanceDrift "原边电感漂移", SecondaryInductanceDrift "副边电感漂移", CouplingLoss "耦合损失", PrimaryOpen "原边开路", SecondaryOpen "副边开路");
  parameter FaultMode faultMode=FaultMode.Normal "故障模式";
  parameter Real severity(min=0,max=1)=1 "故障严重度";
  parameter SI.Time faultStartTime=1 "故障开始时刻";
  parameter SI.Time faultEndTime=3 "故障结束时刻";
  parameter SI.Time transitionTime(min=0)=0.05 "故障渐变时间";
  Real faultActivation(min=0,max=1) "含严重度的故障激活量";
  Real startActivation(min=0,max=1);
  Real endActivation(min=0,max=1);
  parameter SI.Inductance L1_fault=0.5*L1 "原边电感故障目标";
  parameter SI.Inductance L2_fault=0.5*L2 "副边电感故障目标";
  parameter SI.Inductance M_fault=0.1*M "耦合电感故障目标";
  parameter SI.Resistance R_open=1e10 "绕组开路有限大电阻";
  SI.Inductance L1_effective;
  SI.Inductance L2_effective;
  SI.Inductance M_effective;
  SI.Resistance R1_series;
  SI.Resistance R2_series;

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
  L1_effective = if faultMode == FaultMode.PrimaryInductanceDrift then L1 + faultActivation*(L1_fault - L1) else L1;
  L2_effective = if faultMode == FaultMode.SecondaryInductanceDrift then L2 + faultActivation*(L2_fault - L2) else L2;
  M_effective = if faultMode == FaultMode.CouplingLoss then M + faultActivation*(M_fault - M) else M;
  R1_series = if faultMode == FaultMode.PrimaryOpen then faultActivation*R_open else 0;
  R2_series = if faultMode == FaultMode.SecondaryOpen then faultActivation*R_open else 0;
  v1 = L1_effective*der(i1) + M_effective*der(i2) + R1_series*i1;
  v2 = M_effective*der(i1) + L2_effective*der(i2) + R2_series*i2;
  dv = v1 - v2;



  annotation (
    Documentation(info="<html><p><b>Purpose</b> Transformer with two ports</p><p><b>Interfaces</b> extends Modelica.Electrical.Analog.Interfaces.TwoPort(i1(start=0),i2(start=0)); Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger; output Boolean active \"Requested or latched fault\"; output Real effectLevel(min=0,max=1) \"Current realized fault effect\"; output Integer activationCount \"Number of rising activations\"</p><p><b>Parameters</b> parameter SI.Inductance L1(start=1) \"Primary inductance\"; parameter SI.Inductance L2(start=1) \"Secondary inductance\"; parameter SI.Inductance M(start=1) \"Coupling inductance\"; parameter FaultMode faultMode=FaultMode.Normal \"故障模式\"; parameter Real severity(min=0,max=1)=1 \"故障严重度\"; parameter SI.Time faultStartTime=1 \"故障开始时刻\"; parameter SI.Time faultEndTime=3 \"故障结束时刻\"; parameter SI.Time transitionTime(min=0)=0.05 \"故障渐变时间\"; parameter SI.Inductance L1_fault=0.5*L1 \"原边电感故障目标\"; parameter SI.Inductance L2_fault=0.5*L2 \"副边电感故障目标\"; parameter SI.Inductance M_fault=0.1*M \"耦合电感故障目标\"; parameter SI.Resistance R_open=1e10 \"绕组开路有限大电阻\"</p><p><b>Behavior</b> Transformer with two ports</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.ResistorReplacement\">MSLFaults.Examples.ResistorReplacement</a></p><p><b>Purpose</b> Fault-enhanced Transformer</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> The established physical equations and fault modes use the common activation and severity law.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Electrical.Analog.Basic.Transformer</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.ResistorReplacement\">MSLFaults.Examples.ResistorReplacement</a></p><p>用法：将 FaultableTransformer 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p>
<p>The transformer is a two port. The left port voltage <em>v1</em>, left port current <em>i1</em>, right port voltage <em>v2</em> and right port current <em>i2</em> are connected by the following relation:</p>
<blockquote><pre>
| v1 |         | L1   M  |  | i1&#39; |
|    |    =    |         |  |     |
| v2 |         | M    L2 |  | i2&#39; |
</pre></blockquote>
<p><em>L1</em>, <em>L2</em>, and <em>M</em> are the primary, secondary, and coupling inductances respectively.</p>
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
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Text(extent={{-20,-60},{20,-100}}, textString="M", textColor={0,0,255}),
        Line(points={{-40,60},{-40,100},{-90,100}}, color={0,0,255}, thickness=0.5),
        Line(points={{40,60},{40,100},{90,100}}, color={0,0,255}, thickness=0.5),
        Line(points={{-40,-60},{-40,-100},{-90,-100}}, color={0,0,255}, thickness=0.5),
        Line(points={{40,-60},{40,-100},{90,-100}}, color={0,0,255}, thickness=0.5),
        Line(points={{-15,-7},{-14,-1},{-7,7},{7,7},{14,-1},{15,-7}}, color={0,0,255}, smooth=Smooth.Bezier, origin={-33,45}, rotation=270, thickness=0.5),
        Line(points={{-15,-7},{-14,-1},{-7,7},{7,7},{14,-1},{15,-7}}, color={0,0,255}, smooth=Smooth.Bezier, origin={-33,15}, rotation=270, thickness=0.5),
        Line(points={{-15,-7},{-14,-1},{-7,7},{7,7},{14,-1},{15,-7}}, color={0,0,255}, smooth=Smooth.Bezier, origin={-33,-15}, rotation=270, thickness=0.5),
        Line(points={{-15,-7},{-14,-1},{-7,7},{7,7},{14,-1},{15,-7}}, color={0,0,255}, smooth=Smooth.Bezier, origin={-33,-45}, rotation=270, thickness=0.5),
        Line(points={{-15,-7},{-14,-1},{-7,7},{7,7},{14,-1},{15,-7}}, color={0,0,255}, smooth=Smooth.Bezier, origin={33,45}, rotation=90, thickness=0.5),
        Line(points={{-15,-7},{-14,-1},{-7,7},{7,7},{14,-1},{15,-7}}, color={0,0,255}, smooth=Smooth.Bezier, origin={33,15}, rotation=90, thickness=0.5),
        Line(points={{-15,-7},{-14,-1},{-7,7},{7,7},{14,-1},{15,-7}}, color={0,0,255}, smooth=Smooth.Bezier, origin={33,-15}, rotation=90, thickness=0.5),
        Line(points={{-15,-7},{-14,-1},{-7,7},{7,7},{14,-1},{15,-7}}, color={0,0,255}, smooth=Smooth.Bezier, origin={33,-45}, rotation=90, thickness=0.5),
        Text(extent={{-100,20},{-58,-20}}, textString="L1", textColor={0,0,255}),
        Text(extent={{60,20},{100,-20}}, textString="L2", textColor={0,0,255}),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableTransformer;
