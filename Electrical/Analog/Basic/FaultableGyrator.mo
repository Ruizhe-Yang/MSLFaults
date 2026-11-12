within MSLFaults.Electrical.Analog.Basic;
model FaultableGyrator "Gyrator"
  import SI = Modelica.Units.SI;
  extends Modelica.Electrical.Analog.Interfaces.TwoPort;
  parameter SI.Conductance G1(start=1) "Primary gyration conductance";
  parameter SI.Conductance G2(start=1) "Secondary gyration conductance";
  type FaultMode = enumeration(Normal "正常", PrimaryGainDrift "原边回转电导漂移", SecondaryGainDrift "副边回转电导漂移", CouplingLoss "耦合损失");
  parameter FaultMode faultMode=FaultMode.Normal "故障模式";
  parameter Real severity(min=0,max=1)=1 "故障严重度";
  parameter SI.Time faultStartTime=1 "故障开始时刻";
  parameter SI.Time faultEndTime=3 "故障结束时刻";
  parameter SI.Time transitionTime(min=0)=0.05 "故障渐变时间";
  Real faultActivation(min=0,max=1) "含严重度的故障激活量";
  Real startActivation(min=0,max=1);
  Real endActivation(min=0,max=1);
  parameter SI.Conductance G1_fault=0.5*G1;
  parameter SI.Conductance G2_fault=0.5*G2;
  SI.Conductance G1_effective;
  SI.Conductance G2_effective;

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
  G1_effective = if faultMode == FaultMode.PrimaryGainDrift or faultMode == FaultMode.CouplingLoss then G1 + faultActivation*(G1_fault - G1) else G1;
  G2_effective = if faultMode == FaultMode.SecondaryGainDrift or faultMode == FaultMode.CouplingLoss then G2 + faultActivation*(G2_fault - G2) else G2;
  i1 = G2_effective*v2;
  i2 = -G1_effective*v1;


  annotation (
    Documentation(info="<html><p><b>Purpose</b> Gyrator</p><p><b>Interfaces</b> extends Modelica.Electrical.Analog.Interfaces.TwoPort; Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger; output Boolean active \"Requested or latched fault\"; output Real effectLevel(min=0,max=1) \"Current realized fault effect\"; output Integer activationCount \"Number of rising activations\"</p><p><b>Parameters</b> parameter SI.Conductance G1(start=1) \"Primary gyration conductance\"; parameter SI.Conductance G2(start=1) \"Secondary gyration conductance\"; parameter FaultMode faultMode=FaultMode.Normal \"故障模式\"; parameter Real severity(min=0,max=1)=1 \"故障严重度\"; parameter SI.Time faultStartTime=1 \"故障开始时刻\"; parameter SI.Time faultEndTime=3 \"故障结束时刻\"; parameter SI.Time transitionTime(min=0)=0.05 \"故障渐变时间\"; parameter SI.Conductance G1_fault=0.5*G1; parameter SI.Conductance G2_fault=0.5*G2; parameter Boolean enabled=true \"Enable the selected fault mode\"; parameter Boolean useExternalTrigger=false \"Use trigger instead of the internal time window\"</p><p><b>Behavior</b> Gyrator</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.ResistorReplacement\">MSLFaults.Examples.ResistorReplacement</a></p><p><b>Purpose</b> Fault-enhanced Gyrator</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> The established physical equations and fault modes use the common activation and severity law.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Electrical.Analog.Basic.Gyrator</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.ResistorReplacement\">MSLFaults.Examples.ResistorReplacement</a></p><p>用法：将 FaultableGyrator 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p>
<p>A gyrator is a two-port element defined by the following equations:</p>
<blockquote><pre>
i1 =  G2 * v2
i2 = -G1 * v1
</pre></blockquote>
<p>where the constants <em>G1</em>, <em>G2</em> are called the gyration conductance.</p>
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
        Line(points={{-40,30},{40,30}}, color={0,0,255}, thickness=0.5),
        Line(points={{-20,-30},{20,-30}}, color={0,0,255}, thickness=0.5),
        Polygon(points={{30,34},{40,30},{30,26},{30,34}}, fillColor={0,0,255}, fillPattern=FillPattern.Solid, lineColor={0,0,255}, lineThickness=0.5),
        Line(points={{-5,10},{-10,-10}}, color={64,64,64}, thickness=0.5),
        Line(points={{9,10},{4,-9}}, color={64,64,64}, thickness=0.5),
        Line(points={{-12,10},{16,10}}, color={64,64,64}, thickness=0.5),
        Text(extent={{-29,69},{30,40}}, textString="G1"),
        Text(extent={{-29,-39},{29,-68}}, textString="G2"),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Polygon(points={{-10,-26},{-20,-30},{-10,-34},{-10,-26}}, fillColor={0,0,255}, fillPattern=FillPattern.Solid, lineColor={0,0,255}, lineThickness=0.5),
        Line(points={{-100,100},{-40,100},{-40,60}}, color={0,0,255}, thickness=0.5),
        Line(points={{20,25},{-40,25},{-40,-15}}, color={0,0,255}, origin={80,75}, rotation=360, thickness=0.5),
        Line(points={{-35,-20},{25,-20},{25,20}}, color={0,0,255}, origin={-65,-80}, rotation=360, thickness=0.5),
        Line(points={{20,-25},{-40,-25},{-40,15}}, color={0,0,255}, origin={80,-75}, rotation=360, thickness=0.5),
        Text(extent={{108,94},{139,62}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableGyrator;
