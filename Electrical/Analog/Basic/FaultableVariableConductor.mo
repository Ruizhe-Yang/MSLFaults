within MSLFaults.Electrical.Analog.Basic;
model FaultableVariableConductor
  "Ideal linear electrical conductor with variable conductance"
  import SI = Modelica.Units.SI;
  parameter SI.Temperature T_ref=300.15 "Reference temperature";
  parameter SI.LinearTemperatureCoefficient alpha=0
    "Temperature coefficient of conductance (G_actual = G/(1 + alpha*(T_heatPort - T_ref))";
  extends Modelica.Electrical.Analog.Interfaces.OnePort;
  extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(T=T_ref);
  SI.Conductance G_actual
    "Actual conductance = G/(1 + alpha*(T_heatPort - T_ref))";
  Modelica.Blocks.Interfaces.RealInput G(unit="S") annotation (Placement(
        transformation(
        origin={0,120},
        extent={{-20,-20},{20,20}},
        rotation=270)));
  type FaultMode = enumeration(Normal "正常", ConductanceScaleError "输入电导比例误差", ConductanceBias "输入电导偏置", OpenCircuit "低电导开路", ShortCircuit "高电导短路");
  parameter FaultMode faultMode=FaultMode.Normal "故障模式";
  parameter Real severity(min=0,max=1)=1 "故障严重度";
  parameter SI.Time faultStartTime=1 "故障开始时刻";
  parameter SI.Time faultEndTime=3 "故障结束时刻";
  parameter SI.Time transitionTime(min=0)=0.05 "故障渐变时间";
  Real faultActivation(min=0,max=1) "含严重度的故障激活量";
  Real startActivation(min=0,max=1);
  Real endActivation(min=0,max=1);
  parameter Real scaleFault=0.5 "比例故障目标";
  parameter SI.Conductance biasFault=1 "偏置故障目标";
  parameter SI.Conductance G_open=1e-10 "开路有限小电导";
  parameter SI.Conductance G_short=1e6 "短路有限大电导";
  SI.Conductance G_effective "故障后的命令电导";

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
  assert((1 + alpha*(T_heatPort - T_ref)) >= Modelica.Constants.eps,
    "Temperature outside scope of model!");
  G_effective = if faultMode == FaultMode.ConductanceScaleError then G*(1 + faultActivation*(scaleFault - 1))
    elseif faultMode == FaultMode.ConductanceBias then G + faultActivation*biasFault
    elseif faultMode == FaultMode.OpenCircuit then G + faultActivation*(G_open - G)
    elseif faultMode == FaultMode.ShortCircuit then G + faultActivation*(G_short - G) else G;
  G_actual = G_effective/(1 + alpha*(T_heatPort - T_ref));
  i = G_actual*v;
  LossPower = v*i;


  annotation (defaultComponentName="conductor",
    Documentation(info="<html><p><b>Purpose</b> FaultableVariableConductor</p><p><b>Interfaces</b> extends Modelica.Electrical.Analog.Interfaces.OnePort; extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(T=T_ref); Modelica.Blocks.Interfaces.RealInput G(unit=\"S\") annotation (Placement(; Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger; output Boolean active \"Requested or latched fault\"; output Real effectLevel(min=0,max=1) \"Current realized fault effect\"; output Integer activationCount \"Number of rising activations\"</p><p><b>Parameters</b> parameter SI.Temperature T_ref=300.15 \"Reference temperature\"; parameter SI.LinearTemperatureCoefficient alpha=0; parameter FaultMode faultMode=FaultMode.Normal \"故障模式\"; parameter Real severity(min=0,max=1)=1 \"故障严重度\"; parameter SI.Time faultStartTime=1 \"故障开始时刻\"; parameter SI.Time faultEndTime=3 \"故障结束时刻\"; parameter SI.Time transitionTime(min=0)=0.05 \"故障渐变时间\"; parameter Real scaleFault=0.5 \"比例故障目标\"; parameter SI.Conductance biasFault=1 \"偏置故障目标\"; parameter SI.Conductance G_open=1e-10 \"开路有限小电导\"; parameter SI.Conductance G_short=1e6 \"短路有限大电导\"; parameter Boolean enabled=true \"Enable the selected fault mode\"</p><p><b>Behavior</b> FaultableVariableConductor</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.ResistorReplacement\">MSLFaults.Examples.ResistorReplacement</a></p><p><b>Purpose</b> Fault-enhanced VariableConductor</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> The established physical equations and fault modes use the common activation and severity law.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Electrical.Analog.Basic.VariableConductor</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.ResistorReplacement\">MSLFaults.Examples.ResistorReplacement</a></p><p>用法：将 FaultableVariableConductor 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p>
<p>The linear conductor connects the branch voltage <em>v</em> with the branch current <em>i</em> by
<br><em><strong>i = G*v</strong></em>
<br>The Conductance <em>G</em> is given as input signal.
<br><br><strong>Attention!!!</strong>
<br>It is recommended that the G signal should not cross the zero value. Otherwise depending on the surrounding circuit the probability of singularities is high.</p>
</html>",
        revisions="<html>
<ul>
<li><em> August 07, 2009   </em>
       by Anton Haumer<br> temperature dependency of conductance added<br>
       </li>
<li><em> March 11, 2009   </em>
       by Christoph Clauss<br> conditional heat port added<br>
       </li>
<li><em>June 7, 2004   </em>
       by Christoph Clauss<br> implemented<br>
       </li>
</ul>
</html>"),
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
            100}}),
      graphics={
        Line(points={{-90,0},{-70,0}}, color={0,0,255}, thickness=0.5),
        Rectangle(extent={{-70,30},{70,-30}}, lineColor={255,0,0}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, lineThickness=0.75),
        Line(points={{70,0},{90,0}}, color={0,0,255}, thickness=0.5),
        Text(extent={{-150,185},{150,155}}, textString="%name", textColor={0,0,255}),
        Line(points={{-45,-42},{44,45}}, color={255,0,0}, thickness=0.9, pattern=LinePattern.Solid, arrow={Arrow.None,Arrow.Filled}),
        Text(extent={{-32,-55},{32,-84}}, textString="G", textColor={64,64,64}, fontSize=0),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableVariableConductor;
