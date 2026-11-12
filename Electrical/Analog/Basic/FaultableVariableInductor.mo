within MSLFaults.Electrical.Analog.Basic;
model FaultableVariableInductor
  "Ideal linear electrical inductor with variable inductance"
  import SI = Modelica.Units.SI;
  extends Modelica.Electrical.Analog.Interfaces.OnePort;
  Modelica.Blocks.Interfaces.RealInput L(unit="H") annotation (Placement(
        transformation(
        origin={0,120},
        extent={{-20,-20},{20,20}},
        rotation=270), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={0,120})));
  SI.MagneticFlux Psi;
  parameter SI.Inductance Lmin=Modelica.Constants.eps
    "Lower bound for variable inductance";
  parameter SI.Current IC=0 "Initial Value";
  parameter Boolean UIC=false "Decision if initial value IC shall be used";
  type FaultMode = enumeration(Normal "正常", InductanceScaleError "输入电感比例误差", InductanceBias "输入电感偏置", StuckInput "电感输入卡死", InductanceLoss "电感损失");
  parameter FaultMode faultMode=FaultMode.Normal "故障模式";
  parameter Real severity(min=0,max=1)=1 "故障严重度";
  parameter SI.Time faultStartTime=1 "故障开始时刻";
  parameter SI.Time faultEndTime=3 "故障结束时刻";
  parameter SI.Time transitionTime(min=0)=0.05 "故障渐变时间";
  Real faultActivation(min=0,max=1) "含严重度的故障激活量";
  Real startActivation(min=0,max=1);
  Real endActivation(min=0,max=1);
  parameter Real scaleFault=0.5 "比例故障目标";
  parameter SI.Inductance biasFault=1e-3 "偏置故障目标";
  parameter SI.Inductance stuckValue=1e-3 "卡死电感";
  parameter SI.Inductance lossValue=Lmin "电感损失目标";
  SI.Inductance L_effective "故障后的命令电感";

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
initial equation
  if UIC then
    i = IC;
  end if;
equation
  connect(trigger,faultGate.externalTrigger);
  if not useExternalTrigger then faultGate.externalTrigger=false; end if;
  active=faultGate.active;
  effectLevel=faultGate.effectLevel;
  activationCount=faultGate.activationCount;

  startActivation=if severity>0 then effectLevel/severity else 0;
  endActivation=if active or effectLevel>0 then 1 else 0;
  faultActivation=effectLevel;
  assert(L >= 0, "Inductance L_ (= " + String(L) + ") has to be >= 0!");
  // protect solver from index change
  L_effective = if faultMode == FaultMode.InductanceScaleError then L*(1 + faultActivation*(scaleFault - 1))
    elseif faultMode == FaultMode.InductanceBias then L + faultActivation*biasFault
    elseif faultMode == FaultMode.StuckInput then L + faultActivation*(stuckValue - L)
    elseif faultMode == FaultMode.InductanceLoss then L + faultActivation*(lossValue - L) else L;
  Psi = noEvent(max(L_effective, Lmin))*i;
  v = der(Psi);


  annotation (defaultComponentName="inductor",
    Documentation(info="<html><p><b>Purpose</b> FaultableVariableInductor</p><p><b>Interfaces</b> extends Modelica.Electrical.Analog.Interfaces.OnePort; Modelica.Blocks.Interfaces.RealInput L(unit=\"H\") annotation (Placement(; Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger; output Boolean active \"Requested or latched fault\"; output Real effectLevel(min=0,max=1) \"Current realized fault effect\"; output Integer activationCount \"Number of rising activations\"</p><p><b>Parameters</b> parameter SI.Inductance Lmin=Modelica.Constants.eps; parameter SI.Current IC=0 \"Initial Value\"; parameter Boolean UIC=false \"Decision if initial value IC shall be used\"; parameter FaultMode faultMode=FaultMode.Normal \"故障模式\"; parameter Real severity(min=0,max=1)=1 \"故障严重度\"; parameter SI.Time faultStartTime=1 \"故障开始时刻\"; parameter SI.Time faultEndTime=3 \"故障结束时刻\"; parameter SI.Time transitionTime(min=0)=0.05 \"故障渐变时间\"; parameter Real scaleFault=0.5 \"比例故障目标\"; parameter SI.Inductance biasFault=1e-3 \"偏置故障目标\"; parameter SI.Inductance stuckValue=1e-3 \"卡死电感\"; parameter SI.Inductance lossValue=Lmin \"电感损失目标\"</p><p><b>Behavior</b> FaultableVariableInductor</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.InductorDegradation\">MSLFaults.Examples.InductorDegradation</a></p><p><b>Purpose</b> Fault-enhanced VariableInductor</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> The established physical equations and fault modes use the common activation and severity law.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Electrical.Analog.Basic.VariableInductor</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.InductorDegradation\">MSLFaults.Examples.InductorDegradation</a></p><p>用法：将 FaultableVariableInductor 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p>
<p>The linear inductor connects the branch voltage <em>v</em> with the branch current <em>i</em> by
<br><em><strong>v = d Psi/dt </strong></em>with <em><strong>Psi = L * i </strong></em>.
<br>The inductance <em>L</em> is as input signal.
It is required that L &ge; 0, otherwise an assertion is raised. To avoid a variable index system, L = Lmin, if 0 &le; L &lt; Lmin, where Lmin is a parameter with default value Modelica.Constants.eps.</p>
<p>Besides the Lmin parameter the inductor model has got the two parameters IC and UIC that belong together. With the IC parameter the user can specify an initial value of the current that flows through the inductor.</p>
<p><br>Hence the inductor has an initial current at the beginning of the simulation. The other parameter UIC is of type Boolean. If UIC is true, the simulation tool uses</p>
<p><br>the IC value at the initial calculation by adding the equation i= IC. If UIC is false, the IC value can be used (but it does not need to!) to calculate the initial values in order to simplify the numerical algorithms of initial calculation.</p>
</html>",
        revisions="<html>
<ul>
<li><em>June 7, 2004   </em>
       by Christoph Clauss<br>changed, docu added<br>
       </li>
<li><em>April 30, 2004</em>
       by Anton Haumer<br>implemented.
       </li>
</ul>
</html>"),
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
            100}}),
      graphics={
        Line(points={{-90,0},{-60,0}}, color={0,0,255}, thickness=0.5),
        Line(points={{60,0},{90,0}}, color={0,0,255}, thickness=0.5),
        Text(extent={{-150,185},{150,155}}, textString="%name", textColor={0,0,255}),
        Line(points={{-60,0},{-59,6},{-52,14},{-38,14},{-31,6},{-30,0}}, color={0,0,255}, smooth=Smooth.Bezier, thickness=0.5),
        Line(points={{-30,0},{-29,6},{-22,14},{-8,14},{-1,6},{0,0}}, color={255,0,0}, smooth=Smooth.Bezier, thickness=0.8),
        Line(points={{0,0},{1,6},{8,14},{22,14},{29,6},{30,0}}, color={255,0,0}, smooth=Smooth.Bezier, thickness=0.8),
        Line(points={{30,0},{31,6},{38,14},{52,14},{59,6},{60,0}}, color={255,0,0}, smooth=Smooth.Bezier, thickness=0.8),
        Line(points={{-45,-42},{44,45}}, color={255,0,0}, thickness=0.9, pattern=LinePattern.Solid, arrow={Arrow.None,Arrow.Filled}),
        Text(extent={{-32,-55},{32,-84}}, textString="L", textColor={64,64,64}, fontSize=0),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableVariableInductor;
