within MSLFaults.Magnetic.FluxTubes.Basic;
model FaultableEddyCurrent "Fault-enhanced MSL 4.0.0 EddyCurrent"
  extends Modelica.Magnetic.FluxTubes.Interfaces.TwoPort;
  extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(T=293.15);
  type FaultMode=enumeration(Normal, ConductanceDrift, LossIncrease, LossDecrease, EddyPathOpen, EddyPathShort);
  parameter Boolean useConductance=false annotation(Evaluate=true,HideResult=true,choices(checkBox=true));
  parameter Modelica.Units.SI.Conductance G(min=0)=1/0.098e-6;
  parameter Modelica.Units.SI.Resistivity rho=0.098e-6;
  parameter Modelica.Units.SI.Length l=1;
  parameter Modelica.Units.SI.Area A=1;
  final parameter Modelica.Units.SI.Resistance R=rho*l/A;
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter Modelica.Units.SI.Time faultStartTime=1;
  parameter Modelica.Units.SI.Time faultEndTime=3;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05;
  parameter Real conductanceFaultFactor(min=Modelica.Constants.eps)=2;
  Real faultActivation(min=0,max=1),startActivation(min=0,max=1),endActivation(min=0,max=1);
  Modelica.Units.SI.Conductance G_effective;

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
  G_effective=(if useConductance then G else 1/R)*(if faultMode==FaultMode.ConductanceDrift or faultMode==FaultMode.LossIncrease or faultMode==FaultMode.EddyPathShort then 1+faultActivation*(conductanceFaultFactor-1) elseif faultMode==FaultMode.LossDecrease or faultMode==FaultMode.EddyPathOpen then 1-faultActivation*(1-1/conductanceFaultFactor) else 1);
  LossPower=V_m*der(Phi);
  V_m=G_effective*der(Phi);


  annotation(
    Documentation(info="<html><p><b>Purpose</b> Fault-enhanced MSL 4.0.0 EddyCurrent</p><p><b>Interfaces</b> extends Modelica.Magnetic.FluxTubes.Interfaces.TwoPort; extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(T=293.15); Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger; output Boolean active \"Requested or latched fault\"; output Real effectLevel(min=0,max=1) \"Current realized fault effect\"; output Integer activationCount \"Number of rising activations\"</p><p><b>Parameters</b> parameter Boolean useConductance=false; parameter Modelica.Units.SI.Conductance G(min=0)=1/0.098e-6; parameter Modelica.Units.SI.Resistivity rho=0.098e-6; parameter Modelica.Units.SI.Length l=1; parameter Modelica.Units.SI.Area A=1; final parameter Modelica.Units.SI.Resistance R=rho*l/A; parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter Modelica.Units.SI.Time faultStartTime=1; parameter Modelica.Units.SI.Time faultEndTime=3; parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05; parameter Real conductanceFaultFactor(min=Modelica.Constants.eps)=2</p><p><b>Behavior</b> Fault-enhanced MSL 4.0.0 EddyCurrent</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.MagneticReluctance\">MSLFaults.Examples.MagneticReluctance</a></p><p><b>Purpose</b> Fault-enhanced EddyCurrent</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> The established physical equations and fault modes use the common activation and severity law.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Magnetic.FluxTubes.Basic.EddyCurrent</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.MagneticReluctance\">MSLFaults.Examples.MagneticReluctance</a></p><p>用法：将 FaultableEddyCurrent 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p></html>"),
    defaultComponentName="eddyCurrent",Icon(graphics={
        Rectangle(extent={{-70,40},{70,-40}}, lineColor={255,0,0}, lineThickness=0.75),
        Ellipse(extent={{-28,-27},{28,27}}, lineColor={64,64,64}, fillColor={255,255,255}, fillPattern=FillPattern.None, pattern=LinePattern.Solid, lineThickness=0.75),
        Polygon(points={{27,-5},{34,8},{17,9},{27,-5}}, lineColor={64,64,64}, fillColor={64,64,64}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.5),
        Text(extent={{-46,-55},{46,-81}}, textString="eddy", textColor={64,64,64}, fontSize=0),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}, fontSize=0),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableEddyCurrent;
