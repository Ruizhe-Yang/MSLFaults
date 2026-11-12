within MSLFaults.Thermal.HeatTransfer.Components;
model FaultableGeneralHeatFlowToTemperatureAdaptor
  "Signal adaptor for a HeatTransfer port with temperature and derivative of temperature as outputs and heat flow as input (especially useful for FMUs)"
  extends Modelica.Blocks.Interfaces.Adaptors.FlowToPotentialAdaptor(
    final Name_p="T",
    final Name_pder="dT",
    final Name_pder2="d2T",
    final Name_f="Q",
    final Name_fder="der(Q)",
    final Name_fder2="der2(Q)",
    final use_pder2=false,
    final use_fder=false,
    final use_fder2=false,
    p(unit="K", displayUnit="degC"),
    final pder(unit="K/s"),
    final pder2(unit="K/s2"),
    final f(unit="W"),
    final fder(unit="W/s"),
    final fder2(unit="W/s2"));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatPort
    annotation (Placement(transformation(extent={{-30,-10},{-10,10}})));
  type FaultMode=enumeration(Normal "正常", TemperatureBias "温度偏置", TemperatureGainError "温度增益误差", TemperatureStuck "温度卡死", TemperatureDropout "温度丢失");
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter Modelica.Units.SI.Time faultStartTime=1;
  parameter Modelica.Units.SI.Time faultEndTime=3;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05;
  Real faultActivation(min=0,max=1);
  Real startActivation(min=0,max=1);
  Real endActivation(min=0,max=1);
  parameter Modelica.Units.SI.TemperatureDifference biasFault=1;
  parameter Real gainFault=0.9;
  parameter Modelica.Units.SI.Temperature stuckTemperature=293.15;
  Modelica.Units.SI.Temperature temperatureActual;

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
  temperatureActual=heatPort.T;
  y=if faultMode==FaultMode.TemperatureBias then temperatureActual+faultActivation*biasFault elseif faultMode==FaultMode.TemperatureGainError then temperatureActual*(1+faultActivation*(gainFault-1)) elseif faultMode==FaultMode.TemperatureStuck then temperatureActual+faultActivation*(stuckTemperature-temperatureActual) elseif faultMode==FaultMode.TemperatureDropout then temperatureActual*(1-faultActivation) else temperatureActual;
  u = heatPort.Q_flow "input = flow = heat flow";


  annotation (defaultComponentName="heatFlowToTemperatureAdaptor",
    Documentation(info="<html><p><b>Purpose</b> FaultableGeneralHeatFlowToTemperatureAdaptor</p><p><b>Interfaces</b> Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatPort; Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger; output Boolean active \"Requested or latched fault\"; output Real effectLevel(min=0,max=1) \"Current realized fault effect\"; output Integer activationCount \"Number of rising activations\"</p><p><b>Parameters</b> parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter Modelica.Units.SI.Time faultStartTime=1; parameter Modelica.Units.SI.Time faultEndTime=3; parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05; parameter Modelica.Units.SI.TemperatureDifference biasFault=1; parameter Real gainFault=0.9; parameter Modelica.Units.SI.Temperature stuckTemperature=293.15; parameter Boolean enabled=true \"Enable the selected fault mode\"; parameter Boolean useExternalTrigger=false \"Use trigger instead of the internal time window\"</p><p><b>Behavior</b> FaultableGeneralHeatFlowToTemperatureAdaptor</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.ThermalDegradation\">MSLFaults.Examples.ThermalDegradation</a></p><p><b>Purpose</b> Fault-enhanced GeneralHeatFlowToTemperatureAdaptor</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> The established physical equations and fault modes use the common activation and severity law.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Thermal.HeatTransfer.Components.GeneralHeatFlowToTemperatureAdaptor</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.ThermalDegradation\">MSLFaults.Examples.ThermalDegradation</a></p><p>用法：将 FaultableGeneralHeatFlowToTemperatureAdaptor 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p>
<p>
Adaptor between a heatport connector and a signal representation of the flange.
This component is used to provide a pure signal interface around a HeatTransfer model
and export this model in form of an input/output block,
especially as FMU (<a href=\"https://fmi-standard.org\">Functional Mock-up Unit</a>).
Examples of the usage of this adaptor are provided in
<a href=\"modelica://Modelica.Thermal.HeatTransfer.Examples.GenerationOfFMUs\">HeatTransfer.Examples.GenerationOfFMUs</a>.
This adaptor has heatflow as input and temperature and derivative of temperature as output signals.
</p>
</html>"),
    Icon(graphics={
        Rectangle(extent={{-20,100},{20,-100}}, lineColor={191,0,0}, radius=10, lineThickness=0.5),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableGeneralHeatFlowToTemperatureAdaptor;
