within MSLFaults.Thermal.HeatTransfer.Components;
model FaultableGeneralHeatFlowToTemperatureAdaptor
  "Heat-flow adaptor with temperature-output faults"
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
  type FaultMode=enumeration(Normal "Nominal behaviour", TemperatureBias "Additive temperature-output bias", TemperatureGainError "Temperature-output gain error", TemperatureStuck "Blend towards a prescribed temperature", TemperatureDropout "Blend towards zero temperature output");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mechanism" ;
  parameter Real severity(min=0,max=1)=1 "Maximum realised fault level" ;
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault window" ;
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal fault window" ;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a full-scale activation or recovery ramp" ;
  Real faultActivation(min=0,max=1) "Severity-scaled fault envelope" ;
  Real startActivation(min=0,max=1) "Fault envelope normalised by severity" ;
  Real endActivation(min=0,max=1) "Indicator that activation or recovery is present" ;
  parameter Modelica.Units.SI.TemperatureDifference biasFault=1 "Full-effect additive bias in the measured signal unit" ;
  parameter Real gainFault=0.9 "Full-effect gain factor" ;
  parameter Modelica.Units.SI.Temperature stuckTemperature=293.15 "Prescribed stuck temperature output" ;
  Modelica.Units.SI.Temperature temperatureActual "Physical heat-port temperature before the output fault" ;

public
  parameter Boolean enabled=true "Enable the selected fault mechanism";
  parameter Boolean useExternalTrigger=false "Use trigger to request activation instead of the internal time window" annotation(Evaluate=true);
  Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger
     "External activation request" annotation(Placement(transformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270),
      iconTransformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270)));
  output Boolean active "Requested fault activation";
  output Real effectLevel(min=0,max=1) "Realised severity-scaled fault envelope";
  output Integer activationCount "Number of rising fault activations";

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
    Documentation(info="<html><p>Adapt a heat port to a heat-flow input and temperature outputs using the MSL flow-to-potential adaptor. The imposed heat-flow relation remains <code>u=heatPort.Q_flow</code>.</p><p>The reported temperature is blended with an additive bias, gain, prescribed stuck temperature or zero according to <code>faultMode</code>. The physical temperature remains <code>heatPort.T</code>; inherited derivative outputs follow the altered reported temperature. Use <code>temperatureActual</code> to distinguish the physical and reported values.</p><p>The selected fault follows the reversible activation envelope. Read <code>effectLevel</code> during recovery as well as <code>active</code>. Nominal counterpart: <code>Modelica.Thermal.HeatTransfer.Components.GeneralHeatFlowToTemperatureAdaptor</code>.</p></html>"),
    Icon(graphics={
        Rectangle(extent={{-20,100},{20,-100}}, lineColor={191,0,0}, radius=10, lineThickness=0.5),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableGeneralHeatFlowToTemperatureAdaptor;
