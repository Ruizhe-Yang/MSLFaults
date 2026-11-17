within MSLFaults.Thermal.HeatTransfer.Components;
model FaultableHeatCapacitor "Thermal energy storage with variable heat capacity"
  parameter Modelica.Units.SI.HeatCapacity C
    "Nominal heat capacity";
  Modelica.Units.SI.Temperature T(start=293.15, displayUnit="degC")
    "Temperature of element";
  Modelica.Units.SI.TemperatureSlope der_T(start=0)
    "Time derivative of temperature (= der(T))";
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port annotation (Placement(transformation(
        origin={0,-100},
        extent={{-10,-10},{10,10}},
        rotation=90)));
  type FaultMode=enumeration(Normal "Nominal behaviour", CapacityDrift "Change in heat capacity", CapacityLoss "Reduction of heat capacity", ThermalRunaway "Reduction towards a small positive heat capacity", SensorLag "Increase in physical heat capacity");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mechanism" ;
  parameter Real severity(min=0,max=1)=1 "Maximum realised fault level" ;
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault window" ;
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal fault window" ;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a full-scale activation or recovery ramp" ;
  Real faultActivation(min=0,max=1) "Severity-scaled fault envelope" ;
  Real startActivation(min=0,max=1) "Fault envelope normalised by severity" ;
  Real endActivation(min=0,max=1) "Indicator that activation or recovery is present" ;
  parameter Modelica.Units.SI.HeatCapacity CFault=0.5*C "Full-effect heat-capacity target" ;
  parameter Modelica.Units.SI.HeatCapacity CRunaway=1e-6 "Small positive heat-capacity target" ;
  Modelica.Units.SI.HeatCapacity C_effective "Fault-adjusted heat capacity" ;

public
  parameter Boolean enabled=true "Enable the selected fault mechanism";
  parameter Boolean useExternalTrigger=false "Use trigger to request activation instead of the internal time window" annotation(Evaluate=true);
  Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger
     "External activation request" annotation(Placement(transformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270),
      iconTransformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270)));
  output Boolean active "Requested fault activation";
  output Real effectLevel(min=0,max=1) "Realised severity-scaled fault envelope";
  output Integer activationCount "Number of rising fault activations";
  final parameter Modelica.Units.SI.Temperature T_energyReference=0 "Fixed energy datum in kelvin";
  Modelica.Units.SI.Energy storedEnergy "Stored thermal energy relative to the fixed temperature datum" ;
  Modelica.Units.SI.Power lossPower,parameterPower;
protected
  MSLFaults.Utilities.Internal.Activation faultGate(enabled=enabled and faultMode<>FaultMode.Normal,
    permanent=false,severity=severity,startTime=faultStartTime,endTime=faultEndTime,
    transitionTime=transitionTime,useExternalTrigger=useExternalTrigger);
initial equation
  assert(C>0 and CFault>0 and CRunaway>0,"Heat-capacity endpoints must remain positive.");
equation
  connect(trigger,faultGate.externalTrigger);
  if not useExternalTrigger then faultGate.externalTrigger=false; end if;
  active=faultGate.active;
  effectLevel=faultGate.effectLevel;
  activationCount=faultGate.activationCount;

  startActivation=if severity>0 then effectLevel/severity else 0;
  endActivation=if active or effectLevel>0 then 1 else 0;
  faultActivation=effectLevel;
  T = port.T;
  der_T = der(T);
  C_effective=if faultMode==FaultMode.CapacityDrift or faultMode==FaultMode.CapacityLoss then C+faultActivation*(CFault-C) elseif faultMode==FaultMode.ThermalRunaway then C+faultActivation*(CRunaway-C) elseif faultMode==FaultMode.SensorLag then C*(1+10*faultActivation) else C;
  der(storedEnergy)=port.Q_flow;

  storedEnergy=C_effective*(T-T_energyReference);
  lossPower=0;
  parameterPower=der(storedEnergy)-port.Q_flow;
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
            100,100}}),
      graphics={
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Polygon(points={{0,67},{-20,63},{-40,57},{-52,43},{-58,35},{-68,25},{-72,13},{-76,-1},{-78,-15},{-76,-31},{-76,-43},{-76,-53},{-70,-65},{-64,-73},{-48,-77},{-30,-83},{-18,-83},{-2,-85},{8,-89},{22,-89},{32,-87},{42,-81},{54,-75},{56,-73},{66,-61},{68,-53},{70,-51},{72,-35},{76,-21},{78,-13},{78,3},{74,15},{66,25},{54,33},{44,41},{36,57},{26,65},{0,67}}, lineColor={160,160,164}, fillColor={192,192,192}, fillPattern=FillPattern.Solid, lineThickness=0.5),
        Polygon(points={{-58,35},{-68,25},{-72,13},{-76,-1},{-78,-15},{-76,-31},{-76,-43},{-76,-53},{-70,-65},{-64,-73},{-48,-77},{-30,-83},{-18,-83},{-2,-85},{8,-89},{22,-89},{32,-87},{42,-81},{54,-75},{42,-77},{40,-77},{30,-79},{20,-81},{18,-81},{10,-81},{2,-77},{-12,-73},{-22,-73},{-30,-71},{-40,-65},{-50,-55},{-56,-43},{-58,-35},{-58,-25},{-60,-13},{-60,-5},{-60,7},{-58,17},{-56,19},{-52,27},{-48,35},{-44,45},{-40,57},{-58,35}}, fillColor={160,160,164}, fillPattern=FillPattern.Solid, lineColor={64,64,64}, lineThickness=0.5),
        Text(extent={{-69,7},{71,-24}}, textString="%C"),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }),
    Documentation(info="<html><p>Store heat according to <code>storedEnergy=C_effective*(T-T_energyReference)</code>, with the fixed datum <code>T_energyReference=0</code> K and <code>der(storedEnergy)=port.Q_flow</code>. This preserves the stored-energy balance while capacity changes, so temperature may change even at zero external heat flow.</p><p><code>CapacityDrift</code> and <code>CapacityLoss</code> both blend towards <code>CFault</code>. <code>ThermalRunaway</code> blends towards positive <code>CRunaway</code> without adding a heat source. <code>SensorLag</code> increases the physical capacity up to eleven times <code>C</code>; it does not filter a sensor output. All capacity endpoints must remain positive.</p><p><code>lossPower</code> is zero. <code>parameterPower</code> is the residual <code>der(storedEnergy)-port.Q_flow</code> and is identically zero under the model equations.</p><p>The selected fault follows the reversible activation envelope. Read <code>effectLevel</code> during recovery as well as <code>active</code>. Nominal counterpart: <code>Modelica.Thermal.HeatTransfer.Components.HeatCapacitor</code>.</p></html>"));
end FaultableHeatCapacitor;
