within MSLFaults.Thermal.HeatTransfer.Sensors;
model FaultableTemperatureSensor "Temperature measurement with output faults"

  Modelica.Blocks.Interfaces.RealOutput T(unit="K")
    "Absolute temperature as output signal"
    annotation (Placement(transformation(extent={{100,-10},{120,10}}), iconTransformation(extent={{100,-10},{120,10}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port annotation (Placement(transformation(extent={{
            -110,-10},{-90,10}})));
  type FaultMode=enumeration(Normal "Nominal behaviour", Bias "Additive measurement bias", GainError "Measurement gain error", NoiseIncrease "Sinusoidal measurement disturbance", Stuck "Blend towards a prescribed output", Dropout "Blend towards zero output", Saturation "Blend towards symmetric output limits");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mechanism" ;
  parameter Real severity(min=0,max=1)=1 "Maximum realised fault level" ;
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault window" ;
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal fault window" ;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a full-scale activation or recovery ramp" ;
  Real faultActivation(min=0,max=1) "Severity-scaled fault envelope" ;
  Real startActivation(min=0,max=1) "Fault envelope normalised by severity" ;
  Real endActivation(min=0,max=1) "Indicator that activation or recovery is present" ;
  parameter Real biasFault=1 "Full-effect additive bias in the measured signal unit" ;
  parameter Real gainFault=0.9 "Full-effect gain factor" ;
  parameter Real noiseAmplitude=0.01 "Full-effect sinusoidal amplitude in the measured signal unit" ;
  parameter Modelica.Units.SI.Frequency noiseFrequency=37 "Frequency of the sinusoidal disturbance" ;
  parameter Real stuckValue=0 "Prescribed stuck output in the measured signal unit" ;
  parameter Real saturationLimit=1e9 "Symmetric output limit in the measured signal unit" ;
  Real T_actual "Temperature before the output fault" ;

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

  T_actual=port.T;
  T=if faultMode==FaultMode.Bias then T_actual+faultActivation*biasFault elseif faultMode==FaultMode.GainError then T_actual*(1+faultActivation*(gainFault-1)) elseif faultMode==FaultMode.NoiseIncrease then T_actual+faultActivation*noiseAmplitude*sin(2*Modelica.Constants.pi*noiseFrequency*time) elseif faultMode==FaultMode.Stuck then T_actual+faultActivation*(stuckValue-T_actual) elseif faultMode==FaultMode.Dropout then T_actual*(1-faultActivation) elseif faultMode==FaultMode.Saturation then T_actual+faultActivation*(min(saturationLimit,max(-saturationLimit,T_actual))-T_actual) else T_actual;
  port.Q_flow = 0;


  annotation (
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
            100,100}}),
      graphics={
        Ellipse(extent={{-20,-98},{20,-60}}, lineThickness=0.5, fillColor={191,0,0}, fillPattern=FillPattern.Solid, lineColor={64,64,64}),
        Rectangle(extent={{-12,40},{12,-68}}, lineColor={191,0,0}, fillColor={191,0,0}, fillPattern=FillPattern.Solid, lineThickness=0.5),
        Line(points={{12,0},{100,0}}, color={0,0,127}, thickness=0.5),
        Line(points={{-90,0},{-12,0}}, color={191,0,0}, thickness=0.5),
        Polygon(points={{-12,40},{-12,80},{-10,86},{-6,88},{0,90},{6,88},{10,86},{12,80},{12,40},{-12,40}}, lineThickness=0.5, lineColor={64,64,64}),
        Line(points={{-12,40},{-12,-64}}, thickness=0.5, color={64,64,64}),
        Line(points={{12,40},{12,-64}}, thickness=0.5, color={64,64,64}),
        Line(points={{-40,-20},{-12,-20}}, color={64,64,64}, thickness=0.5),
        Line(points={{-40,20},{-12,20}}, color={64,64,64}, thickness=0.5),
        Line(points={{-40,60},{-12,60}}, color={64,64,64}, thickness=0.5),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Text(extent={{20,60},{80,0}}, textColor={64,64,64}, textString="K"),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }),
    Documentation(info="<html><p>Measure <code>port.T</code> without drawing heat from the node. The reported absolute temperature is expressed in kelvin.</p><p><code>Bias</code> adds <code>biasFault</code>; <code>GainError</code> blends towards <code>gainFault</code>; <code>NoiseIncrease</code> adds a deterministic sinusoid. <code>Stuck</code> blends towards <code>stuckValue</code>, <code>Dropout</code> towards zero and <code>Saturation</code> towards symmetric clipping at <code>saturationLimit</code>. Magnitudes use the measured signal unit. These operations affect the reported signal only.</p><p>The selected fault follows the reversible activation envelope. Read <code>effectLevel</code> during recovery as well as <code>active</code>. Nominal counterpart: <code>Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor</code>.</p></html>"));
end FaultableTemperatureSensor;
