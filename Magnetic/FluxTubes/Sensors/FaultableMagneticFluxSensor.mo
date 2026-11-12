within MSLFaults.Magnetic.FluxTubes.Sensors;
model FaultableMagneticFluxSensor "Fault-enhanced MSL 4.0.0 MagneticFluxSensor"
  extends Modelica.Magnetic.FluxTubes.Interfaces.TwoPortElementary;
  extends Modelica.Icons.RoundSensor;
  Modelica.Blocks.Interfaces.RealOutput Phi(final quantity="MagneticFlux",final unit="Wb") annotation(Placement(transformation(origin={0,-110},extent={{10,-10},{-10,10}},rotation=90)));
  type FaultMode=enumeration(Normal, Bias, GainError, NoiseIncrease, Stuck, Dropout, Saturation);
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter Modelica.Units.SI.Time faultStartTime=1;
  parameter Modelica.Units.SI.Time faultEndTime=3;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05;
  parameter Modelica.Units.SI.MagneticFlux biasFault=1e-6;
  parameter Real gainFault=0.5;
  parameter Modelica.Units.SI.MagneticFlux stuckValue=0;
  parameter Modelica.Units.SI.MagneticFlux saturationLimit=1;
  Real faultActivation(min=0,max=1),startActivation(min=0,max=1),endActivation(min=0,max=1);
  Modelica.Units.SI.MagneticFlux Phi_raw,Phi_faulted;

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
  port_p.V_m=port_n.V_m;
  Phi_raw=port_p.Phi;
  0=port_p.Phi+port_n.Phi;
  Phi_faulted=if faultMode==FaultMode.Bias then Phi_raw+faultActivation*biasFault elseif faultMode==FaultMode.GainError then Phi_raw*(1+faultActivation*(gainFault-1)) elseif faultMode==FaultMode.NoiseIncrease then Phi_raw+faultActivation*biasFault*sin(997*time) elseif faultMode==FaultMode.Stuck then Phi_raw+faultActivation*(stuckValue-Phi_raw) elseif faultMode==FaultMode.Dropout then (1-faultActivation)*Phi_raw elseif faultMode==FaultMode.Saturation then min(saturationLimit,max(-saturationLimit,Phi_raw)) else Phi_raw;
  Phi=Phi_faulted;


  annotation(
    Documentation(info="<html><p><b>Purpose</b> Fault-enhanced MSL 4.0.0 MagneticFluxSensor</p><p><b>Interfaces</b> Modelica.Blocks.Interfaces.RealOutput Phi(final quantity=\"MagneticFlux\",final unit=\"Wb\"); Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger; output Boolean active \"Requested or latched fault\"; output Real effectLevel(min=0,max=1) \"Current realized fault effect\"; output Integer activationCount \"Number of rising activations\"</p><p><b>Parameters</b> parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter Modelica.Units.SI.Time faultStartTime=1; parameter Modelica.Units.SI.Time faultEndTime=3; parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05; parameter Modelica.Units.SI.MagneticFlux biasFault=1e-6; parameter Real gainFault=0.5; parameter Modelica.Units.SI.MagneticFlux stuckValue=0; parameter Modelica.Units.SI.MagneticFlux saturationLimit=1; parameter Boolean enabled=true \"Enable the selected fault mode\"; parameter Boolean useExternalTrigger=false \"Use trigger instead of the internal time window\"</p><p><b>Behavior</b> Fault-enhanced MSL 4.0.0 MagneticFluxSensor</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.MagneticReluctance\">MSLFaults.Examples.MagneticReluctance</a></p><p><b>Purpose</b> Fault-enhanced MagneticFluxSensor</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> The established physical equations and fault modes use the common activation and severity law.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Magnetic.FluxTubes.Sensors.MagneticFluxSensor</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.MagneticReluctance\">MSLFaults.Examples.MagneticReluctance</a></p><p>用法：将 FaultableMagneticFluxSensor 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p></html>"),
    defaultComponentName="magFluxSensor",Icon(graphics={
        Ellipse(extent={{-70,-70},{70,70}}, lineColor={255,0,0}, fillColor={255,255,255}, fillPattern=FillPattern.None, pattern=LinePattern.Solid, lineThickness=0.8),
        Text(extent={{-53,-17},{53,-53}}, textString="Phi", textColor={64,64,64}, fontSize=0),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableMagneticFluxSensor;
