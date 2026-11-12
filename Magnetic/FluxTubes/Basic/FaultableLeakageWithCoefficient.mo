within MSLFaults.Magnetic.FluxTubes.Basic;
model FaultableLeakageWithCoefficient "Fault-enhanced MSL 4.0.0 LeakageWithCoefficient"
  extends Modelica.Magnetic.FluxTubes.BaseClasses.Leakage;
  parameter Modelica.Units.SI.CouplingCoefficient c_usefulFlux(final min=Modelica.Constants.eps,final max=1-Modelica.Constants.eps,start=0.7);
  Modelica.Blocks.Interfaces.RealInput R_mUsefulTot(quantity="Reluctance",unit="H-1") annotation(Placement(transformation(extent={{-20,-20},{20,20}},rotation=270,origin={0,120})));
  type FaultMode=enumeration(Normal, CouplingDrift, LeakageIncrease, LeakageDecrease, LeakagePathOpen);
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter Modelica.Units.SI.Time faultStartTime=1;
  parameter Modelica.Units.SI.Time faultEndTime=3;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05;
  parameter Modelica.Units.SI.CouplingCoefficient cFault(min=Modelica.Constants.eps,max=1-Modelica.Constants.eps)=0.4;
  Real faultActivation(min=0,max=1),startActivation(min=0,max=1),endActivation(min=0,max=1);
  Modelica.Units.SI.CouplingCoefficient c_effective;

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
  c_effective=if faultMode==FaultMode.CouplingDrift or faultMode==FaultMode.LeakageIncrease then c_usefulFlux+faultActivation*(cFault-c_usefulFlux) elseif faultMode==FaultMode.LeakageDecrease or faultMode==FaultMode.LeakagePathOpen then c_usefulFlux+faultActivation*((1-Modelica.Constants.eps)-c_usefulFlux) else c_usefulFlux;
  (1-c_effective)*R_m=c_effective*R_mUsefulTot;


  annotation(
    Documentation(info="<html><p><b>Purpose</b> Fault-enhanced MSL 4.0.0 LeakageWithCoefficient</p><p><b>Interfaces</b> Modelica.Blocks.Interfaces.RealInput R_mUsefulTot(quantity=\"Reluctance\",unit=\"H-1\"); Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger; output Boolean active \"Requested or latched fault\"; output Real effectLevel(min=0,max=1) \"Current realized fault effect\"; output Integer activationCount \"Number of rising activations\"</p><p><b>Parameters</b> parameter Modelica.Units.SI.CouplingCoefficient c_usefulFlux(final min=Modelica.Constants.eps,final max=1-Modelica.Constants.eps,start=0.7); parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter Modelica.Units.SI.Time faultStartTime=1; parameter Modelica.Units.SI.Time faultEndTime=3; parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05; parameter Modelica.Units.SI.CouplingCoefficient cFault(min=Modelica.Constants.eps,max=1-Modelica.Constants.eps)=0.4; parameter Boolean enabled=true \"Enable the selected fault mode\"; parameter Boolean useExternalTrigger=false \"Use trigger instead of the internal time window\"</p><p><b>Behavior</b> Fault-enhanced MSL 4.0.0 LeakageWithCoefficient</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.MagneticReluctance\">MSLFaults.Examples.MagneticReluctance</a></p><p><b>Purpose</b> Fault-enhanced LeakageWithCoefficient</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> The established physical equations and fault modes use the common activation and severity law.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Magnetic.FluxTubes.Basic.LeakageWithCoefficient</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.MagneticReluctance\">MSLFaults.Examples.MagneticReluctance</a></p><p>用法：将 FaultableLeakageWithCoefficient 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p></html>"),
    defaultComponentName="leakage",Icon(graphics={
        Line(points={{-60,60},{60,-60}}, color={255,0,0}, thickness=1),
        Line(points={{-50,11},{-32,30},{29,30},{49,11}}, color={255,0,0}, thickness=0.8, pattern=LinePattern.Dash),
        Text(extent={{-30,-45},{30,-72}}, textString="c", textColor={64,64,64}, fontSize=0),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableLeakageWithCoefficient;
