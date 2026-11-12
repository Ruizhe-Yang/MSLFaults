within MSLFaults.Magnetic.FluxTubes.Basic;
model FaultableConstantPermeance "Fault-enhanced MSL 4.0.0 ConstantPermeance"
  extends Modelica.Magnetic.FluxTubes.Interfaces.TwoPort;
  extends Modelica.Magnetic.FluxTubes.Icons.Reluctance;
  type FaultMode=enumeration(Normal, PermeanceDrift, PermeanceLoss, CoreCrack, MagneticOpen, MagneticShort);
  parameter Modelica.Units.SI.Permeance G_m=1 "Magnetic permeance";
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter Modelica.Units.SI.Time faultStartTime=1;
  parameter Modelica.Units.SI.Time faultEndTime=3;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05;
  parameter Modelica.Units.SI.Permeance G_mFault=0.5*G_m;
  parameter Modelica.Units.SI.Permeance G_mOpen=1e-12;
  parameter Modelica.Units.SI.Permeance G_mShort=1e12;
  Real faultActivation(min=0,max=1);
  Real startActivation(min=0,max=1);
  Real endActivation(min=0,max=1);
  Modelica.Units.SI.Permeance G_m_effective;

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
    permanent=faultMode==FaultMode.CoreCrack,severity=severity,startTime=faultStartTime,endTime=faultEndTime,
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
  G_m_effective=if faultMode==FaultMode.PermeanceDrift or faultMode==FaultMode.PermeanceLoss or faultMode==FaultMode.CoreCrack then G_m+faultActivation*(G_mFault-G_m) elseif faultMode==FaultMode.MagneticOpen then G_m+faultActivation*(G_mOpen-G_m) elseif faultMode==FaultMode.MagneticShort then G_m+faultActivation*(G_mShort-G_m) else G_m;
  G_m_effective*V_m=Phi;


  annotation(
    Documentation(info="<html><p><b>Purpose</b> Fault-enhanced MSL 4.0.0 ConstantPermeance</p><p><b>Interfaces</b> extends Modelica.Magnetic.FluxTubes.Interfaces.TwoPort; Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger; output Boolean active \"Requested or latched fault\"; output Real effectLevel(min=0,max=1) \"Current realized fault effect\"; output Integer activationCount \"Number of rising activations\"</p><p><b>Parameters</b> parameter Modelica.Units.SI.Permeance G_m=1 \"Magnetic permeance\"; parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter Modelica.Units.SI.Time faultStartTime=1; parameter Modelica.Units.SI.Time faultEndTime=3; parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05; parameter Modelica.Units.SI.Permeance G_mFault=0.5*G_m; parameter Modelica.Units.SI.Permeance G_mOpen=1e-12; parameter Modelica.Units.SI.Permeance G_mShort=1e12; parameter Boolean enabled=true \"Enable the selected fault mode\"; parameter Boolean useExternalTrigger=false \"Use trigger instead of the internal time window\"</p><p><b>Behavior</b> Fault-enhanced MSL 4.0.0 ConstantPermeance</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.MagneticReluctance\">MSLFaults.Examples.MagneticReluctance</a></p><p><b>Purpose</b> Fault-enhanced ConstantPermeance</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> The established physical equations and fault modes use the common activation and severity law.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery. CoreCrack latch until a new simulation. faultEndTime does not repair a latched fault.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Magnetic.FluxTubes.Basic.ConstantPermeance</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.MagneticReluctance\">MSLFaults.Examples.MagneticReluctance</a></p><p>用法：将 FaultableConstantPermeance 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p></html>"),
    defaultComponentName="permeance",Icon(graphics={
        Rectangle(extent={{-70,30},{70,-30}}, lineColor={255,0,0}, fillColor={255,255,255}, fillPattern=FillPattern.None, pattern=LinePattern.Solid, lineThickness=0.8, radius=0),
        Text(extent={{-43,-48},{43,-77}}, textString="Gm", textColor={64,64,64}, fontSize=0),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableConstantPermeance;
