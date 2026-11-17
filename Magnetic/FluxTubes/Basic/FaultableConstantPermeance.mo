within MSLFaults.Magnetic.FluxTubes.Basic;
model FaultableConstantPermeance "Magnetic permeance with configurable degradation and latched core cracking"
  extends Modelica.Magnetic.FluxTubes.Interfaces.TwoPort;
  extends Modelica.Magnetic.FluxTubes.Icons.Reluctance;
  type FaultMode=enumeration(Normal "Nominal component relation", PermeanceDrift "Changed magnetic permeance", PermeanceLoss "Reduced magnetic permeance", CoreCrack "Latched magnetic-parameter change", MagneticOpen "Finite magnetic open-path approximation", MagneticShort "Finite magnetic short-path approximation");
  parameter Modelica.Units.SI.Permeance G_m=1 "Magnetic permeance";
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault law";
  parameter Real severity(min=0,max=1)=1 "Scale of the activation envelope";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault request";
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal half-open fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a complete activation or recovery ramp";
  parameter Modelica.Units.SI.Permeance G_mFault=0.5*G_m "Magnetic permeance at full configured parameter-change effect";
  parameter Modelica.Units.SI.Permeance G_mOpen=1e-12 "Finite permeance representing an open magnetic path";
  parameter Modelica.Units.SI.Permeance G_mShort=1e12 "Finite permeance representing a short magnetic path";
  Real faultActivation(min=0,max=1) "Activation envelope used by the fault law";
  Real startActivation(min=0,max=1) "Activation envelope divided by positive severity";
  Real endActivation(min=0,max=1) "One while activation or its recovery envelope remains present";
  Modelica.Units.SI.Permeance G_m_effective "Magnetic permeance used by the current relation";

public
  parameter Boolean enabled=true "Enable activation of the selected fault mode";
  parameter Boolean useExternalTrigger=false "Use trigger in place of the internal fault window" annotation(Evaluate=true);
  Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger "External Boolean activation request"
    annotation(Placement(transformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270),
      iconTransformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270)));
  output Boolean active "Requested or latched activation state";
  output Real effectLevel(min=0,max=1) "Severity-scaled activation envelope";
  output Integer activationCount "Number of rising activations, including an initially active request";

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
    Documentation(info="<html><p>Use Phi=G_m_effective*V_m. PermeanceDrift, PermeanceLoss and CoreCrack share G_mFault; magnetic open and short modes approach finite G_mOpen and G_mShort. CoreCrack latches its first activation.</p><p>Connect port_p and port_n, set G_m and the relevant target, and inspect G_m_effective and Phi.</p><p>Drift uses the activation envelope without a separate material-ageing law. Choose non-negative permeances suited to the surrounding magnetic circuit.</p><p>Nominal reference: <code>Modelica.Magnetic.FluxTubes.Basic.ConstantPermeance</code>. Activation uses the configured half-open fault window or the optional trigger; inspect <code>effectLevel</code> during recovery.</p></html>"),
    defaultComponentName="permeance",Icon(graphics={
        Rectangle(extent={{-70,30},{70,-30}}, lineColor={255,0,0}, fillColor={255,255,255}, fillPattern=FillPattern.None, pattern=LinePattern.Solid, lineThickness=0.8, radius=0),
        Text(extent={{-43,-48},{43,-77}}, textString="Gm", textColor={64,64,64}, fontSize=0),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableConstantPermeance;
