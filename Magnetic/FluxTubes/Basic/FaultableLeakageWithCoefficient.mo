within MSLFaults.Magnetic.FluxTubes.Basic;
model FaultableLeakageWithCoefficient "Magnetic leakage path with a variable useful-flux coefficient"
  extends Modelica.Magnetic.FluxTubes.BaseClasses.Leakage;
  parameter Modelica.Units.SI.CouplingCoefficient c_usefulFlux(final min=Modelica.Constants.eps,final max=1-Modelica.Constants.eps,start=0.7) "Nominal useful-flux coupling coefficient";
  Modelica.Blocks.Interfaces.RealInput R_mUsefulTot(quantity="Reluctance",unit="H-1") "Total reluctance of the useful magnetic path" annotation(Placement(transformation(extent={{-20,-20},{20,20}},rotation=270,origin={0,120})));
  type FaultMode=enumeration(Normal "Nominal component relation", CouplingDrift "Changed useful-flux coefficient", LeakageIncrease "Increased magnetic leakage", LeakageDecrease "Reduced magnetic leakage", LeakagePathOpen "Large finite leakage-path reluctance");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault law";
  parameter Real severity(min=0,max=1)=1 "Scale of the activation envelope";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault request";
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal half-open fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a complete activation or recovery ramp";
  parameter Modelica.Units.SI.CouplingCoefficient cFault(min=Modelica.Constants.eps,max=1-Modelica.Constants.eps)=0.4 "Useful-flux coupling coefficient at full configured effect";
  Real faultActivation(min=0,max=1) "Activation envelope used by the fault law",startActivation(min=0,max=1) "Activation envelope divided by positive severity",endActivation(min=0,max=1) "One while activation or its recovery envelope remains present";
  Modelica.Units.SI.CouplingCoefficient c_effective "Effective useful-flux coupling coefficient";

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
    Documentation(info="<html><p>The relation (1-c_effective)*R_m=c_effective*R_mUsefulTot determines leakage reluctance. CouplingDrift and LeakageIncrease approach cFault. LeakageDecrease and LeakagePathOpen approach 1-Modelica.Constants.eps.</p><p>Connect the leakage path and supply R_mUsefulTot; set c_usefulFlux and cFault for the required coupling change.</p><p>A lower useful-flux coefficient produces more leakage for a fixed useful reluctance. LeakagePathOpen approaches a large finite reluctance rather than disconnecting a port.</p><p>Nominal reference: <code>Modelica.Magnetic.FluxTubes.Basic.LeakageWithCoefficient</code>. Activation uses the configured half-open fault window or the optional trigger; inspect <code>effectLevel</code> during recovery.</p></html>"),
    defaultComponentName="leakage",Icon(graphics={
        Line(points={{-60,60},{60,-60}}, color={255,0,0}, thickness=1),
        Line(points={{-50,11},{-32,30},{29,30},{49,11}}, color={255,0,0}, thickness=0.8, pattern=LinePattern.Dash),
        Text(extent={{-30,-45},{30,-72}}, textString="c", textColor={64,64,64}, fontSize=0),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableLeakageWithCoefficient;
