within MSLFaults.Magnetic.FluxTubes.Basic;
model FaultableEddyCurrent "Eddy-current loss element with a variable effective conductance"
  extends Modelica.Magnetic.FluxTubes.Interfaces.TwoPort;
  extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(T=293.15);
  type FaultMode=enumeration(Normal "Nominal component relation", ConductanceDrift "Changed eddy-current conductance", LossIncrease "Increased eddy-current conductance", LossDecrease "Reduced eddy-current conductance", EddyPathOpen "Finite reduction of eddy-current conductance", EddyPathShort "Finite increase of eddy-current conductance");
  parameter Boolean useConductance=false "Specify conductance directly instead of using material geometry" annotation(Evaluate=true,HideResult=true,choices(checkBox=true));
  parameter Modelica.Units.SI.Conductance G(min=0)=1/0.098e-6 "Directly specified electrical conductance";
  parameter Modelica.Units.SI.Resistivity rho=0.098e-6 "Electrical resistivity of the eddy-current path";
  parameter Modelica.Units.SI.Length l=1 "Effective path length";
  parameter Modelica.Units.SI.Area A=1 "Effective path cross-sectional area";
  final parameter Modelica.Units.SI.Resistance R=rho*l/A "Resistance calculated from material geometry";
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault law";
  parameter Real severity(min=0,max=1)=1 "Scale of the activation envelope";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault request";
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal half-open fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a complete activation or recovery ramp";
  parameter Real conductanceFaultFactor(min=Modelica.Constants.eps)=2 "Conductance multiplier or reciprocal divisor at full effect";
  Real faultActivation(min=0,max=1) "Activation envelope used by the fault law",startActivation(min=0,max=1) "Activation envelope divided by positive severity",endActivation(min=0,max=1) "One while activation or its recovery envelope remains present";
  Modelica.Units.SI.Conductance G_effective "Effective electrical conductance in the eddy-current relation";

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
  G_effective=(if useConductance then G else 1/R)*(if faultMode==FaultMode.ConductanceDrift or faultMode==FaultMode.LossIncrease or faultMode==FaultMode.EddyPathShort then 1+faultActivation*(conductanceFaultFactor-1) elseif faultMode==FaultMode.LossDecrease or faultMode==FaultMode.EddyPathOpen then 1-faultActivation*(1-1/conductanceFaultFactor) else 1);
  LossPower=V_m*der(Phi);
  V_m=G_effective*der(Phi);


  annotation(
    Documentation(info="<html><p>The magnetic relation is V_m=G_effective*der(Phi), with LossPower=V_m*der(Phi). ConductanceDrift, LossIncrease and EddyPathShort scale towards conductanceFaultFactor; LossDecrease and EddyPathOpen scale towards its reciprocal.</p><p>Select direct G or the rho, l and A geometry relation; set conductanceFaultFactor and connect the optional heat port.</p><p>The path modes use finite scaling, not a topology change. A factor above one is required for the named increase/decrease directions.</p><p>Nominal reference: <code>Modelica.Magnetic.FluxTubes.Basic.EddyCurrent</code>. Activation uses the configured half-open fault window or the optional trigger; inspect <code>effectLevel</code> during recovery.</p></html>"),
    defaultComponentName="eddyCurrent",Icon(graphics={
        Rectangle(extent={{-70,40},{70,-40}}, lineColor={255,0,0}, lineThickness=0.75),
        Ellipse(extent={{-28,-27},{28,27}}, lineColor={64,64,64}, fillColor={255,255,255}, fillPattern=FillPattern.None, pattern=LinePattern.Solid, lineThickness=0.75),
        Polygon(points={{27,-5},{34,8},{17,9},{27,-5}}, lineColor={64,64,64}, fillColor={64,64,64}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.5),
        Text(extent={{-46,-55},{46,-81}}, textString="eddy", textColor={64,64,64}, fontSize=0),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}, fontSize=0),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableEddyCurrent;
