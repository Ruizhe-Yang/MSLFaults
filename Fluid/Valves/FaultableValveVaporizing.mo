within MSLFaults.Fluid.Valves;
model FaultableValveVaporizing "Vaporising valve with opening-command faults"
  replaceable package Medium=Modelica.Media.Water.WaterIF97_ph constrainedby Modelica.Media.Interfaces.PartialTwoPhaseMedium;
  Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare package Medium=Medium) annotation(Placement(transformation(extent={{-110,-10},{-90,10}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare package Medium=Medium) annotation(Placement(transformation(extent={{90,-10},{110,10}})));
  parameter Modelica.Units.SI.Pressure dp_nominal=1e5 "Nominal pressure drop" ;
  parameter Medium.MassFlowRate m_flow_nominal=1 "Nominal mass flow rate" ;
  parameter Modelica.Fluid.Types.CvTypes CvData=Modelica.Fluid.Types.CvTypes.Av
    "Selection of flow coefficient (OpPoint is not supported by ValveVaporizing)";
  parameter Modelica.Units.SI.Area Av=1e-4 "Av (metric) flow coefficient";
  parameter Real Kv=0 "Kv (metric) flow coefficient [m3/h]";
  parameter Real Cv=0 "Cv (US) flow coefficient [USG/min]";
  parameter Medium.Density rho_nominal=Medium.density_pTX(
    Medium.p_default, Medium.T_default, Medium.X_default)
    "Nominal inlet density";
  parameter Real opening_nominal(min=0,max=1)=1 "Nominal opening; use is specific to the selected valve model" ;
  parameter Boolean filteredOpening=false
    "Enable the MSL second-order opening filter";
  parameter Modelica.Units.SI.Time riseTime=1 "Opening filter rise time";
  parameter Real leakageOpening(min=0,max=1)=1e-3
    "Minimum opening used by the MSL valve regularisation";
  parameter Boolean checkValve=false "Stop reverse flow";
  parameter Real Fl_nominal=0.9 "Nominal liquid-pressure recovery factor" ;
  type FaultMode=enumeration(Normal "Nominal behaviour", StuckClosed "Blend towards the closed-command target", StuckOpen "Blend towards the open-command target", OpeningBias "Additive opening-command bias", CavitationIncrease "Reduction of the applied opening", LeakageIncrease "Increased residual flow or opening");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mechanism" ;
  parameter Real severity(min=0,max=1)=1 "Maximum realised fault level" ;
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault window" ;
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal fault window" ;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a full-scale activation or recovery ramp" ;
  Real faultActivation(min=0,max=1) "Severity-scaled fault envelope" ;
  Real startActivation(min=0,max=1) "Fault envelope normalised by severity" ;
  Real endActivation(min=0,max=1) "Indicator that activation or recovery is present" ;
  parameter Real openingBias=0.1 "Full-effect additive opening bias" ;
  Modelica.Blocks.Interfaces.RealInput opening  "Requested valve opening" annotation(Placement(transformation(origin={0,100},extent={{-20,-20},{20,20}},rotation=270)));
  Modelica.Blocks.Interfaces.RealOutput opening_filtered if filteredOpening
     "Opening after the nominal valve filter" annotation(Placement(transformation(extent={{60,40},{80,60}})));
  Real opening_effective(min=0,max=1) "Fault-adjusted valve opening" ;
  Modelica.Fluid.Valves.ValveVaporizing nominal(
    redeclare package Medium=Medium,
    CvData=CvData,
    Av=Av,
    Kv=Kv,
    Cv=Cv,
    dp_nominal=dp_nominal,
    m_flow_nominal=m_flow_nominal,
    rho_nominal=rho_nominal,
    opening_nominal=opening_nominal,
    filteredOpening=filteredOpening,
    riseTime=riseTime,
    leakageOpening=leakageOpening,
    checkValve=checkValve,
    Fl_nominal=Fl_nominal);

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
  opening_effective=max(0,min(1,if faultMode==FaultMode.StuckClosed then opening*(1-faultActivation) elseif faultMode==FaultMode.StuckOpen then opening+faultActivation*(1-opening) elseif faultMode==FaultMode.OpeningBias then opening+faultActivation*openingBias elseif faultMode==FaultMode.LeakageIncrease then max(opening,0.2*faultActivation) elseif faultMode==FaultMode.CavitationIncrease then opening*(1-0.5*faultActivation) else opening));
  nominal.opening=opening_effective;
  connect(nominal.opening_filtered,opening_filtered);
  connect(port_a,nominal.port_a);connect(port_b,nominal.port_b);


  annotation(Icon(coordinateSystem(preserveAspectRatio=true,extent={{-100,-100},{100,100}}),
      graphics={
        Polygon(points={{-100,50},{100,-50},{100,50},{0,0},{-100,-50},{-100,50}}, lineColor={255,0,0}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, lineThickness=0.75),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Ellipse(extent={{-30,-86},{-18,-74}}, lineColor={0,127,255}, fillColor={255,255,255}, fillPattern=FillPattern.None, pattern=LinePattern.Solid, lineThickness=0.6),
        Ellipse(extent={{-8,-76},{5,-63}}, lineColor={0,127,255}, fillColor={255,255,255}, fillPattern=FillPattern.None, pattern=LinePattern.Solid, lineThickness=0.6),
        Ellipse(extent={{18,-88},{34,-72}}, lineColor={0,127,255}, fillColor={255,255,255}, fillPattern=FillPattern.None, pattern=LinePattern.Solid, lineThickness=0.6),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }),Documentation(info="<html><p>Wrap the MSL <code>ValveVaporizing</code> with a two-phase <code>Medium</code> and the selected flow-coefficient data. Optional opening filtering and reverse-flow blocking are handled by the contained valve.</p><p>Faults move the opening towards zero or one, add <code>openingBias</code>, or impose a leakage floor of <code>0.2*effectLevel</code>. <code>CavitationIncrease</code> multiplies the opening by <code>1-0.5*effectLevel</code>; it does not alter <code>Fl_nominal</code> or add a cavitation state. The applied opening is limited to <code>[0,1]</code>.</p><p>The selected fault follows the reversible activation envelope. Read <code>effectLevel</code> during recovery as well as <code>active</code>. Nominal counterpart: <code>Modelica.Fluid.Valves.ValveVaporizing</code>.</p></html>"));
end FaultableValveVaporizing;
