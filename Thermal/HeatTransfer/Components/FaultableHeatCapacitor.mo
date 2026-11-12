within MSLFaults.Thermal.HeatTransfer.Components;
model FaultableHeatCapacitor "Lumped thermal element storing heat"
  parameter Modelica.Units.SI.HeatCapacity C
    "Heat capacity of element (= cp*m)";
  Modelica.Units.SI.Temperature T(start=293.15, displayUnit="degC")
    "Temperature of element";
  Modelica.Units.SI.TemperatureSlope der_T(start=0)
    "Time derivative of temperature (= der(T))";
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port annotation (Placement(transformation(
        origin={0,-100},
        extent={{-10,-10},{10,10}},
        rotation=90)));
  type FaultMode=enumeration(Normal "正常", CapacityDrift "热容漂移", CapacityLoss "热容下降", ThermalRunaway "热失控", SensorLag "等效热惯性增加");
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter Modelica.Units.SI.Time faultStartTime=1;
  parameter Modelica.Units.SI.Time faultEndTime=3;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05;
  Real faultActivation(min=0,max=1);
  Real startActivation(min=0,max=1);
  Real endActivation(min=0,max=1);
  parameter Modelica.Units.SI.HeatCapacity CFault=0.5*C;
  parameter Modelica.Units.SI.HeatCapacity CRunaway=1e-6;
  Modelica.Units.SI.HeatCapacity C_effective;

public
  parameter Boolean enabled=true "Enable the selected fault mode";
  parameter Boolean useExternalTrigger=false "Use trigger instead of the internal time window" annotation(Evaluate=true);
  Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger
    annotation(Placement(transformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270),
      iconTransformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270)));
  output Boolean active "Requested or latched fault";
  output Real effectLevel(min=0,max=1) "Current realized fault effect";
  output Integer activationCount "Number of rising activations";
  final parameter Modelica.Units.SI.Temperature T_energyReference=0 "Fixed energy datum in kelvin";
  Modelica.Units.SI.Energy storedEnergy;
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
    Documentation(info="<html><p><b>Purpose</b> Lumped thermal element storing heat</p><p><b>Interfaces</b> Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port annotation (Placement(transformation(; Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger; output Boolean active \"Requested or latched fault\"; output Real effectLevel(min=0,max=1) \"Current realized fault effect\"; output Integer activationCount \"Number of rising activations\"</p><p><b>Parameters</b> parameter Modelica.Units.SI.HeatCapacity C; parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter Modelica.Units.SI.Time faultStartTime=1; parameter Modelica.Units.SI.Time faultEndTime=3; parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05; parameter Modelica.Units.SI.HeatCapacity CFault=0.5*C; parameter Modelica.Units.SI.HeatCapacity CRunaway=1e-6; parameter Boolean enabled=true \"Enable the selected fault mode\"; parameter Boolean useExternalTrigger=false \"Use trigger instead of the internal time window\"; final parameter Modelica.Units.SI.Temperature T_energyReference=0 \"Fixed energy datum in kelvin\"</p><p><b>Behavior</b> Lumped thermal element storing heat</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.CapacitorDegradation\">MSLFaults.Examples.CapacitorDegradation</a></p><p><b>Purpose</b> Fault-enhanced HeatCapacitor</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> Variable heat capacity is a behavioral approximation with energy referenced to 0 K. Energy remains balanced with port heat flow. A change of capacity can change temperature and does not simulate material loss or reaction heat.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Thermal.HeatTransfer.Components.HeatCapacitor</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.CapacitorDegradation\">MSLFaults.Examples.CapacitorDegradation</a></p><p>用法：将 FaultableHeatCapacitor 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p>
<p>
This is a generic model for the heat capacity of a material.
No specific geometry is assumed beyond a total volume with
uniform temperature for the entire volume.
Furthermore, it is assumed that the heat capacity
is constant (independent of temperature).
</p>
<p>
The temperature T [Kelvin] of this component is a <strong>state</strong>.
A default of T = 25 degree Celsius (= Modelica.Units.Conversions.from_degC(25))
is used as start value for initialization.
This usually means that at start of integration the temperature of this
component is 25 degrees Celsius. You may, of course, define a different
temperature as start value for initialization. Alternatively, it is possible
to set parameter <strong>steadyStateStart</strong> to <strong>true</strong>. In this case
the additional equation '<strong>der</strong>(T) = 0' is used during
initialization, i.e., the temperature T is computed in such a way that
the component starts in <strong>steady state</strong>. This is useful in cases,
where one would like to start simulation in a suitable operating
point without being forced to integrate for a long time to arrive
at this point.
</p>
<p>
Note, that parameter <strong>steadyStateStart</strong> is not available in
the parameter menu of the simulation window, because its value
is utilized during translation to generate quite different
equations depending on its setting. Therefore, the value of this
parameter can only be changed before translating the model.
</p>
<p>
This component may be used for complicated geometries where
the heat capacity C is determined my measurements. If the component
consists mainly of one type of material, the <strong>mass m</strong> of the
component may be measured or calculated and multiplied with the
<strong>specific heat capacity cp</strong> of the component material to
compute C:
</p>
<blockquote><pre>
C = cp*m.
Typical values for cp at 20 degC in J/(kg.K):
   aluminium   896
   concrete    840
   copper      383
   iron        452
   silver      235
   steel       420 ... 500 (V2A)
   wood       2500
</pre></blockquote>
</html>"));
end FaultableHeatCapacitor;
