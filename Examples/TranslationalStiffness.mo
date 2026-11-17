within MSLFaults.Examples;
model TranslationalStiffness "Compare stiffness, mass and damping changes"
  extends Modelica.Icons.Example;
  MSLFaults.Utilities.Internal.TranslationalDrive native annotation(Placement(transformation(origin={-245,59},extent={{-25,-25},{25,25}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput nativePosition annotation(Placement(transformation(origin={-188,11},extent={{-7,-7},{7,7}},rotation=0)));
  MSLFaults.Utilities.Internal.TranslationalDrive normal(redeclare MSLFaults.Mechanics.Translational.Components.FaultableMass mass,redeclare MSLFaults.Mechanics.Translational.Components.FaultableSpring spring,redeclare MSLFaults.Mechanics.Translational.Components.FaultableDamper damper) annotation(Placement(transformation(origin={-134,-64},extent={{-25,-25},{25,25}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput normalPosition annotation(Placement(transformation(origin={-77,-112},extent={{-7,-7},{7,7}},rotation=0)));
  MSLFaults.Utilities.Internal.TranslationalDrive stiffnessFault(redeclare MSLFaults.Mechanics.Translational.Components.FaultableMass mass,redeclare MSLFaults.Mechanics.Translational.Components.FaultableSpring spring(faultMode=MSLFaults.Mechanics.Translational.Components.FaultableSpring.FaultMode.StiffnessLoss,cFault=50,faultStartTime=0,faultEndTime=11,transitionTime=0),redeclare MSLFaults.Mechanics.Translational.Components.FaultableDamper damper) annotation(Placement(transformation(origin={-23,59},extent={{-25,-25},{25,25}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput stiffnessFaultPosition annotation(Placement(transformation(origin={34,11},extent={{-7,-7},{7,7}},rotation=0)));
  MSLFaults.Utilities.Internal.TranslationalDrive massFault(redeclare MSLFaults.Mechanics.Translational.Components.FaultableMass mass(faultMode=MSLFaults.Mechanics.Translational.Components.FaultableMass.FaultMode.MassLoss,mFault=0.5,faultStartTime=0,faultEndTime=11,transitionTime=0),redeclare MSLFaults.Mechanics.Translational.Components.FaultableSpring spring,redeclare MSLFaults.Mechanics.Translational.Components.FaultableDamper damper) annotation(Placement(transformation(origin={88,-64},extent={{-25,-25},{25,25}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput massFaultPosition annotation(Placement(transformation(origin={145,-112},extent={{-7,-7},{7,7}},rotation=0)));
  MSLFaults.Utilities.Internal.TranslationalDrive dampingFault(redeclare MSLFaults.Mechanics.Translational.Components.FaultableMass mass,redeclare MSLFaults.Mechanics.Translational.Components.FaultableSpring spring,redeclare MSLFaults.Mechanics.Translational.Components.FaultableDamper damper(faultMode=MSLFaults.Mechanics.Translational.Components.FaultableDamper.FaultMode.DampingIncrease,dIncrease=4,faultStartTime=0,faultEndTime=11,transitionTime=0)) annotation(Placement(transformation(origin={199,59},extent={{-25,-25},{25,25}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput dampingFaultPosition annotation(Placement(transformation(origin={256,11},extent={{-7,-7},{7,7}},rotation=0)));
equation
  connect(native.position,nativePosition) annotation(Line(points={{-220,59},{-188,59},{-188,11}},color={0,0,127}));
  connect(normal.position,normalPosition) annotation(Line(points={{-109,-64},{-77,-64},{-77,-112}},color={0,0,127}));
  connect(stiffnessFault.position,stiffnessFaultPosition) annotation(Line(points={{2,59},{34,59},{34,11}},color={0,0,127}));
  connect(massFault.position,massFaultPosition) annotation(Line(points={{113,-64},{145,-64},{145,-112}},color={0,0,127}));
  connect(dampingFault.position,dampingFaultPosition) annotation(Line(points={{224,59},{256,59},{256,11}},color={0,0,127}));
  annotation(experiment(StartTime=0,StopTime=10,Interval=0.01,Tolerance=1e-6),
    Diagram(coordinateSystem(extent={{-360,-240},{360,240}},preserveAspectRatio=true),graphics={Text(extent={{-280.5,182},{284.5,168}},textString="Same initial position and velocity, same 1 N force",textColor={34,61,73})}),
    Documentation(info="<html><p>Compare five force-driven mass-spring-damper assemblies. The native and normal paths use a 1 N force and 100 N/m stiffness. The stiffness fault uses 50 N/m, while the other paths alter only mass or damping.</p><p>Plot the five position outputs; compare settling position separately from oscillation and decay. Use the stored experiment settings.</p></html>"));
end TranslationalStiffness;
