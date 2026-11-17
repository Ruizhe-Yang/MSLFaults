within MSLFaults.UsersGuide;
class Limitations "Modelling assumptions and component-specific limits"
  extends Modelica.Icons.Information;
  annotation(Documentation(info="<html><p>Fault targets are deterministic parameters. Finite open, short, blockage and locking approximations retain the component topology. Their magnitudes and the solver settings must suit the surrounding network.</p><p>Normal-mode compatibility depends on the selected class, inherited options, nominal modifiers and initialisation. Sensor saturation, threshold switching and modes sharing a target law require the individual component documentation. Prescribed storage-parameter changes also require inspection of the stated energy or momentum balance.</p><p>Fluid and Thermal.FluidHeatFlow use different connector and medium systems. The examples demonstrate selected configurations and do not establish every possible host-model combination.</p></html>"));
end Limitations;
