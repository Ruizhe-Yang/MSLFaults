# Changelog

## 0.6.0

- Rename `FaultReplacementLibrary` to `MSLFaults`.
- Retain the six physical and signal domains and add the native limiter replacement.
- Introduce shared half-open activation, an optional trigger and mode-specific permanent latching.
- Revise anchor equations for charge, flux linkage, momentum and thermal storage. Expose losses and parameter work.
- Retire old Tests and duplicate benchmark entries. Move the transistor stage to the electrical domain.

### Migration

- The default window now starts at 1 s and ends at 3 s. The nominal mode is still Normal. Set faultStartTime, faultEndTime and transitionTime explicitly to retain a former timing profile.
- Use effectLevel and active for realized and requested effects. The original startActivation and endActivation are compatibility diagnostics. R_seriesNominal now bounds a selected full-fault series target and does not add a nominal series resistance.
