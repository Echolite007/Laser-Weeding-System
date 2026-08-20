function m = loop_margins(loop_tf)
% Stability margins of an open-loop transfer function, packed into a struct
% with the derived dB / Hz quantities that are used throughout the project.

[gainMargin_abs, phaseMargin_deg, phaseCrossFreq_rad_s, gainCrossFreq_rad_s] = ...
    margin(loop_tf);

m = struct();
m.GM_abs        = gainMargin_abs;
m.GM_dB         = 20*log10(gainMargin_abs);
m.PM_deg        = phaseMargin_deg;
m.wcg_rad_s     = phaseCrossFreq_rad_s;
m.wcp_rad_s     = gainCrossFreq_rad_s;
m.fc_hz         = gainCrossFreq_rad_s/(2*pi);

end
