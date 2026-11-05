# August 3 OEM ACC vs November 5 V5.0 longitudinal

The received August 3 rlogs contain genuine camera-origin bus-2 0x271 and 0x273 commands mirrored to bus0, without an openpilot sendcan event. Full analysis is attached in the original ChatGPT conversation as V5.0_stock_ACC_Aug03_vs_Nov05_findings.md.

Do not use camera-request magnitude as measured brake pressure. In 410 OEM BRAKING samples with pump 0.4–0.5, 0x275 word0 strongly tracks requested 0x271 brake magnitude (r=-0.995), while Nov 5 OP-only hydraulic braking did not activate the same 0x275 field. Vehicle deceleration and CAN arbitration must be validated separately.

V5.0 safety blocks bus2 camera 0x271/0x273 forwarding to bus0, which differs from the August setup. The next revision must distinguish OEM request, OP TX, and independent friction/regen feedback without increasing brake authority or removing the physical-brake acceleration veto.
