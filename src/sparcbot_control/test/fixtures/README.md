# Test fixtures

`wheel_controller_reference.csv` holds the outputs of a reference wheel velocity
controller (feedforward + deadband + PID) for 5 test sequences, one per gain set,
240 steps each, with a controller reset at step 120. Every row is one control
step for both wheels: the shared `dt_s` and gains, and for each wheel the inputs
(`*_target_m_s`, `*_measured_m_s`) and the expected outputs (`*_error_m_s`,
`*_feedforward_command`, `*_feedback_command`, `*_command`).

The tests replay the inputs through `WheelController.compute()` in
`wheel_controller.py` and check the outputs match to 1e-9.

The feedforward values are fixed test data (listed in `test/reference.py`),
independent of `config/sparcbot.yaml`; measuring your own robot does not affect
these tests.
