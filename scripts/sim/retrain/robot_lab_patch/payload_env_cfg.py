# Copyright (c) 2024-2026 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

"""Plan-a fallback for the Go2W payload-envelope retrain (CEO-approved 2026-07-07).

NOT enabled by default. This is the escalation path if plan-d (widened mass/CoM
randomization in ``rough_env_cfg.py``) does NOT bring the deployed policy inside the
6.5 kg front-offset envelope. Activate by training the registered task id
``RobotLab-Isaac-Velocity-Flat-Unitree-Go2W-Payload-v0`` (see ``__init__.py``).

Design choice — event, NOT a URDF swap
--------------------------------------
The task brief offered two equivalent plan-a forms: (i) point the training URDF at
``assets/urdf/go2w_sensored.urdf`` (the armed/sensored deployment body), or (ii) add
a fixed ~6.5 kg front-offset rigid load to the base as an event. We take form (ii).

Reason (deployment-consistency RED LINE): the sensored URDF adds 8 PiPER arm joints
(``piper_joint[1-8]``). Training on it would change the robot's DOF count and joint
ordering, so the resulting actor would emit a different action vector and read a
different obs vector — it would NOT be isomorphic to the deployment shim
``scripts/sim/go2w_policy.py`` (obs57 / act16, legs+wheels only). The shim is frozen
(zero-edit). Form (ii) keeps the 16-DOF locomotion morphology exactly and only changes
the base's inertial properties toward the deployment payload — so the new ckpt stays
drop-in for the shim.

Payload model (from ``assets/urdf/go2w_sensored.urdf``, deployment truth)
------------------------------------------------------------------------
  base (bare trunk)   6.921 kg  CoM at ( 0.021,  0, -0.005)
  PiPER arm (4.66 kg) mounted at (+0.06, 0, +0.067)   -> pulls CoM forward + up
  NUC/mounts (~1.8 kg to reach the audited ~6.5 kg deployment payload; the URDF's
             nuc_weight placeholder is only 0.5 kg, so the URDF UNDER-models the real
             payload — see runbook "payload fidelity" note) mounted rear/top.
  Net deployment payload over bare trunk: ~6.5 kg, CoM shifted +6.5 cm forward /
  +7.5 cm up (audit figures).

We therefore add a base mass-add CENTERED on the real payload (5.0-8.0 kg, so every env
carries a realistic load, not a small perturbation) and bias the base CoM FORWARD/UP to
reproduce the front-offset. Everything else (rewards, commands, actuator gains,
decimation, obs/act) is inherited unchanged from the flat cfg.
"""

from isaaclab.utils import configclass

from .flat_env_cfg import UnitreeGo2WFlatEnvCfg


@configclass
class UnitreeGo2WPayloadFlatEnvCfg(UnitreeGo2WFlatEnvCfg):
    """Flat Go2W with a fixed ~6.5 kg front-offset payload baked into base randomization."""

    def __post_init__(self):
        # post init of parent (flat -> rough): sets all the go2w semantics, incl. the
        # plan-d widened envelope. We then TIGHTEN+BIAS the base terms toward the payload.
        super().__post_init__()

        # Base mass-add centered on the real payload (every env carries ~5-8 kg on base,
        # not a +/- perturbation around bare). operation="add" is inherited.
        self.events.randomize_rigid_body_mass_base.params["mass_distribution_params"] = (5.0, 8.0)

        # Front/up-biased CoM shift on the base (reproduce +6.5 cm fwd / +7.5 cm up).
        # x forward-biased, z up-biased; y stays symmetric. Ranges keep spread so the
        # policy is robust to payload-placement variation, but the mean is offset.
        self.events.randomize_com_positions.params["com_range"] = {
            "x": (0.03, 0.10),   # forward bias (mean ~+6.5 cm)
            "y": (-0.03, 0.03),
            "z": (0.03, 0.10),   # up bias (mean ~+7.5 cm)
        }

        # If the weight of rewards is 0, set rewards to None (mirror the flat cfg guard).
        if self.__class__.__name__ == "UnitreeGo2WPayloadFlatEnvCfg":
            self.disable_zero_weight_rewards()
