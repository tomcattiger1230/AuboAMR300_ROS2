"""Measured interlock for another empty-wrist observation of supported steel."""
import numpy as np


class TrayGraspRecovery:
    """Release a stalled, still-supported partial grasp without moving the arm.

    Only measured feedback and the surveyed tray envelope enter this interlock.
    Reobservation is returned only after three open/unloaded samples; the caller
    must still acquire three fresh full-cap images before resuming manipulation.
    """
    def __init__(self):
        self.pending=False;self.started_ns=None;self.empty_samples=0;self.error=None

    def step(self, *, stage, route, returned, valid, anchored, timestamp_ns,
             last_good_ns, fingers, finger_forces, tray_force, tcp, reobservations):
        fingers=np.asarray(fingers,float);forces=np.asarray(finger_forces,float);tcp=np.asarray(tcp,float)
        supported=(stage=='pickup_tray' and route==2 and returned and not anchored
            and fingers.shape==(2,) and forces.shape==(2,) and tcp.shape==(3,)
            and np.isfinite(fingers).all() and np.isfinite(forces).all() and np.isfinite(tcp).all()
            and np.isfinite(tray_force) and 5<=tray_force<=250
            and np.all((forces>=0)&(forces<=250)) and np.all(np.abs(fingers)<=.04)
            and np.linalg.norm(tcp[:2]-[.2927,0.])<=.03 and .700<=tcp[2]<=.745)
        if not self.pending:
            if not (supported and not valid and 500_000_000<timestamp_ns-last_good_ns<600_000_000_000):return None
            if reobservations>=2:
                self.error='tray visual reacquisition attempts exhausted';return 'failed'
            self.pending=True;self.started_ns=timestamp_ns;self.empty_samples=0
        if not supported or timestamp_ns-self.started_ns>3_000_000_000:
            self.error='partial grasp release lost support or timed out';return 'failed'
        empty=bool(np.all(np.abs(fingers)<.003) and np.all(forces<.5))
        self.empty_samples=self.empty_samples+1 if empty else 0
        if self.empty_samples>=3:
            self.pending=False;return 'reobserve'
        return 'open'


def tray_reobservation_allowed(*, stage, route, returned, valid, anchored,
                              timestamp_ns, last_good_ns, fingers, finger_forces, tray_force):
    fingers=np.asarray(fingers,float);forces=np.asarray(finger_forces,float)
    return (stage=='pickup_tray' and route in (0,1) and returned and not valid and not anchored
            and 500_000_000<timestamp_ns-last_good_ns<600_000_000_000
            and fingers.shape==(2,) and forces.shape==(2,)
            and np.isfinite(fingers).all() and np.isfinite(forces).all() and np.isfinite(tray_force)
            and np.all(np.abs(fingers)<.003) and np.all((forces>=0)&(forces<.5)) and tray_force>=5.)
