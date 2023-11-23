#!/usr/bin/env python3
import sys
import numpy as np
import pandas as pd
from kalman_filter import kalman_filter

def load_csv(filename):
  data = pd.read_csv(filename)
  for c in data.columns:
    setattr(sys.modules["__main__"], c, data[c].to_numpy())
  globals()["t"] -= globals()["t"][0]

# preparing ekf input data
load_csv("robotPose.line.csv")
# odom_v = .07*t * (t<6) + .07*6 * (t>6)
odom_v = -.0058*t**2 + .098*t
odom_v += (np.random.random(t.shape)-.5) * .15
odom_v[0] = 0

QVAL=0.5
RVAL=0.5
ekf = kalman_filter(
  P=np.eye(6),
  Q=np.eye(6) * QVAL,
  R=np.eye(4) * RVAL,
  x=np.array([odom_x[0], odom_y[0], odom_theta[0], 0, 0, 0]),
  dt=0.1,
)

output = pd.DataFrame(columns=[
  "kf_x", "kf_y", "kf_theta", "kf_omega", "kf_v", "kf_vdot", "odom_v"])

for v, w, ax, ay in zip(odom_v, odom_omega, imu_ax, imu_ay):
  # print(v, w, ax, ay)
  ekf.predict()
  ekf.update([v, w, ax, ay])
  output.loc[len(output)] = list(ekf.get_states()) + [v]

output.to_csv(f"kf_q{QVAL}_r{RVAL}.csv")
