from opendbc.car.crc import CRC8H2F
from opendbc.car.mazda.values import Buttons, MazdaFlags

def create_steering_control(packer, CP, frame, apply_torque, lkas):
  if CP.flags & MazdaFlags.CX50H:
    return create_steering_control_cx50h(packer, CP, frame, apply_torque, lkas)

  tmp = apply_torque + 2048

  lo = tmp & 0xFF
  hi = tmp >> 8

  # copy values from camera
  b1 = int(lkas["BIT_1"])
  er1 = int(lkas["ERR_BIT_1"])
  lnv = 0
  ldw = 0
  er2 = int(lkas["ERR_BIT_2"])

  # Some older models do have these, newer models don't.
  # Either way, they all work just fine if set to zero.
  steering_angle = 0
  b2 = 0

  tmp = steering_angle + 2048
  ahi = tmp >> 10
  amd = (tmp & 0x3FF) >> 2
  amd = (amd >> 4) | ((amd & 0xF) << 4)
  alo = (tmp & 0x3) << 2

  ctr = frame % 16
  # bytes:     [    1  ] [ 2 ] [             3               ]  [           4         ]
  csum = 249 - ctr - hi - lo - (lnv << 3) - er1 - (ldw << 7) - (er2 << 4) - (b1 << 5)

  # bytes      [ 5 ] [ 6 ] [    7   ]
  csum = csum - ahi - amd - alo - b2

  if ahi == 1:
    csum = csum + 15

  if csum < 0:
    if csum < -256:
      csum = csum + 512
    else:
      csum = csum + 256

  csum = csum % 256

  values = {}
  if CP.flags & MazdaFlags.GEN1:
    values = {
      "LKAS_REQUEST": apply_torque,
      "CTR": ctr,
      "ERR_BIT_1": er1,
      "LINE_NOT_VISIBLE": lnv,
      "LDW": ldw,
      "BIT_1": b1,
      "ERR_BIT_2": er2,
      "STEERING_ANGLE": steering_angle,
      "ANGLE_ENABLED": b2,
      "CHKSUM": csum
    }

  return packer.make_can_msg("CAM_LKAS", 0, values)

def create_steering_control_cx50h(packer, CP, frame, apply_torque, lkas):
  values = {
    "CHKSUM": 0,
    "CTR": lkas["CTR"],
    "LKAS_EFFECTIVE": 1,
    "LKAS_EFFECTIVE_INV": 0,
    "BIT_1": lkas["BIT_1"],
    "INVARIANT": 489,
    "STEER_TORQUE_MOTOR": lkas["STEER_TORQUE_MOTOR"],
    "STEER_TORQUE_SENSOR": lkas["STEER_TORQUE_SENSOR"],
    "LKAS_REQUEST": apply_torque
  }

  _, dat, _ = packer.make_can_msg("CAM_LKAS", 0, values)

  values["CHKSUM"] = mazda_cx50_hybrid_checksum(dat)

  return packer.make_can_msg("CAM_LKAS", 0, values)

MAZDA_CX50_HYBRID_CHECKSUM_INITIAL = {
  0x00: 0xde,
  0x01: 0xa4,
  0x02: 0xe5,
  0x03: 0x06,
  0x04: 0xb2,
  0x05: 0xab,
  0x06: 0x9b,
  0x07: 0x4b,
  0x08: 0x7e,
  0x09: 0x8d,
  0x0A: 0x3c,
  0x0B: 0x37,
  0x0C: 0x28,
  0x0D: 0xb1,
  0x0E: 0x18,
  0x0F: 0x82
}

def mazda_cx50_hybrid_checksum(d: bytearray) -> int:
  # First byte is checksum, last byte is always zero, initial value is determined by counter
  counter = d[1] & 0x0F
  crc = MAZDA_CX50_HYBRID_CHECKSUM_INITIAL[counter]
  for i in range(1, len(d) - 1):
    crc ^= d[i]
    crc = CRC8H2F[crc]
  return CRC8H2F[crc]

def create_alert_command(packer, cam_msg: dict, ldw: bool, steer_required: bool):
  values = {s: cam_msg[s] for s in [
    "LINE_VISIBLE",
    "LINE_NOT_VISIBLE",
    "LANE_LINES",
    "BIT1",
    "BIT2",
    "BIT3",
    "NO_ERR_BIT",
    "S1",
    "S1_HBEAM",
  ]}
  values.update({
    # TODO: what's the difference between all these? do we need to send all?
    "HANDS_WARN_3_BITS": 0b111 if steer_required else 0,
    "HANDS_ON_STEER_WARN": steer_required,
    "HANDS_ON_STEER_WARN_2": steer_required,

    # TODO: right lane works, left doesn't
    # TODO: need to do something about L/R
    "LDW_WARN_LL": 0,
    "LDW_WARN_RL": 0,
  })
  return packer.make_can_msg("CAM_LANEINFO", 0, values)


def create_button_cmd(packer, CP, counter, button):

  can = int(button == Buttons.CANCEL)
  res = int(button == Buttons.RESUME)

  if CP.flags & MazdaFlags.GEN1:
    values = {
      "CAN_OFF": can,
      "CAN_OFF_INV": (can + 1) % 2,

      "SET_P": 0,
      "SET_P_INV": 1,

      "RES": res,
      "RES_INV": (res + 1) % 2,

      "SET_M": 0,
      "SET_M_INV": 1,

      "DISTANCE_LESS": 0,
      "DISTANCE_LESS_INV": 1,

      "DISTANCE_MORE": 0,
      "DISTANCE_MORE_INV": 1,

      "MODE_X": 0,
      "MODE_X_INV": 1,

      "MODE_Y": 0,
      "MODE_Y_INV": 1,

      "BIT1": 1,
      "BIT2": 1,
      "BIT3": 1,
      "CTR": (counter + 1) % 16,
    }

    return packer.make_can_msg("CRZ_BTNS", 0, values)
