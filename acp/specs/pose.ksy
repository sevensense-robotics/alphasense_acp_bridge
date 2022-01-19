meta:
  id: pose
  title: Phase spase state of the Robot (Position, Orientation, Velocity incl. Direction and Speed)
  endian: le
  imports:
    - messageheader
    - rot
seq:
  - id: header
    type: messageheader
  - id: yaw
    type: f8
  - id: rmr
    type: rot
  - id: velocity
    type: f8
    repeat: expr
    repeat-expr: 3
  - id: position
    type: f8
    repeat: expr
    repeat-expr: 3

