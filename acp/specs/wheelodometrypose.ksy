meta:
  id: wheelodometrypose
  title: Wheel Odometry (pose)
  endian: le
  imports:
    - messageheader
    - rot
seq:
  - id: header
    type: messageheader
  - id: rot
    type: rot
  - id: position
    type: f8
    repeat: expr
    repeat-expr: 3
