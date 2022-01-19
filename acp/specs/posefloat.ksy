meta:
  id: posefloat
  title: integer positioning output
  endian: le
  imports:
    - messageheader
    - rot
seq:
  - id: header
    type: messageheader
  - id: position
    type: f8
    repeat: expr
    repeat-expr: 3
  - id: velocity
    type: f8
    repeat: expr
    repeat-expr: 3
  - id: rmr
    type: rot
  - id: pitch
    type: f8
  - id: roll
    type: f8
  - id: yaw
    type: f8
  - id: quality
    type: u1
  - id: relocalization
    type: u1
