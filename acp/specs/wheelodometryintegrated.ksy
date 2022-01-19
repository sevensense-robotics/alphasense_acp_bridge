meta:
  id: wheelodometryintegrated
  title: Wheel Odometry (integrated)
  endian: le
  imports:
    - messageheader
seq:
  - id: header
    type: messageheader
  - id: twist
    type: f8
    repeat: expr
    repeat-expr: 6
