meta:
  id: wheelodometryint
  title: Wheel Odometry (integer)
  endian: le
  imports:
    - messageheader
seq:
  - id: header
    type: messageheader
  - id: twist
    type: s4
    repeat: expr
    repeat-expr: 6
