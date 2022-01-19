meta:
  id: wheelodometrywheelticks
  title: Wheel Odometry (wheel ticks)
  endian: le
  imports:
    - messageheader
seq:
  - id: header
    type: messageheader
  - id: leftwheelticks
    type: s4
  - id: rightwheelticks
    type: s4
