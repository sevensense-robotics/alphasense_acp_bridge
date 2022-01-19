meta:
  id: wheelodometrywheelvelocity
  title: Wheel Odometry (wheel velocity)
  endian: le
  imports:
    - messageheader
seq:
  - id: header
    type: messageheader
  - id: leftwheelvelocity
    type: f8
  - id: rightwheelvelocity
    type: f8
