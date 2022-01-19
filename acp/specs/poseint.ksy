meta:
  id: poseint
  title: integer positioning output
  endian: le
  imports:
    - messageheader
seq:
  - id: header
    type: messageheader
  - id: positionmm
    type: s4
    repeat: expr
    repeat-expr: 3
  - id: velocitymmps
    type: s4
    repeat: expr
    repeat-expr: 3
  - id: pitchcdeg
    type: s4
  - id: rollcdeg
    type: s4
  - id: yawcdeg
    type: s4
  - id: quality
    type: u1
  - id: relocalization
    type: u1
