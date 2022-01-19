meta:
  id: opstate
  title: Operation state of the state machine
  endian: le
  imports:
    - messageheader
seq:
  - id: header
    type: messageheader
  - id: task
    type: s4
  - id: stage
    type: s4
  - id: taskuuid
    size: 16
  - id: optimizationprogress
    type: s4
    
