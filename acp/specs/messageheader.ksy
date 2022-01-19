meta:
  id: messageheader
  title: Identification of message type
  endian: le
seq:
  - id: type
    type: s4
  - id: seq
    type: u4
  - id: timestamp
    type: s8
