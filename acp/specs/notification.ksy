meta:
  id: notification
  title: Notification
  endian: le
  imports:
    - messageheader
seq:
  - id: header
    type: messageheader
  - id: moduleid
    type: s4
  - id: statuscode
    type: s4
  - id: severity
    type: s4
  - id: logid
    type: s4
  - id: logentryid
    type: s4
