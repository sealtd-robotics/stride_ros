README
---

This package uses `protobuf` to serialize data and communicate through zeroMQ.

Build Python Protobuf

```bash
protoc -I=. --python_out=proto_src ext-interface.proto
```