# vesc_zmq_sender

Command-line ZeroMQ publisher for testing the vesc_zmq_control project. It reads command lines from stdin, encodes numeric fields in base64, and publishes a JSON payload to a topic.

## Build

Install ZeroMQ (macOS):

```
brew install zeromq
```

Configure and build:

```
cmake -S . -B build
cmake --build build
```

## Run

Bind to the same endpoint used by vesc_zmq_control:

```
./build/vesc_zmq_sender --endpoint tcp://127.0.0.1:5555 --robot racecar
```

Input format:

```
speed steering accel [current] [brake]
```

Examples:

```
1.0 0.2 0.0
0.0 0.0 -2.0
0.0 0.0 0.0 4.0
```

Options:
- `--endpoint tcp://ip:port`
- `--robot <name>` (default: racecar)
- `--topic <full_topic>` (override)
- `--encoding ascii|float32_le|float32_be|float64_le|float64_be`
- `--bind` or `--connect` (default: bind)

Notes:
- Default topic is `/<robot_name>/ackermann` to match vesc_zmq_control.
- Numeric fields are base64-encoded strings by default.
