#!/usr/bin/env python3
import argparse
import socket
import sys
from contextlib import contextmanager


def channel(value: str) -> int:
  iv = int(value, 0)
  if not 0 <= iv <= 255:
    raise argparse.ArgumentTypeError("channel must be 0-255")
  return iv


def create_connection(host: str, port: int, timeout: float):
  try:
    return socket.create_connection((host, port), timeout=timeout)
  except OSError as exc:
    raise SystemExit(f"[fatal] connect failed: {exc}") from exc


def read_line(sock: socket.socket, timeout: float) -> str:
  sock.settimeout(timeout)
  data = bytearray()
  while True:
    try:
      chunk = sock.recv(1)
    except socket.timeout as exc:
      raise TimeoutError("timeout waiting for device response") from exc

    if not chunk:
      if data:
        return data.decode("utf-8", errors="replace")
      raise ConnectionError("device closed the connection")

    if chunk == b"\n":
      return data.decode("utf-8", errors="replace")
    if chunk == b"\r":
      continue
    data.extend(chunk)


def send_command(sock: socket.socket, timeout: float, command: str) -> str:
  payload = (command + "\n").encode("ascii", errors="ignore")
  sock.sendall(payload)
  return read_line(sock, timeout)


@contextmanager
def open_client(host: str, port: int, timeout: float):
  sock = create_connection(host, port, timeout)
  try:
    yield sock
  finally:
    try:
      sock.shutdown(socket.SHUT_RDWR)
    except OSError:
      pass
    sock.close()


def do_set(args):
  command = f"SET {args.r} {args.g} {args.b}"
  with open_client(args.host, args.port, args.timeout) as sock:
    reply = send_command(sock, args.timeout, command)
    print(reply)


def do_off(args):
  with open_client(args.host, args.port, args.timeout) as sock:
    reply = send_command(sock, args.timeout, "OFF")
    print(reply)


def do_get(args):
  with open_client(args.host, args.port, args.timeout) as sock:
    reply = send_command(sock, args.timeout, "GET")
    print(reply)


def do_ping(args):
  with open_client(args.host, args.port, args.timeout) as sock:
    reply = send_command(sock, args.timeout, "PING")
    print(reply)


def do_repl(args):
  print(f"[repl] connecting to {args.host}:{args.port}")
  with open_client(args.host, args.port, args.timeout) as sock:
    print("type commands like 'SET 255 0 0', 'GET', 'OFF'. Ctrl+C to exit.")
    try:
      while True:
        try:
          line = input("rgb> ")
        except EOFError:
          print()
          break
        line = line.strip()
        if not line:
          continue
        try:
          reply = send_command(sock, args.timeout, line)
          print(reply)
        except Exception as exc:
          print(f"[error] {exc}")
          break
    except KeyboardInterrupt:
      print()
  print("[repl] bye")


def build_parser():
  ap = argparse.ArgumentParser(description="RGB LED Wi-Fi controller helper")
  ap.add_argument("--host", required=True, help="ESP32 hostname or IP, e.g. esp32s3-rgb.local")
  ap.add_argument("--port", type=int, default=12345)
  ap.add_argument("--timeout", type=float, default=5.0, help="socket timeout seconds")
  sub = ap.add_subparsers(required=True)

  p_set = sub.add_parser("set", help="set RGB color (0-255)")
  p_set.add_argument("r", type=channel)
  p_set.add_argument("g", type=channel)
  p_set.add_argument("b", type=channel)
  p_set.set_defaults(func=do_set)

  p_off = sub.add_parser("off", help="turn LED off")
  p_off.set_defaults(func=do_off)

  p_get = sub.add_parser("get", help="read current color")
  p_get.set_defaults(func=do_get)

  p_ping = sub.add_parser("ping", help="connectivity check")
  p_ping.set_defaults(func=do_ping)

  p_repl = sub.add_parser("repl", help="interactive command prompt")
  p_repl.set_defaults(func=do_repl)

  return ap


def main(argv=None):
  parser = build_parser()
  args = parser.parse_args(argv)
  try:
    args.func(args)
  except TimeoutError as exc:
    raise SystemExit(f"[fatal] {exc}") from exc
  except ConnectionError as exc:
    raise SystemExit(f"[fatal] {exc}") from exc


if __name__ == "__main__":
  main(sys.argv[1:])
