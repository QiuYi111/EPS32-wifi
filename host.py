#!/usr/bin/env python3
# host_nc.py  —— 纯 nc 版 REPL/压测
import argparse, subprocess, os, sys, time, selectors, shutil

def check_nc(nc):
    path = shutil.which(nc)
    if not path:
        print(f"[fatal] cannot find '{nc}' in PATH")
        sys.exit(2)
    return path

def open_nc(nc, host, port, timeout=5):
    # BSD nc 常用：-v -w 超时；不依赖 -N
    return subprocess.Popen(
        [nc, "-v", "-w", str(int(timeout)), host, str(port)],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        bufsize=0
    )

def read_exact(pipe, n, sel, timeout=10.0):
    """从 pipe 读满 n 字节，超时返回已读"""
    out = bytearray()
    end = time.time() + timeout
    while len(out) < n:
        remain = end - time.time()
        if remain <= 0:
            break
        events = sel.select(remain)
        if not events:  # 超时
            break
        for key, _ in events:
            chunk = key.fileobj.read(n - len(out))
            if not chunk:
                return bytes(out)  # 对端关闭
            out += chunk
    return bytes(out)

def run_repl_nc(nc, host, port, timeout=5, encoding="utf-8"):
    nc = check_nc(nc)
    print(f"[repl] nc {host}:{port}")
    p = open_nc(nc, host, port, timeout)
    sel = selectors.DefaultSelector()
    sel.register(p.stdout, selectors.EVENT_READ)

    try:
        print("type lines. Ctrl+C to quit.")
        for line in sys.stdin:
            data = line.encode(encoding, errors="ignore")
            p.stdin.write(data); p.stdin.flush()
            echo = read_exact(p.stdout, len(data), sel, timeout=10.0)
            if echo:
                print(f"[echo] {echo.decode(encoding, errors='ignore')}", end="")
            else:
                print("\n[repl] peer closed")
                break
    except KeyboardInterrupt:
        pass
    finally:
        try:
            p.stdin.close()
        except Exception:
            pass
        p.terminate()
        _ = p.communicate(timeout=0.2)[1] if p.poll() is None else None

def bench_via_nc(nc, host, port, size_mb=4, chunk=1460, timeout=5):
    nc = check_nc(nc)
    total = size_mb * 1024 * 1024
    payload = os.urandom(chunk)
    sent = recv = 0

    print(f"[bench-nc] {host}:{port} size={size_mb}MiB chunk={chunk}")
    p = open_nc(nc, host, port, timeout)
    sel = selectors.DefaultSelector()
    sel.register(p.stdout, selectors.EVENT_READ)

    t0 = time.time(); last = t0
    try:
        while sent < total:
            p.stdin.write(payload); p.stdin.flush()
            sent += len(payload)

            r = read_exact(p.stdout, len(payload), sel, timeout=2.0)
            recv += len(r)

            now = time.time()
            if now - last > 1.0:
                print(f"  {sent/total*100:5.1f}%  recv={recv/1024/1024:6.2f} MiB")
                last = now

            if not r:  # 对端关闭
                print("[bench-nc] peer closed early"); break
    except KeyboardInterrupt:
        print("\n[bench-nc] interrupted")
    finally:
        try:
            p.stdin.close()
        except Exception:
            pass
        p.terminate()
        _ = p.communicate(timeout=0.2)[1] if p.poll() is None else None

    dt = max(time.time() - t0, 1e-9)
    thr = (recv / 1024 / 1024) / dt
    print(f"[bench-nc] time={dt:.3f}s goodput≈{thr:.2f} MiB/s")

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", required=True)
    ap.add_argument("--port", type=int, default=12345)
    ap.add_argument("--bench", action="store_true")
    ap.add_argument("--size-mb", type=int, default=4)
    ap.add_argument("--chunk", type=int, default=1460)
    ap.add_argument("--timeout", type=float, default=5.0)
    ap.add_argument("--nc", default="nc", help="nc/ncat 路径，默认 nc")
    args = ap.parse_args()

    if args.bench:
        bench_via_nc(args.nc, args.host, args.port, args.size_mb, args.chunk, args.timeout)
    else:
        run_repl_nc(args.nc, args.host, args.port, args.timeout)
