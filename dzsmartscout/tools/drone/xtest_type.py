#!/usr/bin/env python3
"""通过 XTEST 扩展向指定 X display 注入键盘输入（ctypes 直调 libX11/libXtst，
无需 xdotool —— 无人机上没有装）。用于解锁锁屏等必须真实按键的场景。

用法: python3 xtest_type.py <display> <text> [--enter] [--delay 0.06]
"""
import ctypes
import sys
import time

libX11 = ctypes.CDLL('libX11.so.6')
libXtst = ctypes.CDLL('libXtst.so.6')

libX11.XOpenDisplay.restype = ctypes.c_void_p
libX11.XOpenDisplay.argtypes = [ctypes.c_char_p]
libX11.XStringToKeysym.restype = ctypes.c_ulong
libX11.XStringToKeysym.argtypes = [ctypes.c_char_p]
libX11.XKeysymToKeycode.restype = ctypes.c_ubyte
libX11.XKeysymToKeycode.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
libX11.XFlush.argtypes = [ctypes.c_void_p]
libX11.XCloseDisplay.argtypes = [ctypes.c_void_p]
libXtst.XTestFakeKeyEvent.argtypes = [ctypes.c_void_p, ctypes.c_uint, ctypes.c_int, ctypes.c_ulong]


def main():
    if len(sys.argv) < 3:
        raise SystemExit(__doc__)
    display = sys.argv[1].encode()
    text = sys.argv[2]
    do_enter = '--enter' in sys.argv
    delay = 0.06
    if '--delay' in sys.argv:
        delay = float(sys.argv[sys.argv.index('--delay') + 1])

    dpy = libX11.XOpenDisplay(display)
    if not dpy:
        raise SystemExit('cannot open display %s' % display)

    def tap(keycode):
        libXtst.XTestFakeKeyEvent(dpy, keycode, 1, 0)
        libXtst.XTestFakeKeyEvent(dpy, keycode, 0, 0)
        libX11.XFlush(dpy)
        time.sleep(delay)

    sent = 0
    for ch in text:
        keysym = libX11.XStringToKeysym(ch.encode())
        if keysym == 0:
            print('skip unsupported char: %r' % ch)
            continue
        keycode = libX11.XKeysymToKeycode(dpy, keysym)
        if keycode == 0:
            print('no keycode for: %r' % ch)
            continue
        tap(keycode)
        sent += 1

    if do_enter:
        keycode = libX11.XKeysymToKeycode(dpy, libX11.XStringToKeysym(b'Return'))
        tap(keycode)
        sent += 1

    libX11.XCloseDisplay(dpy)
    print('sent %d key events to %s (enter=%s)' % (sent, display.decode(), do_enter))


if __name__ == '__main__':
    main()
