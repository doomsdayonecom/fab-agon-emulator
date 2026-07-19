//! Minimal, dependency-free HTTP control server for HEADLESS TESTING.
//!
//! Opt-in via `--control-port <n>` (off by default; zero impact otherwise).
//! Lets an automated test harness verify graphics and game state without a
//! human at the screen:
//!
//!   GET /screenshot            -> PPM (P6) of the live VGA framebuffer
//!   GET /mem?addr=<n>&len=<n>  -> raw eZ80 RAM bytes (application/octet-stream)
//!   GET /regs                  -> eZ80 registers (text/plain, debug format)
//!
//! `addr`/`len` accept decimal or `0x`-hex. `len` is capped at 65536.
//!
//! This is a deliberately tiny HTTP/1.1 server on std::net (no extra crates).
//! The intent is that the SAME contract is portable to the Neo6502 / X16
//! emulators, so a single pytest harness works across all the sibling ports.
//!
//! Memory/registers are served over the existing debugger command channel
//! (`DebugCmd::GetMemory` / `GetRegisters`), which the eZ80 thread services in
//! `DebuggerServer::tick` regardless of pause state — so reads are live.

use crate::{DebugCmd, DebugResp};
use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::mpsc::{Receiver, Sender};
use std::sync::{Arc, Mutex};
use std::time::Duration;

/// The latest VGA frame, RGB24 (width*height*3, top-to-bottom). The main
/// render loop refreshes this each frame while the control server is enabled.
#[derive(Default)]
pub struct FrameSnapshot {
    pub width: u32,
    pub height: u32,
    pub rgb: Vec<u8>,
}

const CMD_TIMEOUT: Duration = Duration::from_millis(1000);

/// Blocking accept loop — run in its own thread. Owns the debugger command
/// channel (single consumer), so requests are handled one at a time.
pub fn start(
    port: u16,
    frame: Arc<Mutex<FrameSnapshot>>,
    tx_cmd: Sender<DebugCmd>,
    rx_resp: Receiver<DebugResp>,
) {
    let listener = match TcpListener::bind(("127.0.0.1", port)) {
        Ok(l) => l,
        Err(e) => {
            eprintln!("[control] could not bind 127.0.0.1:{}: {}", port, e);
            return;
        }
    };
    eprintln!(
        "[control] listening on http://127.0.0.1:{}  (/screenshot /mem /regs)",
        port
    );
    for stream in listener.incoming().flatten() {
        handle(stream, &frame, &tx_cmd, &rx_resp);
    }
}

fn handle(
    mut stream: TcpStream,
    frame: &Arc<Mutex<FrameSnapshot>>,
    tx_cmd: &Sender<DebugCmd>,
    rx_resp: &Receiver<DebugResp>,
) {
    let mut buf = [0u8; 2048];
    let n = match stream.read(&mut buf) {
        Ok(n) => n,
        Err(_) => return,
    };
    let req = String::from_utf8_lossy(&buf[..n]);
    let target = req
        .lines()
        .next()
        .and_then(|line| line.split_whitespace().nth(1))
        .unwrap_or("/");
    let (path, query) = target.split_once('?').unwrap_or((target, ""));

    match path {
        "/screenshot" => {
            let f = frame.lock().unwrap();
            if f.width == 0 {
                respond(&mut stream, 503, "text/plain", b"no frame captured yet");
                return;
            }
            let mut body = format!("P6\n{} {}\n255\n", f.width, f.height).into_bytes();
            body.extend_from_slice(&f.rgb);
            respond(&mut stream, 200, "image/x-portable-pixmap", &body);
        }
        "/mem" => match (query_int(query, "addr"), query_int(query, "len")) {
            (Some(addr), Some(len)) if len >= 1 && len <= 0x10000 => {
                match read_mem(tx_cmd, rx_resp, addr, len) {
                    Some(data) => respond(&mut stream, 200, "application/octet-stream", &data),
                    None => respond(&mut stream, 504, "text/plain", b"mem read timed out"),
                }
            }
            _ => respond(
                &mut stream,
                400,
                "text/plain",
                b"usage: /mem?addr=<n>&len=<1..65536>  (n decimal or 0xHEX)",
            ),
        },
        "/regs" => {
            if tx_cmd.send(DebugCmd::GetRegisters).is_err() {
                respond(&mut stream, 500, "text/plain", b"eZ80 gone");
                return;
            }
            match recv_until(rx_resp, |r| matches!(r, DebugResp::Registers(_))) {
                Some(DebugResp::Registers(r)) => {
                    respond(&mut stream, 200, "text/plain", format!("{:?}\n", r).as_bytes())
                }
                _ => respond(&mut stream, 504, "text/plain", b"regs read timed out"),
            }
        }
        _ => respond(&mut stream, 404, "text/plain", b"not found"),
    }
}

/// Send GetMemory and drain responses until the matching Memory arrives.
fn read_mem(tx: &Sender<DebugCmd>, rx: &Receiver<DebugResp>, start: u32, len: u32) -> Option<Vec<u8>> {
    tx.send(DebugCmd::GetMemory { start, len }).ok()?;
    match recv_until(rx, |r| matches!(r, DebugResp::Memory { start: s, .. } if *s == start)) {
        Some(DebugResp::Memory { data, .. }) => Some(data),
        _ => None,
    }
}

/// Receive responses (skipping unrelated ones) until `pred` matches or timeout.
fn recv_until(rx: &Receiver<DebugResp>, pred: impl Fn(&DebugResp) -> bool) -> Option<DebugResp> {
    loop {
        match rx.recv_timeout(CMD_TIMEOUT) {
            Ok(resp) if pred(&resp) => return Some(resp),
            Ok(_) => continue,
            Err(_) => return None,
        }
    }
}

fn query_int(query: &str, key: &str) -> Option<u32> {
    query
        .split('&')
        .filter_map(|pair| pair.split_once('='))
        .find(|(k, _)| *k == key)
        .and_then(|(_, v)| parse_int(v))
}

fn parse_int(s: &str) -> Option<u32> {
    let s = s.trim();
    match s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        Some(hex) => u32::from_str_radix(hex, 16).ok(),
        None => s.parse::<u32>().ok(),
    }
}

fn respond(stream: &mut TcpStream, code: u16, content_type: &str, body: &[u8]) {
    let status = match code {
        200 => "200 OK",
        400 => "400 Bad Request",
        404 => "404 Not Found",
        500 => "500 Internal Server Error",
        503 => "503 Service Unavailable",
        504 => "504 Gateway Timeout",
        _ => "500 Internal Server Error",
    };
    let header = format!(
        "HTTP/1.1 {}\r\nContent-Type: {}\r\nContent-Length: {}\r\nConnection: close\r\n\r\n",
        status,
        content_type,
        body.len()
    );
    let _ = stream.write_all(header.as_bytes());
    let _ = stream.write_all(body);
    let _ = stream.flush();
}
