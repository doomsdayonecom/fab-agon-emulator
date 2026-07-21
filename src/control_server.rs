//! Headless HTTP control server for automated testing.
//!
//! Conforms to the **Retro Remote Debug Controller** contract, SPEC.md 0.3.0
//! (github doomsdayonecom/retro-remote-debug-controller), so ONE shared pytest
//! client (`client/emu_control.py`) drives the FAB Agon emulator, the X16
//! emulator and the Neo6502 emulator identically. FAB is Rust, so it does not
//! vendor the shared C core (`retro_control.c`) — it conforms to the HTTP
//! surface directly. `conformance/` is the arbiter.
//!
//! Opt-in via `--control-port <n>` (off by default, zero impact otherwise).
//! Binds 127.0.0.1 only. Endpoints (see SPEC.md):
//!
//!   GET  /status                 contract/platform/frame/paused/running (JSON)
//!   GET  /screenshot             PPM (P6) of the live VGA framebuffer
//!   GET  /mem?addr=&len=[&bank=]  raw eZ80 RAM bytes (bank tolerated + ignored)
//!   POST /mem?addr=[&bank=]       poke the request body's bytes (0.3)
//!   GET  /regs                   eZ80 registers, ADL (JSON)
//!   GET  /audio                  VDP audio since last call as a PCM WAV (0.3)
//!   POST /step?frames=N          advance N frames then halt -> {"frame":N}
//!   POST /pause | /resume        halt / free-run -> {"paused":bool}
//!   POST /key?text=c|code=vk[&down=0|1]   inject a key (0.2); no down = tap
//!   POST /reset                  soft reset (0.2)
//!
//! Determinism (SPEC): a monotonic frame counter + a run budget live on the
//! render thread (where the screenshot snapshot is already published); the CPU
//! gate is the machine's existing `paused` atomic. /step sets the budget and
//! resumes the CPU; the render thread decrements per completed frame and halts
//! the CPU when the budget hits 0. Reads (/mem, /regs) marshal to the eZ80
//! thread over the existing debugger command channel, so they're consistent.

use crate::ascii2vk::ascii2vk;
use crate::{DebugCmd, DebugResp};
use ez80::Reg16;
use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Receiver, Sender};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

const CONTRACT: &str = "0.3.0";
/// RRDC audio native format (the Agon VDP mixes 8-bit unsigned mono @16384 Hz).
const AUDIO_RATE: u32 = 16384;
const AUDIO_CHANNELS: u16 = 1;
/// POST /mem body cap (matches the /mem read cap).
const MEM_CAP: usize = 0x10000;

/// A queued key injection (fabgl virtual key, isDown). The control thread
/// pushes; the render/main thread drains and calls sendVKeyEventToFabgl, so
/// fabgl key events are always delivered on the same thread as normal input.
pub type KeyQueue = Arc<Mutex<Vec<(u32, u8)>>>;
/// Rolling capture of the VDP's generated audio (u8 PCM, mono, 16384 Hz), plus a
/// count of samples the ring dropped (oldest-first) on overflow since the last
/// drain. The SDL audio callback appends every block it drains from
/// getAudioSamples; GET /audio drains this and reports the drop count.
#[derive(Default)]
pub struct AudioRing {
    pub samples: Vec<u8>,
    pub dropped: u32,
}
pub type AudioCapture = Arc<Mutex<AudioRing>>;
const PLATFORM: &str = "agon";
const EMULATOR: &str = "fab-agon-emulator";
const CMD_TIMEOUT: Duration = Duration::from_millis(1000);
const STEP_TIMEOUT: Duration = Duration::from_millis(5000);

/// Shared render-thread state: the latest VGA frame (RGB24, top-to-bottom) plus
/// the SPEC frame counter + step budget. The render loop fills `rgb` each frame
/// and advances `frame`/`budget`; the control thread reads them.
pub struct FrameSnapshot {
    pub width: u32,
    pub height: u32,
    pub rgb: Vec<u8>,
    /// Monotonic completed-frame counter (SPEC /status.frame).
    pub frame: u64,
    /// Run budget: -1 = free-run, 0 = halted, N>0 = run N frames then halt.
    pub budget: i64,
}

impl Default for FrameSnapshot {
    fn default() -> Self {
        FrameSnapshot { width: 0, height: 0, rgb: Vec::new(), frame: 0, budget: -1 }
    }
}

/// Blocking accept loop — run in its own thread. Owns the debugger command
/// channel (single consumer), so requests are handled one at a time.
pub fn start(
    port: u16,
    frame: Arc<Mutex<FrameSnapshot>>,
    ez80_paused: Arc<AtomicBool>,
    keys: KeyQueue,
    soft_reset: Arc<AtomicBool>,
    tx_cmd: Sender<DebugCmd>,
    rx_resp: Receiver<DebugResp>,
    audio: AudioCapture,
) {
    let listener = match TcpListener::bind(("127.0.0.1", port)) {
        Ok(l) => l,
        Err(e) => {
            eprintln!("[control] could not bind 127.0.0.1:{}: {}", port, e);
            return;
        }
    };
    eprintln!(
        "[control] listening on http://127.0.0.1:{}  (SPEC {} — /status /screenshot /mem /regs /step /pause /resume /key /reset /audio)",
        port, CONTRACT
    );
    for stream in listener.incoming().flatten() {
        handle(stream, &frame, &ez80_paused, &keys, &soft_reset, &tx_cmd, &rx_resp, &audio);
    }
}

fn handle(
    mut stream: TcpStream,
    frame: &Arc<Mutex<FrameSnapshot>>,
    ez80_paused: &Arc<AtomicBool>,
    keys: &KeyQueue,
    soft_reset: &Arc<AtomicBool>,
    tx_cmd: &Sender<DebugCmd>,
    rx_resp: &Receiver<DebugResp>,
    audio: &AudioCapture,
) {
    let mut buf = [0u8; 2048];
    let n = match stream.read(&mut buf) {
        Ok(n) => n,
        Err(_) => return,
    };
    let req = String::from_utf8_lossy(&buf[..n]);
    let mut parts = req.lines().next().unwrap_or("").split_whitespace();
    let method = parts.next().unwrap_or("");
    let target = parts.next().unwrap_or("/");
    let (path, query) = target.split_once('?').unwrap_or((target, ""));

    match (method, path) {
        ("GET", "/status") => {
            let (f, paused) = {
                let s = frame.lock().unwrap();
                (s.frame, s.budget == 0)
            };
            let body = format!(
                "{{\"contract\":\"{CONTRACT}\",\"emulator\":\"{EMULATOR}\",\"platform\":\"{PLATFORM}\",\"frame\":{f},\"paused\":{paused},\"running\":true}}"
            );
            respond(&mut stream, 200, "application/json; charset=utf-8", body.as_bytes());
        }
        ("GET", "/screenshot") => {
            let f = frame.lock().unwrap();
            if f.width == 0 {
                json_error(&mut stream, 503, "no frame captured yet");
                return;
            }
            let mut body = format!("P6\n{} {}\n255\n", f.width, f.height).into_bytes();
            body.extend_from_slice(&f.rgb);
            respond(&mut stream, 200, "image/x-portable-pixmap", &body);
        }
        ("GET", "/audio") => {
            // RRDC 0.3: drain the VDP audio synthesised since the last call and
            // return it as a self-describing PCM WAV (16-bit signed, mono,
            // 16384 Hz). Empty ring => a valid zero-frame WAV. Headless capture
            // needs SDL_AUDIODRIVER=dummy so the callback runs.
            let (samples, dropped) = {
                let mut cap = audio.lock().unwrap();
                let d = cap.dropped;
                cap.dropped = 0;
                (std::mem::take(&mut cap.samples), d)
            };
            let wav = build_wav(&samples);
            let extra = format!("X-Rrdc-Audio-Dropped: {}\r\n", dropped);
            respond_ex(&mut stream, 200, "audio/wav", &wav, &extra);
        }
        ("GET", "/mem") => match (query_int(query, "addr"), query_int(query, "len")) {
            // bank is tolerated and ignored (Agon is flat 24-bit). len==0 is valid.
            (Some(addr), Some(len)) if len <= 0x10000 => {
                match read_mem(tx_cmd, rx_resp, addr, len) {
                    Some(data) => respond(&mut stream, 200, "application/octet-stream", &data),
                    None => json_error(&mut stream, 504, "mem read timed out"),
                }
            }
            _ => json_error(&mut stream, 400, "usage: /mem?addr=<n>&len=<0..65536>"),
        },
        ("POST", "/mem") => match query_int(query, "addr") {
            // RRDC 0.3: poke the request body's bytes starting at addr.
            Some(addr) => {
                let body = read_body(&mut stream, &buf[..n], &req);
                match write_mem(tx_cmd, rx_resp, addr, &body) {
                    Some(written) => respond(
                        &mut stream, 200, "application/json; charset=utf-8",
                        format!("{{\"written\":{written}}}").as_bytes()),
                    None => json_error(&mut stream, 504, "mem write timed out"),
                }
            }
            None => json_error(&mut stream, 400, "usage: POST /mem?addr=<n> with raw body bytes"),
        },
        ("GET", "/regs") => {
            if tx_cmd.send(DebugCmd::GetRegisters).is_err() {
                json_error(&mut stream, 500, "eZ80 gone");
                return;
            }
            match recv_until(rx_resp, |r| matches!(r, DebugResp::Registers(_))) {
                Some(DebugResp::Registers(r)) => {
                    // ADL registers (SPEC Agon appendix): 24-bit index regs, 16-bit AF,
                    // full 24-bit pc, 8-bit mbase, adl flag 0/1.
                    let body = format!(
                        "{{\"af\":{},\"bc\":{},\"de\":{},\"hl\":{},\"ix\":{},\"iy\":{},\"sp\":{},\"pc\":{},\"mbase\":{},\"adl\":{}}}",
                        r.get16(Reg16::AF),
                        r.get24(Reg16::BC),
                        r.get24(Reg16::DE),
                        r.get24(Reg16::HL),
                        r.get24(Reg16::IX),
                        r.get24(Reg16::IY),
                        r.get24(Reg16::SP),
                        r.pc,
                        r.mbase,
                        r.adl as u8,
                    );
                    respond(&mut stream, 200, "application/json; charset=utf-8", body.as_bytes());
                }
                _ => json_error(&mut stream, 504, "regs read timed out"),
            }
        }
        ("POST", "/step") => {
            let frames = query_int(query, "frames").unwrap_or(1).max(1) as i64;
            {
                let mut s = frame.lock().unwrap();
                s.budget = frames;
            }
            ez80_paused.store(false, Ordering::Relaxed); // resume so the render thread can advance
            let deadline = Instant::now() + STEP_TIMEOUT;
            let final_frame = loop {
                {
                    let s = frame.lock().unwrap();
                    if s.budget == 0 {
                        break s.frame;
                    }
                }
                if Instant::now() > deadline {
                    let f = {
                        let mut s = frame.lock().unwrap();
                        s.budget = 0;
                        s.frame
                    };
                    ez80_paused.store(true, Ordering::Relaxed);
                    break f;
                }
                std::thread::sleep(Duration::from_millis(1));
            };
            respond(&mut stream, 200, "application/json; charset=utf-8",
                    format!("{{\"frame\":{final_frame}}}").as_bytes());
        }
        ("POST", "/pause") => {
            {
                let mut s = frame.lock().unwrap();
                s.budget = 0;
            }
            ez80_paused.store(true, Ordering::Relaxed);
            respond(&mut stream, 200, "application/json; charset=utf-8", b"{\"paused\":true}");
        }
        ("POST", "/resume") => {
            {
                let mut s = frame.lock().unwrap();
                s.budget = -1;
            }
            ez80_paused.store(false, Ordering::Relaxed);
            respond(&mut stream, 200, "application/json; charset=utf-8", b"{\"paused\":false}");
        }
        ("POST", "/key") => {
            // Resolve a fabgl virtual key from ?code=<vk> or ?text=<char>.
            let vk = match query_int(query, "code") {
                Some(code) => code,
                None => match query_str(query, "text").and_then(|t| t.chars().next()) {
                    Some(c) => ascii2vk(c),
                    None => {
                        json_error(&mut stream, 400, "usage: /key?text=<char>|code=<vk>[&down=0|1]");
                        return;
                    }
                },
            };
            if vk == 0 {
                json_error(&mut stream, 400, "unmapped key");
                return;
            }
            // down=1 press, down=0 release, omitted = tap (press then release).
            // The render thread drains this queue and delivers to fabgl.
            let mut q = keys.lock().unwrap();
            match query_int(query, "down") {
                Some(0) => q.push((vk, 0)),
                Some(_) => q.push((vk, 1)),
                None => {
                    q.push((vk, 1));
                    q.push((vk, 0));
                }
            }
            respond(&mut stream, 200, "application/json; charset=utf-8", b"{\"injected\":true}");
        }
        ("POST", "/reset") => {
            soft_reset.store(true, Ordering::Relaxed);
            respond(&mut stream, 200, "application/json; charset=utf-8", b"{\"reset\":true}");
        }
        // Known path, wrong method -> 405; anything else -> 404.
        (_, "/status") | (_, "/screenshot") | (_, "/mem") | (_, "/regs")
        | (_, "/step") | (_, "/pause") | (_, "/resume") | (_, "/key") | (_, "/reset") => {
            json_error(&mut stream, 405, "method not allowed")
        }
        _ => json_error(&mut stream, 404, "not found"),
    }
}

/// Send GetMemory and drain responses until the matching Memory arrives.
fn read_mem(tx: &Sender<DebugCmd>, rx: &Receiver<DebugResp>, start: u32, len: u32) -> Option<Vec<u8>> {
    if len == 0 {
        return Some(Vec::new());
    }
    tx.send(DebugCmd::GetMemory { start, len }).ok()?;
    match recv_until(rx, |r| matches!(r, DebugResp::Memory { start: s, .. } if *s == start)) {
        Some(DebugResp::Memory { data, .. }) => Some(data),
        _ => None,
    }
}

/// POST /mem: poke `data` at `start`; returns bytes written.
fn write_mem(tx: &Sender<DebugCmd>, rx: &Receiver<DebugResp>, start: u32, data: &[u8]) -> Option<u32> {
    if data.is_empty() {
        return Some(0);
    }
    tx.send(DebugCmd::SetMemory { start, data: data.to_vec() }).ok()?;
    match recv_until(rx, |r| matches!(r, DebugResp::MemoryWritten { .. })) {
        Some(DebugResp::MemoryWritten { written }) => Some(written),
        _ => None,
    }
}

/// Read a POST body: the bytes already past the header boundary in `initial`,
/// plus however many more the Content-Length header calls for (capped MEM_CAP).
fn read_body(stream: &mut TcpStream, initial: &[u8], req: &str) -> Vec<u8> {
    let content_length = req
        .lines()
        .filter_map(|l| l.split_once(':'))
        .find(|(k, _)| k.trim().eq_ignore_ascii_case("content-length"))
        .and_then(|(_, v)| v.trim().parse::<usize>().ok())
        .unwrap_or(0)
        .min(MEM_CAP);
    let hdr_end = initial
        .windows(4)
        .position(|w| w == b"\r\n\r\n")
        .map(|i| i + 4)
        .unwrap_or(initial.len());
    let mut body: Vec<u8> = initial[hdr_end..].to_vec();
    while body.len() < content_length {
        let mut chunk = [0u8; 4096];
        match stream.read(&mut chunk) {
            Ok(0) => break,
            Ok(k) => body.extend_from_slice(&chunk[..k]),
            Err(_) => break,
        }
    }
    body.truncate(content_length);
    body
}

/// Wrap the ring's u8 (unsigned, centred on 128) mono samples as a 16-bit
/// signed PCM WAV. An empty slice yields a valid zero-frame WAV.
fn build_wav(samples: &[u8]) -> Vec<u8> {
    let data_bytes = (samples.len() * 2) as u32;
    let byte_rate = AUDIO_RATE * AUDIO_CHANNELS as u32 * 2;
    let mut out = Vec::with_capacity(44 + samples.len() * 2);
    out.extend_from_slice(b"RIFF");
    out.extend_from_slice(&(36 + data_bytes).to_le_bytes());
    out.extend_from_slice(b"WAVE");
    out.extend_from_slice(b"fmt ");
    out.extend_from_slice(&16u32.to_le_bytes());
    out.extend_from_slice(&1u16.to_le_bytes()); // PCM
    out.extend_from_slice(&AUDIO_CHANNELS.to_le_bytes());
    out.extend_from_slice(&AUDIO_RATE.to_le_bytes());
    out.extend_from_slice(&byte_rate.to_le_bytes());
    out.extend_from_slice(&(AUDIO_CHANNELS * 2).to_le_bytes()); // block align
    out.extend_from_slice(&16u16.to_le_bytes()); // bits/sample
    out.extend_from_slice(b"data");
    out.extend_from_slice(&data_bytes.to_le_bytes());
    for &s in samples {
        let s16 = ((s as i16) - 128) << 8;
        out.extend_from_slice(&s16.to_le_bytes());
    }
    out
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
    query_str(query, key).and_then(parse_int)
}

fn query_str<'a>(query: &'a str, key: &str) -> Option<&'a str> {
    query
        .split('&')
        .filter_map(|pair| pair.split_once('='))
        .find(|(k, _)| *k == key)
        .map(|(_, v)| v)
}

fn parse_int(s: &str) -> Option<u32> {
    let s = s.trim();
    match s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        Some(hex) => u32::from_str_radix(hex, 16).ok(),
        None => s.parse::<u32>().ok(),
    }
}

fn json_error(stream: &mut TcpStream, code: u16, msg: &str) {
    let body = format!("{{\"error\":\"{}\"}}", msg);
    respond(stream, code, "application/json; charset=utf-8", body.as_bytes());
}

fn respond(stream: &mut TcpStream, code: u16, content_type: &str, body: &[u8]) {
    respond_ex(stream, code, content_type, body, "");
}

/// Like `respond`, but `extra` is inserted as additional CRLF-terminated header
/// line(s) (e.g. `"X-Rrdc-Audio-Dropped: 0\r\n"`).
fn respond_ex(stream: &mut TcpStream, code: u16, content_type: &str, body: &[u8], extra: &str) {
    let status = match code {
        200 => "200 OK",
        400 => "400 Bad Request",
        404 => "404 Not Found",
        405 => "405 Method Not Allowed",
        500 => "500 Internal Server Error",
        501 => "501 Not Implemented",
        503 => "503 Service Unavailable",
        504 => "504 Gateway Timeout",
        _ => "500 Internal Server Error",
    };
    let header = format!(
        "HTTP/1.1 {}\r\nContent-Type: {}\r\nContent-Length: {}\r\n{}Connection: close\r\n\r\n",
        status,
        content_type,
        body.len(),
        extra
    );
    let _ = stream.write_all(header.as_bytes());
    let _ = stream.write_all(body);
    let _ = stream.flush();
}
