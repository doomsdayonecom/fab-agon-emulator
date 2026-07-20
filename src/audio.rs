use crate::control_server::AudioCapture;
use sdl3::audio::{AudioCallback, AudioStream};

/// Keep at most this many bytes (~4 s at 16384 Hz) of un-read capture, so the
/// buffer stays bounded when nothing is draining /audio.
const CAPTURE_CAP: usize = 64 * 1024;

#[allow(non_snake_case)]
pub struct VdpAudioStream {
    pub buffer: Vec<u8>,
    pub getAudioSamples:
        libloading::Symbol<'static, unsafe extern "C" fn(out: *mut u8, length: u32)>,
    pub capture: AudioCapture,
}
impl AudioCallback<u8> for VdpAudioStream {
    fn callback(&mut self, stream: &mut AudioStream, requested: i32) {
        self.buffer.resize(requested as usize, 0);

        unsafe {
            (*self.getAudioSamples)(&mut self.buffer[0] as *mut u8, requested as u32);
        };

        // Tee a copy into the rolling capture for the /audio endpoint.
        if let Ok(mut cap) = self.capture.lock() {
            cap.extend_from_slice(&self.buffer);
            if cap.len() > CAPTURE_CAP {
                let drop = cap.len() - CAPTURE_CAP;
                cap.drain(0..drop);
            }
        }

        match stream.put_data(&self.buffer) {
            Ok(()) => {}
            Err(err) => println!("Failed to put audio data: {err}"),
        }
    }
}
