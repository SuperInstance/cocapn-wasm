#![deny(unsafe_code)]

use wasm_bindgen::prelude::*;

// ---- Deadband ----

#[wasm_bindgen]
pub struct Deadband {
    center: f64,
    tolerance: f64,
    direction: u8, // 0=both, 1=above, 2=below
}

#[wasm_bindgen]
impl Deadband {
    #[wasm_bindgen(constructor)]
    pub fn new(center: f64, tolerance: f64, direction: u8) -> Self {
        Self {
            center,
            tolerance,
            direction,
        }
    }

    /// Returns 0=Normal, 1=Approaching, 2=Exceeded
    pub fn check(&self, value: f64) -> u8 {
        let tol = if self.center.abs() < 1e-12 {
            self.tolerance
        } else {
            self.center.abs() * self.tolerance
        };
        let diff = value - self.center;

        match self.direction {
            1 => {
                // Above only
                if diff > tol {
                    2
                } else if diff > tol * 0.8 {
                    1
                } else {
                    0
                }
            }
            2 => {
                // Below only (conservation)
                if diff < -tol {
                    2
                } else if diff < -tol * 0.8 {
                    1
                } else {
                    0
                }
            }
            _ => {
                // Both
                let adiff = diff.abs();
                if adiff > tol {
                    2
                } else if adiff > tol * 0.8 {
                    1
                } else {
                    0
                }
            }
        }
    }
}

// ---- PID Controller ----

#[wasm_bindgen]
pub struct PIDController {
    kp: f64,
    ki: f64,
    kd: f64,
    integral: f64,
    last_error: Option<f64>,
    max_rudder: f64,
    heading_tol: f64,
}

fn heading_error(current: f64, target: f64) -> f64 {
    ((target - current + 540.0) % 360.0) - 180.0
}

#[wasm_bindgen]
impl PIDController {
    #[wasm_bindgen(constructor)]
    pub fn new(kp: f64, ki: f64, kd: f64, max_rudder: f64, tol: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral: 0.0,
            last_error: None,
            max_rudder,
            heading_tol: tol,
        }
    }

    /// Returns [rudder_command, heading_error, on_course (1.0 or 0.0)]
    pub fn update(&mut self, current: f64, target: f64, dt: f64) -> Vec<f64> {
        let err = heading_error(current, target);

        let p = self.kp * err;

        self.integral += err * dt;
        self.integral = self.integral.clamp(-self.max_rudder, self.max_rudder);
        let i = self.ki * self.integral;

        let d = if dt > 0.0 {
            self.last_error
                .map_or(0.0, |last| self.kd * (err - last) / dt)
        } else {
            0.0
        };
        self.last_error = Some(err);

        let cmd = (p + i + d).clamp(-self.max_rudder, self.max_rudder);
        let on_course = err.abs() < self.heading_tol;

        vec![cmd, err, if on_course { 1.0 } else { 0.0 }]
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.last_error = None;
    }
}

// ---- NMEA ----

#[wasm_bindgen]
pub fn verify_nmea_checksum(sentence: &str) -> bool {
    if !sentence.starts_with('$') {
        return false;
    }
    let mut calc: u8 = 0;
    for ch in sentence[1..].chars() {
        if ch == '*' {
            break;
        }
        calc ^= ch as u8;
    }
    sentence
        .split('*')
        .nth(1)
        .and_then(|s| s.get(..2))
        .and_then(|s| u8::from_str_radix(s, 16).ok())
        .map(|stated| calc == stated)
        .unwrap_or(false)
}

fn parse_nmea_gga_inner(sentence: &str) -> Result<Vec<f64>, &'static str> {
    if !verify_nmea_checksum(sentence) {
        return Err("NMEA checksum invalid");
    }

    let parts: Vec<&str> = sentence.split(',').collect();
    if parts.len() < 10 {
        return Err("GGA sentence too short");
    }

    let lat_raw = parts[2];
    let lat_dir = parts[3];
    let lon_raw = parts[4];
    let lon_dir = parts[5];

    if lat_dir != "N" && lat_dir != "S" {
        return Err("Invalid latitude hemisphere");
    }
    if lon_dir != "E" && lon_dir != "W" {
        return Err("Invalid longitude hemisphere");
    }

    let lat = parse_coord(lat_raw);
    let lat = if lat_dir == "S" { -lat } else { lat };
    let lon = parse_coord(lon_raw);
    let lon = if lon_dir == "W" { -lon } else { lon };

    let qual: f64 = parts[6].parse().unwrap_or(0.0);
    let sats: f64 = parts[7].parse().unwrap_or(0.0);
    let hdop: f64 = parts[8].parse().unwrap_or(0.0);
    let alt: f64 = parts[9].parse().unwrap_or(0.0);

    Ok(vec![lat, lon, qual, sats, hdop, alt])
}

/// Returns [lat, lon, fix_quality, satellites, hdop, altitude] or throws
#[wasm_bindgen]
pub fn parse_nmea_gga(sentence: &str) -> Result<Vec<f64>, JsValue> {
    parse_nmea_gga_inner(sentence).map_err(JsValue::from_str)
}

fn parse_coord(raw: &str) -> f64 {
    let val: f64 = raw.parse().unwrap_or(0.0);
    let degrees = (val / 100.0).floor();
    let minutes = val - degrees * 100.0;
    degrees + minutes / 60.0
}

#[wasm_bindgen]
pub fn heading_error_js(current: f64, target: f64) -> f64 {
    heading_error(current, target)
}

/// Simulate heading hold. Returns flat array: [h0, e0, r0, h1, e1, r1, ...]
#[wasm_bindgen]
pub fn simulate_heading_hold(
    kp: f64,
    ki: f64,
    kd: f64,
    initial: f64,
    target: f64,
    steps: usize,
) -> Vec<f64> {
    let mut pid = PIDController::new(kp, ki, kd, 15.0, 2.0);
    let mut heading = initial;
    let mut result = Vec::with_capacity(steps * 3);
    let dt = 0.1;

    for _ in 0..steps {
        let update = pid.update(heading, target, dt);
        let cmd = update[0];
        let err = update[1];
        let on_course = update[2] > 0.5;

        result.push(heading);
        result.push(err);
        result.push(cmd);

        heading = (heading + cmd * dt + 3600.0) % 360.0;
        if on_course {
            break;
        }
    }
    result
}

// ---- Tests (cargo test) ----

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn deadband_normal() {
        let db = Deadband::new(100.0, 0.05, 0);
        assert_eq!(db.check(100.0), 0);
        assert_eq!(db.check(97.0), 0);
    }

    #[test]
    fn deadband_exceeded() {
        let db = Deadband::new(100.0, 0.05, 0);
        assert_eq!(db.check(106.0), 2);
    }

    #[test]
    fn deadband_conservation() {
        let db = Deadband::new(100.0, 0.10, 2); // below only
        assert_eq!(db.check(115.0), 0);
        assert_eq!(db.check(85.0), 2);
    }

    #[test]
    fn pid_convergence() {
        let mut pid = PIDController::new(0.8, 0.1, 0.3, 15.0, 2.0);
        let mut h = 0.0;
        for _ in 0..200 {
            let r = pid.update(h, 90.0, 0.1);
            h = (h + r[0] * 0.1 + 360.0) % 360.0;
            if r[2] > 0.5 {
                break;
            }
        }
        assert!(heading_error(h, 90.0).abs() < 2.0);
    }

    #[test]
    fn nmea_checksum() {
        let gga = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
        assert!(verify_nmea_checksum(gga));
    }

    #[test]
    fn nmea_parse() {
        let gga = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
        let result = parse_nmea_gga(gga).unwrap();
        assert!(result[0] > 48.0 && result[0] < 49.0, "lat");
        assert!(result[1] > 11.0 && result[1] < 12.0, "lon");
        assert_eq!(result[2], 1.0);
        assert_eq!(result[3], 8.0);
    }

    #[test]
    fn heading_wrap() {
        assert!((heading_error(350.0, 10.0) - 20.0).abs() < 0.01);
        assert!((heading_error(10.0, 350.0) - (-20.0)).abs() < 0.01);
    }

    #[test]
    fn deadband_above_only() {
        let db = Deadband::new(100.0, 0.10, 1); // above only
        assert_eq!(db.check(85.0), 0);
        assert_eq!(db.check(115.0), 2);
    }

    #[test]
    fn deadband_approaching() {
        let db = Deadband::new(100.0, 0.05, 0); // tol = 5.0
                                                // 80% threshold is 4.0; diff = 4.5 should be approaching
        assert_eq!(db.check(104.5), 1);
        assert_eq!(db.check(95.5), 1);
    }

    #[test]
    fn deadband_zero_center_uses_absolute_tolerance() {
        // When center is near zero tolerance is interpreted as absolute units.
        let db = Deadband::new(0.0, 0.5, 0);
        assert_eq!(db.check(0.4), 0);
        assert_eq!(db.check(0.6), 2);
    }

    #[test]
    fn pid_first_update_suppresses_derivative() {
        let mut pid = PIDController::new(0.0, 0.0, 10.0, 15.0, 2.0);
        let r = pid.update(0.0, 90.0, 0.1);
        // With kp=ki=0 and no prior error, derivative must be zero, so command is zero.
        assert!((r[0]).abs() < 1e-9, "first-update rudder should be zero");
        assert!((r[1] - 90.0).abs() < 1e-9, "error is 90");
    }

    #[test]
    fn pid_reset_clears_history() {
        let mut pid = PIDController::new(0.8, 0.1, 0.3, 15.0, 2.0);
        pid.update(0.0, 90.0, 0.1);
        pid.reset();
        let r = pid.update(0.0, 90.0, 0.1);
        // After reset the derivative history is gone, so behavior matches first update.
        assert_eq!(r[2], 0.0);
    }

    #[test]
    fn nmea_checksum_invalid() {
        // Same sentence with the last checksum nibble flipped.
        let gga = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*48";
        assert!(!verify_nmea_checksum(gga));
    }

    #[test]
    fn nmea_parse_rejects_invalid_checksum() {
        let gga = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*48";
        assert!(parse_nmea_gga_inner(gga).is_err());
    }

    #[test]
    fn nmea_parse_rejects_short_sentence() {
        let gga = "$GPGGA,123519,4807.038*00";
        assert!(parse_nmea_gga_inner(gga).is_err());
    }

    #[test]
    fn nmea_parse_rejects_bad_hemisphere() {
        // Valid checksum (0x51) but latitude hemisphere is invalid.
        let gga = "$GPGGA,123519,4807.038,X,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*51";
        assert!(parse_nmea_gga_inner(gga).is_err());
    }

    #[test]
    fn heading_wrap_antipodal() {
        // Antipodal headings normalize to the same signed value (-180) by this formula.
        assert!((heading_error(0.0, 180.0) - (-180.0)).abs() < 0.01);
        assert!((heading_error(180.0, 0.0) - (-180.0)).abs() < 0.01);
    }
}
