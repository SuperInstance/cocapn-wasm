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
    last_error: f64,
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
            last_error: 0.0,
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
            self.kd * (err - self.last_error) / dt
        } else {
            0.0
        };
        self.last_error = err;

        let cmd = (p + i + d).clamp(-self.max_rudder, self.max_rudder);
        let on_course = err.abs() < self.heading_tol;

        vec![cmd, err, if on_course { 1.0 } else { 0.0 }]
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.last_error = 0.0;
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

/// Returns [lat, lon, fix_quality, satellites, hdop, altitude] or throws
#[wasm_bindgen]
pub fn parse_nmea_gga(sentence: &str) -> Result<Vec<f64>, JsValue> {
    let parts: Vec<&str> = sentence.split(',').collect();
    if parts.len() < 10 {
        return Err(JsValue::from_str("GGA sentence too short"));
    }

    let lat_raw = parts[2];
    let lat_dir = parts[3];
    let lon_raw = parts[4];
    let lon_dir = parts[5];

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
}
