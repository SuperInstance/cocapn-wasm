# cocapn-wasm

CoCapn for the browser — deadbands, PID, and marine logic compiled to WebAssembly.

The same deadband that runs on the ESP32. The same PID that holds the heading. The same NMEA parser that reads the GPS. Now in Chrome.

## Why WASM?

The captain's dashboard runs in a browser. The boat monitoring UI is a web app. OpenCPN has a web interface. WASM lets you run the exact same logic — not a port, not a rewrite, the *same Rust code* — in the browser.

## Build

```bash
wasm-pack build --target web
```

Then serve `index.html` with any static server.

## Demo

`index.html` includes an interactive demo:
- **Deadband checker** — input center/tolerance/value, see state
- **PID simulator** — adjust gains, watch convergence on canvas
- **NMEA parser** — paste a sentence, see parsed data

## API

```javascript
import init, { Deadband, PIDController, verify_nmea_checksum, parse_nmea_gga, simulate_heading_hold } from './pkg/cocapn_wasm.js';

await init();

// Deadband
const db = new Deadband(100.0, 0.05, 0);  // center, tolerance, direction (0=both,1=above,2=below)
db.check(97.0);   // 0 = NORMAL
db.check(106.0);  // 2 = EXCEEDED

// PID
const pid = new PIDController(0.8, 0.1, 0.3, 15.0, 2.0);
const [rudder, error, onCourse] = pid.update(currentHeading, targetHeading, dt);

// NMEA
verify_nmea_checksum("$GPGGA,...*47");  // true/false
const [lat, lon, quality, sats, hdop, alt] = parse_nmea_gga(sentence);

// Simulation
const data = simulate_heading_hold(0.8, 0.1, 0.3, 0, 90, 300);
// Returns flat array: [heading0, error0, rudder0, heading1, ...]
```

## Size

Compiled with `opt-level = "z"` and LTO. Target: < 20KB gzipped.

## Tests

```bash
cargo test
```

7 tests: deadband (normal, exceeded, conservation), PID convergence, NMEA checksum + parsing, heading wrap.

## License

MIT
