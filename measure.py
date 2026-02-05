"""
MicroPython LED-to-Phototransistor Latency Measurement
Designed for RP2040 (Raspberry Pi Pico)

Measures the time latency between LED activation and ADC voltage change detection
using precise timing and edge detection on the analog input.

Includes WS2812 NeoPixel status LED on GPIO16 for visual feedback.
"""

from machine import Pin, ADC
import neopixel
import time
import urandom
import json
import gc

# NeoPixel color definitions (GRB format for WS2812)
COLOR_BLACK = (0, 0, 0)
COLOR_RED = (0, 255, 0)
COLOR_GREEN = (255, 0, 0)
COLOR_BLUE = (0, 0, 255)
COLOR_YELLOW = (255, 255, 0)
COLOR_CYAN = (255, 0, 255)
COLOR_MAGENTA = (0, 255, 255)
COLOR_WHITE = (255, 255, 255)

# Standard frame rates for camera/display systems (Hz)
COMMON_FRAME_RATES = [24, 30, 60, 90, 100, 120, 144]


class CalibrationData:
    """Data structure for calibration results."""

    def __init__(self, baseline_adc, led_on_adc):
        self.baseline_adc = baseline_adc
        self.led_on_adc = led_on_adc
        #self.edge_threshold = ((self.baseline_adc + led_on_adc)) // 2
        delta = abs(baseline_adc - led_on_adc)
        self.rising_edge = (led_on_adc > baseline_adc)
        self.edge_threshold = self.baseline_adc + (1 if self.rising_edge else -1) * ((delta // 2))


class TimingCalibration:
    """Data structure for timing calibration results."""

    def __init__(self, settle_time_ms, recovery_time_ms):
        self.settle_time_ms = settle_time_ms
        self.recovery_time_ms = recovery_time_ms


class FrameRateConfig:
    """Configuration for frame-rate-aware pulse timing."""

    def __init__(self, frame_rates=None):
        """
        Initialize frame rate configuration.

        Args:
            frame_rates (list): Frame rates in Hz to test (default: common rates)
        """
        self.frame_rates = frame_rates if frame_rates else COMMON_FRAME_RATES
        self.frame_periods_us = [1_000_000 // rate for rate in self.frame_rates]
        self.max_frame_period_us = max(self.frame_periods_us)
        self.avg_frame_period_us = sum(self.frame_periods_us) // len(self.frame_periods_us)


class NeoPixelInterferenceTest:
    """Data structure for NeoPixel interference test results."""

    def __init__(self, neopixel_safe, latency_with_neopixel_ns, 
                 latency_without_neopixel_ns, variance_ns):
        self.neopixel_safe = neopixel_safe
        self.latency_with_neopixel_ns = latency_with_neopixel_ns
        self.latency_without_neopixel_ns = latency_without_neopixel_ns
        self.variance_ns = variance_ns


class MeasurementResult:
    """Data structure for a single latency measurement."""

    def __init__(self, measurement_num, latency_ns, edge_detected=True, frame_phase_us=0):
        self.measurement_num = measurement_num
        self.latency_ns = latency_ns
        self.edge_detected = edge_detected
        self.frame_phase_us = frame_phase_us


class LatencyStatistics:
    """Data structure for latency measurement statistics."""

    def __init__(self, num_measurements, min_latency_ns, max_latency_ns, 
                 avg_latency_ns, std_dev_ns, all_latencies_ns, all_measurement_tuples, frame_rate_hz=None):
        self.num_measurements = num_measurements
        self.min_latency_ns = min_latency_ns
        self.max_latency_ns = max_latency_ns
        self.avg_latency_ns = avg_latency_ns
        self.std_dev_ns = std_dev_ns
        self.all_latencies_ns = all_latencies_ns
        self.frame_rate_hz = frame_rate_hz
        self.all_measurement_tuples = all_measurement_tuples

    def to_dict(self):
        """Convert to dictionary for serialization."""
        return {
            'num_measurements': self.num_measurements,
            'min_latency_ns': self.min_latency_ns,
            'max_latency_ns': self.max_latency_ns,
            'avg_latency_ns': self.avg_latency_ns,
            'std_dev_ns': self.std_dev_ns,
            'min_latency_us': self.min_latency_ns / 1000,
            'max_latency_us': self.max_latency_ns / 1000,
            'avg_latency_us': self.avg_latency_ns / 1000,
            'std_dev_us': self.std_dev_ns / 1000,
            'all_latencies_ns': self.all_latencies_ns,
            'all_measurement_tiples': self.all_measurement_tuples,
            'frame_rate_hz': self.frame_rate_hz
        }


class LEDLatencyMeasurement:
    """
    Measures the latency between LED on command and phototransistor response detection.
    All output is returned as structured data objects, not printed.
    Optimized for RP2040 (Raspberry Pi Pico) ADC with dynamic timing calibration.
    Includes WS2812 NeoPixel status LED on GPIO16.

    Supports frame-rate-aware pulse timing to detect aliasing with camera/display
    frame rates and ensure measurements span the entire frame period.
    """

    def __init__(self, led_pin, adc_pin, neopixel_pin=16, frame_rates=None):
        """
        Initialize the latency measurement system.

        Args:
            led_pin (int): GPIO pin number for LED digital output control
            adc_pin (int): GPIO pin number for ADC input (voltage divider from phototransistor)
                          Valid pins: 26, 27, 28, 29 (ADC0-3)
            neopixel_pin (int): GPIO pin for WS2812 NeoPixel status LED (default 16)
            frame_rates (list): Frame rates in Hz to optimize for (default: common rates)
        """
        self.led_pin = Pin(led_pin, Pin.OUT)
        self.adc_pin = ADC(adc_pin)
        self.neopixel = neopixel.NeoPixel(Pin(neopixel_pin), 1)

        # Frame rate configuration
        self.frame_rate_config = FrameRateConfig(frame_rates)

        # RP2040 ADC notes:
        # - Always returns 16-bit values (0-65535)
        # - Input range: 0-3.3V
        # - No atten() or width() configuration needed

        # Timing variables
        self.led_on_time = 0
        self.edge_detected_time = 0
        self.latency_ns = 0
        self.baseline_adc_value = 0
        self.edge_threshold = 0

        # Calibrated timing (will be set by calibration)
        self.settle_time_ms = 50
        self.recovery_time_ms = 50

        # NeoPixel interference check
        self.neopixel_safe = True

    def set_neopixel(self, color):
        """Set NeoPixel to specified color and write."""
        self.neopixel[0] = color
        self.neopixel.write()

    def _calculate_frame_phase_offset(self, measurement_num):
        """
        Calculate a randomized frame phase offset to prevent aliasing.

        Distributes measurements across the entire frame period for each frame rate,
        ensuring that latency measurements are not correlated with any single frame rate.

        Args:
            measurement_num (int): Sequential measurement number

        Returns:
            int: Offset in microseconds (0 to max_frame_period)
        """
        # Use pseudo-random offset within the largest frame period
        # This ensures sampling across the frame cycle for all frame rates
        random_offset = urandom.getrandbits(16) % self.frame_rate_config.max_frame_period_us
        return random_offset

    def _measure_single_latency(self, frame_phase_offset_us=0):
        """
        Measure a single latency value with optional frame phase offset.

        Args:
            frame_phase_offset_us (int): Optional offset in microseconds for frame-phase testing

        Returns:
            tuple: (latency_ns, frame_phase_us) or (None, 0) if edge not detected
        """
        self.led_pin.off()
        time.sleep_ms(self.recovery_time_ms)

        # Add frame phase offset if specified
        if frame_phase_offset_us > 0:
            time.sleep_us(frame_phase_offset_us)

        self.led_on_time = time.time_ns()
        self.led_pin.on()

        edge_detected = False
        timeout_ns = 1_000_000_000
        current_adc = None
        cycles = 0

        while not edge_detected:
            cycles += 1
            current_adc = self.adc_pin.read_u16()
            
            current_time_ns = time.time_ns()
            edge_delta = (current_adc - self.edge_threshold) * (1 if self.rising_edge else -1)

            if edge_delta > 0:
                self.edge_detected_time = current_time_ns
                edge_detected = True
                self.led_pin.off()
                break

            if (current_time_ns - self.led_on_time) > timeout_ns:
                self.led_pin.off()
                break
        
        gc.collect()
        if edge_detected:
            return self.edge_detected_time - self.led_on_time, cycles, frame_phase_offset_us
        else:
            return None, 0, 0

    def _calibrate_settle_time(self, max_settle_ms=200):
        """
        Dynamically calibrate the minimum settle and recovery times needed for ADC stability.
        Measures actual LED settle time and actual phototransistor discharge (recovery) time.

        Args:
            max_settle_ms (int): Maximum settle time to test (default 200ms)

        Returns:
            TimingCalibration: Object with measured settle_time_ms and recovery_time_ms
        """
        self.set_neopixel(COLOR_YELLOW)

        # Turn off LED and establish baseline
        self.led_pin.off()
        time.sleep_ms(100)
        baseline = self.adc_pin.read_u16()

        # Turn on LED and measure time to stabilization
        self.led_pin.on()
        time.sleep_ms(10)  # Initial delay before sampling

        prev_value = self.adc_pin.read_u16()
        stable_count = 0
        required_stable_samples = 5  # Need 5 consecutive stable readings
        settle_time_ms = 10

        for elapsed_ms in range(10, max_settle_ms, 5):
            time.sleep_ms(5)
            current_value = self.adc_pin.read_u16()

            # Check if value is stable (within 5 counts of previous)
            if abs(current_value - prev_value) <= 5:
                stable_count += 1
                if stable_count >= required_stable_samples:
                    settle_time_ms = elapsed_ms
                    break
            else:
                stable_count = 0

            prev_value = current_value

        # Now measure recovery time - how long to return to baseline
        self.led_pin.off()
        time.sleep_ms(10)

        prev_value = self.adc_pin.read_u16()
        stable_count = 0
        recovery_time_ms = 10  # Start from 10ms, let calibration determine actual time

        for elapsed_ms in range(10, max_settle_ms, 5):
            time.sleep_ms(5)
            current_value = self.adc_pin.read_u16()

            # Check if value returned to baseline (within 5 counts)
            if abs(current_value - baseline) <= 5:
                stable_count += 1
                if stable_count >= required_stable_samples:
                    recovery_time_ms = elapsed_ms
                    break
            else:
                stable_count = 0

            prev_value = current_value

        # Store calibrated values - trust the actual measurement
        self.settle_time_ms = settle_time_ms
        self.recovery_time_ms = recovery_time_ms

        return TimingCalibration(settle_time_ms, recovery_time_ms)

    def _calibrate_baseline(self, samples=100):
        """
        Calibrate the baseline ADC value when LED is off.

        Args:
            samples (int): Number of samples to average for baseline

        Returns:
            int: Average baseline ADC value
        """
        total = 0
        for _ in range(samples):
            total += self.adc_pin.read_u16()

        self.baseline_adc_value = total // samples
        return self.baseline_adc_value

    def _determine_threshold(self, led_on_samples=100):
        """
        Determine the ADC threshold for edge detection by sampling with LED on.
        Detects falling edge (ADC value drops when phototransistor is illuminated).
        Uses calibrated settle time.

        Args:
            led_on_samples (int): Number of samples to take with LED on

        Returns:
            CalibrationData: Calibration results object
        """
        # Turn on LED and let it stabilize using calibrated time
        self.led_pin.on()
        time.sleep_ms(self.settle_time_ms)
        time.sleep(0.2)

        total = 0
        for _ in range(led_on_samples):
            total += self.adc_pin.read_u16()

        led_on_value = total // led_on_samples
        self.led_pin.off()

        calibration_data = CalibrationData(self.baseline_adc_value, led_on_value)
        
        # Delegate some calculations such as rising/falling and threshold to the calibration data object.
        self.edge_threshold = calibration_data.edge_threshold
        self.rising_edge = calibration_data.rising_edge
        print(calibration_data.__dict__)
        
        return calibration_data

    def measure_latency_with_interference_test(self, num_measurements=10, delay_between_ms=None):
        """
        Perform latency measurements with integrated NeoPixel interference testing.
        Includes frame-phase randomization to prevent aliasing.

        First 20 measurements with NeoPixel OFF, then 20 with NeoPixel ON to compare.
        Remaining measurements use randomized frame phase offsets.

        Args:
            num_measurements (int): Number of measurements to perform
            delay_between_ms (int): Delay between measurements in milliseconds (optional)

        Returns:
            tuple: (LatencyStatistics, NeoPixelInterferenceTest)
        """
        # Disable the garbage collection for the duration of this function
        #gc.disable()
        
        if delay_between_ms is None:
            delay_between_ms = self.recovery_time_ms

        interference_test_count = min(20, num_measurements // 2)

        # Phase 1: Measure with NeoPixel OFF
        self.set_neopixel(COLOR_BLACK)
        time.sleep_ms(50)

        measurements_without = []
        for _ in range(interference_test_count):
            latency, cycle_count, phase = self._measure_single_latency()
            if latency is not None:
                measurements_without.append((latency, cycle_count, phase))
            time.sleep_ms(delay_between_ms)

        gc.collect()
        latencies_without = [m[0] for m in measurements_without]
        
        # Phase 2: Measure with NeoPixel ON
        self.set_neopixel(COLOR_GREEN)
        time.sleep_ms(50)

        measurements_with = []
        for _ in range(interference_test_count):
            latency, cycle_count, phase = self._measure_single_latency()
            if latency is not None:
                measurements_with.append((latency, cycle_count, phase))
            time.sleep_ms(delay_between_ms)
            
        gc.collect()
        latencies_with = [m[0] for m in measurements_with]

        # Phase 3: Continue measurements with NeoPixel ON and frame-phase randomization
        self.set_neopixel(COLOR_CYAN)

        measurements_main = []
        for measurement_num in range(num_measurements - interference_test_count * 2):
            # Calculate randomized frame phase offset
            frame_phase_offset = self._calculate_frame_phase_offset(measurement_num)
            latency, cycle_count, phase = self._measure_single_latency(frame_phase_offset_us=frame_phase_offset)
            if latency is not None:
                measurements_main.append((latency, cycle_count, phase))
            time.sleep_ms(delay_between_ms)

        gc.collect()
        gc.enable()
        latencies_main = [m[0] for m in measurements_main]

        # Combine all latencies for final statistics
        all_measurements = measurements_without + measurements_with + measurements_main
        all_latencies = latencies_without + latencies_with + latencies_main

        # Calculate interference test results
        avg_without = sum(latencies_without) // len(latencies_without) if latencies_without else 0
        avg_with = sum(latencies_with) // len(latencies_with) if latencies_with else 0
        variance = abs(avg_with - avg_without)
        neopixel_safe = variance < (avg_without * 0.05) if avg_without > 0 else True

        neopixel_test = NeoPixelInterferenceTest(neopixel_safe, avg_with, avg_without, variance)
        self.neopixel_safe = neopixel_safe

        # Calculate main statistics
        if all_latencies:
            min_latency_ns = min(all_latencies)
            max_latency_ns = max(all_latencies)
            avg_latency_ns = sum(all_latencies) // len(all_latencies)

            variance_calc = sum((x - avg_latency_ns) ** 2 for x in all_latencies) / len(all_latencies)
            std_dev_ns = int(variance_calc ** 0.5)

            stats = LatencyStatistics(
                num_measurements=len(all_latencies),
                min_latency_ns=min_latency_ns,
                max_latency_ns=max_latency_ns,
                avg_latency_ns=avg_latency_ns,
                std_dev_ns=std_dev_ns,
                all_latencies_ns=all_latencies,
                all_measurement_tuples=all_measurements
            )

            return stats, neopixel_test
        else:
            return None, neopixel_test

    def calculate_optimal_measurements(self, target_test_time_ms=5000):
        """
        Calculate the optimal number of measurements to fit within target test time.

        Uses MEASURED recovery time from calibration, not assumptions.
        Accounts for recovery time, latency, and randomized frame phase offset.

        Args:
            target_test_time_ms (int): Target total test duration in milliseconds

        Returns:
            int: Optimal number of measurements
        """
        # Time per measurement = recovery_time + latency (assume ~100µs) + average frame offset + delay_between
        # Average frame offset is max_frame_period / 2 (since it's uniformly distributed 0 to max)
        avg_frame_offset_ms = self.frame_rate_config.max_frame_period_us // 2000
        # Delay between measurements uses the measured recovery_time_ms
        time_per_measurement_ms = (self.recovery_time_ms * 2) + 0.1 + avg_frame_offset_ms

        if time_per_measurement_ms > 0:
            num_measurements = target_test_time_ms / time_per_measurement_ms
            return max(1, int(num_measurements))
        else:
            return 1

    def calibrate_and_measure(self, target_test_time_ms=5000):
        """
        Complete calibration and measurement sequence with dynamic timing and NeoPixel feedback.
        Includes integrated NeoPixel interference testing during measurement phase.
        Uses frame-rate-aware pulse timing to prevent aliasing.

        Args:
            target_test_time_ms (int): Target total test duration in milliseconds

        Returns:
            tuple: (TimingCalibration, CalibrationData, NeoPixelInterferenceTest, LatencyStatistics)
        """
        # Step 1: Calibrate timing
        timing_calib = self._calibrate_settle_time()

        # Step 2: Calibrate baseline
        self._calibrate_baseline()

        # Step 3: Determine threshold
        data_calib = self._determine_threshold()

        # Step 4: Calculate optimal number of measurements
        num_measurements = self.calculate_optimal_measurements(target_test_time_ms)

        # Step 5: Perform measurements with integrated NeoPixel interference testing
        statistics, neopixel_test = self.measure_latency_with_interference_test(
            num_measurements=num_measurements,
            delay_between_ms=self.recovery_time_ms
        )

        # Step 6: Set final NeoPixel color based on success
        if statistics is not None and self.neopixel_safe:
            self.set_neopixel(COLOR_GREEN)
        elif statistics is not None:
            self.set_neopixel(COLOR_YELLOW)
        else:
            self.set_neopixel(COLOR_RED)

        return timing_calib, data_calib, neopixel_test, statistics


def print_timing_calibration(timing_calib):
    """Pretty print timing calibration data."""
    print("Timing Calibration Results:")
    print(f"  LED settle time: {timing_calib.settle_time_ms} ms")
    print(f"  ADC recovery time: {timing_calib.recovery_time_ms} ms")


def print_calibration_data(calib_data):
    """Pretty print calibration data."""
    print("\nADC Calibration Results:")
    print(f"  Baseline ADC value (LED off): {calib_data.baseline_adc}")
    print(f"  LED on ADC value: {calib_data.led_on_adc}")
    print(f"  Edge detection threshold: {calib_data.edge_threshold}")


def print_neopixel_test(neopixel_test):
    """Pretty print NeoPixel interference test results."""
    print("\nNeoPixel Interference Test:")
    print(f"  Latency without NeoPixel: {neopixel_test.latency_without_neopixel_ns / 1000:.2f} µs")
    print(f"  Latency with NeoPixel: {neopixel_test.latency_with_neopixel_ns / 1000:.2f} µs")
    print(f"  Variance: {neopixel_test.variance_ns / 1000:.2f} µs")
    if neopixel_test.neopixel_safe:
        print(f"  Status: SAFE - NeoPixel does not affect measurements")
    else:
        print(f"  Status: WARNING - NeoPixel may affect measurements")


def print_statistics(stats):
    """Pretty print latency statistics."""
    if stats is None:
        print("No measurements completed!")
        return

    print("\nLatency Statistics:")
    print(f"  Measurements completed: {stats.num_measurements}")
    print(f"  Minimum latency: {stats.min_latency_ns / 1000:.2f} µs ({stats.min_latency_ns} ns)")
    print(f"  Maximum latency: {stats.max_latency_ns / 1000:.2f} µs ({stats.max_latency_ns} ns)")
    print(f"  Average latency: {stats.avg_latency_ns / 1000:.2f} µs ({stats.avg_latency_ns} ns)")
    print(f"  Std deviation:   {stats.std_dev_ns / 1000:.2f} µs ({stats.std_dev_ns} ns)")


def print_measurement_header(num_measurements):
    """Print measurement header."""
    print(f"\nStarting {num_measurements} latency measurements (frame-rate aware)...")
    print("=" * 60)


# Example usage for RP2040 (Raspberry Pi Pico)
if __name__ == "__main__":
    # Configuration - adjust these pins based on your wiring
    LED_PIN = 7  # GPIO pin for LED control
    ADC_PIN = 26  # ADC0 on GPIO26 (or 27, 28, 29 for ADC1-3)
    NEOPIXEL_PIN = 16  # WS2812 NeoPixel on GPIO16

    # Frame rates to test (Hz) - adjust for your camera/display system
    FRAME_RATES = [24, 30, 60, 90, 100, 120, 144]

    # Create measurement instance with frame rate configuration
    meter = LEDLatencyMeasurement(LED_PIN, ADC_PIN, NEOPIXEL_PIN, frame_rates=FRAME_RATES)

    # Print calibration header
    print("Starting comprehensive calibration sequence...")
    print("-" * 60)
    print(f"Frame rates configured: {FRAME_RATES} Hz")
    print("Using randomized frame-phase offsets to prevent aliasing")
    print("\nStep 0: Timing calibration...")

    # Perform complete calibration and measurement sequence with dynamic timing
    timing_calib, data_calib, neopixel_test, statistics = meter.calibrate_and_measure(target_test_time_ms=5000)

    # Print results using helper functions
    print("\nStep 1: Timing calibration complete")
    print_timing_calibration(timing_calib)
    print("\nStep 2: ADC calibration complete")
    print_calibration_data(data_calib)
    print("\nStep 3: Performing latency measurements with NeoPixel interference test...")

    if statistics is not None:
        print_measurement_header(statistics.num_measurements)
        print("=" * 60)
        print_neopixel_test(neopixel_test)
        print_statistics(statistics)

        # Statistics can also be accessed as a dictionary
        stats_dict = statistics.to_dict()

        # Calculate actual test duration
        actual_duration_ms = (statistics.num_measurements * 
                             (timing_calib.recovery_time_ms + 0.1))
        print(f"\nActual test duration: ~{actual_duration_ms:.0f} ms")

        if neopixel_test.neopixel_safe:
            print("\nNeoPixel Status: SAFE for use during measurements")
        else:
            print("\nNeoPixel Status: CAUTION - minimal interference detected")

        print("Measurement completed successfully!")
        print(json.dumps(statistics.to_dict()))
    else:
        print("ERROR: No edges detected during measurement!")
        print("Check your voltage divider and phototransistor connection.")

