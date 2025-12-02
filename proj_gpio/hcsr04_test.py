#!/usr/bin/env python3
"""
HC-SR04 Ultrasonic Distance Sensor
Banana Pi BIP-F3: GPIO-77(Echo Input), GPIO-70(Trigger Output)

通过GPIO 输入输出功能实现超声波模块的测距
以下示例展示如何控制 GPIO 输入输出信号来实现超声波模块的测距

物理连接
HC-SR04          香蕉派BIP-F3
--------         ------------
VCC       -->    5V
GND       -->    GND
Trig      -->    GPIO-70 (输出)
Echo      -->    GPIO-77 (输入)
"""

from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device, DistanceSensor
import time
import warnings

# Configure pin factory
Device.pin_factory = LGPIOFactory(chip=0)

# Pin configuration
ECHO_PIN = 77   # GPIO-77 Input (Echo)
TRIGGER_PIN = 70  # GPIO-70 Output (Trigger)

def test_gpio_pins():
    """Test GPIO pins before using sensor"""
    print("=" * 60)
    print("GPIO Pin Test")
    print("=" * 60)
    
    try:
        from gpiozero import DigitalInputDevice, DigitalOutputDevice
        
        # Test Trigger pin (Output)
        print(f"Testing GPIO-{TRIGGER_PIN} (Trigger - Output)...")
        trigger = DigitalOutputDevice(TRIGGER_PIN)
        trigger.off()
        time.sleep(0.1)
        trigger.on()
        time.sleep(0.1)
        trigger.off()
        print(f"  [OK] GPIO-{TRIGGER_PIN} works as output")
        trigger.close()
        
        # Test Echo pin (Input)
        print(f"\nTesting GPIO-{ECHO_PIN} (Echo - Input)...")
        echo = DigitalInputDevice(ECHO_PIN)
        echo_value = echo.value
        print(f"  Current Echo pin state: {echo_value}")
        
        if echo_value == 1:
            print("  [WARNING] Echo pin is HIGH! Should be LOW when idle.")
            print("  Possible issues:")
            print("    1. HC-SR04 not powered")
            print("    2. Wrong pin number")
            print("    3. Loose connection")
            print("    4. Damaged sensor")
        else:
            print(f"  [OK] GPIO-{ECHO_PIN} is LOW (correct idle state)")
        
        echo.close()
        print("\n" + "=" * 60)
        return echo_value == 0
        
    except Exception as e:
        print(f"[ERROR] Pin test failed: {e}")
        return False

def main():
    """Main program"""
    print("=" * 60)
    print("HC-SR04 Ultrasonic Distance Sensor - Banana Pi BIP-F3")
    print("=" * 60)
    print(f"Config: Echo=GPIO-{ECHO_PIN}(Input), Trigger=GPIO-{TRIGGER_PIN}(Output)")
    print("Press Ctrl+C to exit\n")
    
    # Test pins first
    if not test_gpio_pins():
        print("\n[WARNING] Pin test shows potential issues!")
        response = input("Continue anyway? (y/n): ")
        if response.lower() != 'y':
            print("Exiting...")
            return
    
    print("\nStarting distance measurement...")
    print("-" * 60)
    
    try:
        # Suppress warnings for cleaner output
        warnings.filterwarnings('ignore')
        
        sensor = DistanceSensor(
            echo=ECHO_PIN,
            trigger=TRIGGER_PIN,
            max_distance=4,
            threshold_distance=0.3
        )
        
        error_count = 0
        success_count = 0
        
        while True:
            try:
                distance_m = sensor.distance
                distance_cm = distance_m * 100
                
                if distance_cm > 0 and distance_cm < 400:
                    success_count += 1
                    
                    # Status indicator
                    if distance_cm < 10:
                        status = "[VERY CLOSE]"
                    elif distance_cm < 30:
                        status = "[CLOSE]"
                    elif distance_cm < 100:
                        status = "[NORMAL]"
                    else:
                        status = "[FAR]"
                    
                    print(f"{status} Distance: {distance_cm:6.2f} cm ({distance_m:5.2f} m) | Success: {success_count}")
                    error_count = 0  # Reset error count on success
                else:
                    raise ValueError("Out of range")
                    
            except Exception as e:
                error_count += 1
                print(f"[ERROR] Measurement failed (#{error_count}): {e}")
                
                if error_count >= 5:
                    print("\n[CRITICAL] Too many consecutive errors!")
                    print("Please check:")
                    print("  1. HC-SR04 power connection (VCC to 5V, GND to GND)")
                    print("  2. Trigger wire to GPIO-70")
                    print("  3. Echo wire to GPIO-77")
                    print("  4. Sensor is not damaged")
                    break
            
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("\n\nProgram stopped by user")
    except Exception as e:
        print(f"\n[ERROR] {e}")
    finally:
        try:
            sensor.close()
            print("Cleanup completed")
        except:
            pass

def simple_test():
    """Simplified test without error handling"""
    from gpiozero.pins.lgpio import LGPIOFactory
    from gpiozero import Device, DistanceSensor
    
    Device.pin_factory = LGPIOFactory(chip=0)
    
    print("Simple HC-SR04 Test")
    print("GPIO-77(Echo), GPIO-70(Trigger)")
    print("-" * 40)
    
    sensor = DistanceSensor(echo=77, trigger=70)
    
    try:
        while True:
            distance = sensor.distance * 100
            print(f"Distance: {distance:6.2f} cm")
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nExit")
    finally:
        sensor.close()

def manual_test():
    """Manual GPIO control to test HC-SR04"""
    from gpiozero import DigitalInputDevice, DigitalOutputDevice
    from gpiozero.pins.lgpio import LGPIOFactory
    from gpiozero import Device
    
    Device.pin_factory = LGPIOFactory(chip=0)
    
    print("Manual HC-SR04 Test")
    print("=" * 60)
    
    trigger = DigitalOutputDevice(70)
    echo = DigitalInputDevice(77)
    
    try:
        print("Initial Echo state:", "HIGH" if echo.value else "LOW")
        
        for i in range(5):
            print(f"\nTest #{i+1}")
            
            # Send trigger pulse
            trigger.off()
            time.sleep(0.000002)
            trigger.on()
            time.sleep(0.00001)  # 10us pulse
            trigger.off()
            
            # Wait for echo
            timeout = time.time() + 0.5
            
            # Wait for echo to go HIGH
            while echo.value == 0 and time.time() < timeout:
                pass
            
            if time.time() >= timeout:
                print("  [FAIL] Echo never went HIGH - check connections!")
                continue
            
            pulse_start = time.time()
            
            # Wait for echo to go LOW
            while echo.value == 1 and time.time() < timeout:
                pass
            
            pulse_end = time.time()
            
            if time.time() >= timeout:
                print("  [FAIL] Echo stuck HIGH!")
                continue
            
            # Calculate distance
            pulse_duration = pulse_end - pulse_start
            distance = (pulse_duration * 34300) / 2
            
            print(f"  [OK] Distance: {distance:.2f} cm")
            
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\n\nStopped")
    finally:
        trigger.close()
        echo.close()

if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1:
        if sys.argv[1] == "simple":
            simple_test()
        elif sys.argv[1] == "manual":
            manual_test()
        elif sys.argv[1] == "test":
            test_gpio_pins()
        else:
            print("Usage: python3 hcsr04_test.py [simple|manual|test]")
    else:
        main()