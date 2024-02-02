import smbus
import time

# Define the I2C address of the ESP32
I2C_ADDRESS = 8

bus = smbus.SMBus(1)  # 1 indicates /dev/i2c-1

def send_pwm_values(pwm_values):
    try:
        # Send PWM values over I2C
        bus.write_i2c_block_data(I2C_ADDRESS, 0, pwm_values)
        
        print("PWM values sent successfully:", pwm_values)
    except Exception as e:
        print("Error:", str(e))

if __name__ == "__main__":
    try:
        # Test PWM values for each motor
        pwm_values = [1300, 1200, 1300, 1500, 1500, 1600]  
        send_pwm_values(pwm_values)
        
        time.sleep(2)

        # Stop all motors
        stop_values = [1500] * 6  
        send_pwm_values(stop_values)
    except KeyboardInterrupt:
        print("Keyboard interrupt detected. Exiting...")
    finally:
        # Stop all motors before exiting
        stop_values = [1500] * 6
        send_pwm_values(stop_values)
