import time
import pigpio

GPIO = 18  # pin segnale

pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("pigpiod non è attivo. Avvia: sudo systemctl start pigpiod")

def angle_to_pulsewidth(angle: float) -> int:
    # tipico servo: 0°=500us, 180°=2500us (a volte 1000-2000us)
    angle = max(0.0, min(180.0, angle))
    return int(500 + (angle / 180.0) * 2000)

try:
    for a in [0, 45, 90, 135, 180, 90]:
        pw = angle_to_pulsewidth(a)
        pi.set_servo_pulsewidth(GPIO, pw)
        time.sleep(0.7)

    # ferma il servo (niente impulsi)
    pi.set_servo_pulsewidth(GPIO, 0)

finally:
    pi.stop()