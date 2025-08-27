from machine import UART,Pin
import time
import struct
import sys
import select

class BureiteEncoder:
    def _init_(self, uart_id=0, baudrate=9600, tx=0, rx=1, re_pin=2, de_pin=3, resolution=16384, slave_address=0x01):
        self.uart = UART(uart_id, baudrate=baudrate, tx=Pin(tx), rx=Pin(rx))
        self.re_pin = Pin(re_pin, Pin.OUT)
        self.de_pin = Pin(de_pin, Pin.OUT)
        self.resolution = resolution
        self.slave_address = slave_address

        self.re_pin.value(0)
        self.de_pin.value(0)

        time.sleep(1) 

    def calc_crc(self, data):
        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

    def enviar_comando_lectura(self):
        cmd = bytearray([self.slave_address, 0x03, 0x00, 0x00, 0x00, 0x01])  
        crc = self.calc_crc(cmd)
        cmd += struct.pack('<H', crc)

        while self.uart.any():
            self.uart.read()

        self.re_pin.value(1)
        self.de_pin.value(1)
        time.sleep(0.01)

        self.uart.write(cmd)
        time.sleep(0.01)

        self.re_pin.value(0)
        self.de_pin.value(0)

    def leer_respuesta(self):
        time.sleep(0.01)
        if self.uart.any():
            respuesta = self.uart.read()
            idx = respuesta.find(b'\x01\x03\x02') if respuesta else -1
            if idx != -1 and len(respuesta) >= idx + 7:
                datos = respuesta[idx:idx + 7]
                cuerpo = datos[:-2]
                crc_recibido = struct.unpack('<H', datos[-2:])[0]
                crc_calculado = self.calc_crc(cuerpo)

                if crc_recibido == crc_calculado:
                    valor = (datos[3] << 8) | datos[4]
                    angle = valor * 360.0 / self.resolution
                    return angle
                else:
                    print("CRC inválido")
            else:
                #print("No se encontró una respuesta válida en el buffer")
                return None
        else:
            print("No se recibió respuesta")
        return None

    def leer_angulo(self):
        self.enviar_comando_lectura()
        return self.leer_respuesta()
    

class ODriveUART:
    def _init_(self, uart_id=1, baudrate=115200, tx=8, rx=9):
        self.uart = UART(uart_id, baudrate=baudrate, tx=Pin(tx), rx=Pin(rx))
        time.sleep(0.1)

    def send_cmd(self, cmd):
        # Asegura que el comando se envíe con newline
        full_cmd = cmd.encode('utf-8') + b'\n'
        self.uart.write(full_cmd)
        time.sleep(0.1)

    def set_velocity(self, velocity):
        self.send_cmd(f'w axis0.controller.input_vel {velocity}')

    def get_vbus_voltage(self):
        self.send_cmd('r vbus_voltage')


    def stop_motor(self):
        self.set_velocity(0)

class ODriveUARTProbe:
    def _init_(self, uart_id=1, baudrate=115200, tx=4, rx=5, lf=b'\n'):
        self.lf = lf
        self.u = UART(uart_id, baudrate=baudrate, tx=Pin(tx), rx=Pin(rx))
        time.sleep(0.2)

    def txrx(self, line, timeout_ms=400):
        # limpia el buffer rx antes
        while self.u.any():
            self.u.read()
        self.u.write(line + self.lf)
        start = time.ticks_ms()
        buf = b""
        # espera algo de respuesta
        while time.ticks_diff(time.ticks_ms(), start) < timeout_ms:
            if self.u.any():
                buf += self.u.read()
                if b'\n' in buf or b'\r' in buf:
                    break
            time.sleep(0.005)
        return buf


class Shoulder:
    def _init_(self, kP=0.5, v_max=10.0, deadband=0.5):
        self.odrive = ODriveUART()
        self.encoder = BureiteEncoder()

        # metas
        self.goal_pos = None     # [deg]
        self.goal_vel = None     # [deg/s]
        self.vel = None          # [deg/s] modo manual

        # estado
        self.angulo_actual = None

        # control
        self.kP = kP
        self.v_max = v_max
        self.deadband = deadband

    def go_pos(self, goal_pos):
        self.goal_pos = float(goal_pos)
        self.goal_vel = None  # salir de modo velocidad
        self.vel = None

    def set_vel(self, vel):
        self.vel = float(vel)
        self.goal_pos = None
        self.goal_vel = None

    def set_goal_vel(self, goal_vel):
        self.goal_vel = float(goal_vel)
        self.goal_pos = None
        self.vel = None
        self.odrive.set_velocity(self.goal_vel)

    def stop(self):
        self.goal_pos = None
        self.goal_vel = None
        self.vel = None
        self.odrive.stop_motor()

    # ---------- utilidades de ángulo ----------
    @staticmethod
    def _wrap_deg_pm180(a):
        """Envuelve a (-180, 180] en grados."""
        return ((a + 180.0) % 360.0) - 180.0

    @staticmethod
    def _shortest_angular_error_deg(goal, actual):
        """Error con signo por el camino más corto [deg]."""
        return Shoulder._wrap_deg_pm180(goal - actual)

    # ---------- lazo principal ----------
    def update(self):
        self.angulo_actual = self.encoder.leer_angulo()  # [deg], idealmente 0..360 (pero puede ser cualquier real)
        if self.angulo_actual is None:
            return

        # --- Control a posición (P con camino corto) ---
        if self.goal_pos is not None:
            error = Shoulder._shortest_angular_error_deg(self.goal_pos, self.angulo_actual)

            # zona muerta
            if abs(error) < self.deadband:
                self.odrive.set_velocity(0.0)
                return

            v_cmd = self.kP * error

            # saturación
            if v_cmd > self.v_max:
                v_cmd = self.v_max
            elif v_cmd < -self.v_max:
                v_cmd = -self.v_max

            print(f"Act={self.angulo_actual:.2f}°, Goal={self.goal_pos:.2f}°, "
                  f"Err(short)={error:.2f}°, Vcmd={v_cmd:.2f} deg/s")

            # Si tu ODrive espera rad/s, convierte:
            # from math import pi
            # v_cmd = v_cmd * (pi/180.0)

            self.odrive.set_velocity(v_cmd)
            return

        # --- Control a velocidad “abierta” objetivo ---
        if self.goal_vel is not None:
            # ya se envió en set_goal_vel; podrías re‑enviar aquí si quieres mantenerlo
            # self.odrive.set_velocity(self.goal_vel)
            return

        # --- Velocidad manual “abierta” ---
        if self.vel is not None:
            self.odrive.set_velocity(self.vel)



shoulder = Shoulder()
shoulder.go_pos(int(input("Enter position: ")))

while True:
    shoulder.update()