#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import sys, select, termios, tty

# Guardamos la configuración original de tu terminal de Linux
settings = termios.tcgetattr(sys.stdin)

# Función mágica para leer 1 sola tecla pulsada sin esperar al "Enter"
def get_key():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class TeleopTeclado(Node):
    def __init__(self):
        super().__init__('teleop_teclado')
        # Creamos los dos megáfonos (Publishers)
        self.pub_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_estado = self.create_publisher(Int32, 'cmd_estado', 10)
        
        print("Nodo Teleop Iniciado.")
        print("Usa W/A/S/D para moverte. C para agacharte. J para saltar. Q para salir.")

    def run(self):
        msg_vel = Twist()
        msg_estado = Int32()

        while True:
            tecla = get_key() # El programa se queda "congelado" aquí hasta que toques una tecla
            
            if tecla == 'w':
                msg_vel.linear.x = 1.0
                msg_vel.angular.z = 0.0
                self.pub_vel.publish(msg_vel)
                print("Avanzando...")
                
            elif tecla == 's':
                msg_vel.linear.x = -1.0
                msg_vel.angular.z = 0.0
                self.pub_vel.publish(msg_vel)
                print("Retrocediendo...")
                
            elif tecla == 'a':
                msg_vel.angular.z = 1.0
                self.pub_vel.publish(msg_vel)
                print("Girando Izquierda...")
                
            elif tecla == 'd':
                msg_vel.angular.z = -1.0
                self.pub_vel.publish(msg_vel)
                print("Girando Derecha...")
                
            # PARADA CON ESPACIO
            elif tecla == ' ':
                msg_estado.data = 3
                self.pub_estado.publish(msg_estado)
                print("Comando: REPOSO enviado.")

            # ACCIONES DE ESTADO
            elif tecla == 'c':
                msg_estado.data = 1 # Inventamos que 1 es AGACHADO
                self.pub_estado.publish(msg_estado)
                print("Comando: AGACHARSE enviado.")
                
            elif tecla == 'j':
                msg_estado.data = 2 # Inventamos que 2 es SALTAR
                self.pub_estado.publish(msg_estado)
                print("Comando: SECUENCIA DE SALTO iniciada.")

            elif tecla == 'q':
                break # Salir del bucle

def main(args=None):
    rclpy.init(args=args)
    nodo = TeleopTeclado()
    
    try:
        nodo.run()
    except Exception as e:
        print(e)
    finally:
        # Al cerrar, enviamos un mensaje de parada por seguridad
        nodo.pub_vel.publish(Twist()) 
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()