import rclpy
from rclpy.node import Node
import re
from std_msgs.msg import String

class ChatbotNode(Node):
    def __init__(self):
        
        # inicia o nó 'chatbot_node'
        super().__init__('chatbot_node')
        self.publisher_ = self.create_publisher(String, 'chatbot_feedback', 10)
        self.get_logger().info("Chatbot iniciado. Digite 'sair' para encerrar.")

        # dicionário para entender as intenções com expressões regulares
        self.patterns = {
            "secretaria": r"(vá para|dirija-se a|me leve para) a secretaria",
            "laboratório": r"(vá para|dirija-se ao|me leve para) o laboratório",
            "biblioteca": r"(vá para|dirija-se a|me leve para) a biblioteca",
            "auditório": r"(vá para|dirija-se ao|me leve para) o auditório"
        }

        # dicionário para mapear destinos 
        self.actions = {
            "secretaria": lambda: "O robô está se dirigindo à secretaria.",
            "laboratório": lambda: "O robô está se dirigindo ao laboratório.",
            "biblioteca": lambda: "O robô está se dirigindo à biblioteca.",
            "auditório": lambda: "O robô está se dirigindo ao auditório."
        }

    # função para processar os comandos enviados pelo user
    def process_command(self, user_input):

        # procura se existe no dicionário o movimento
        for place, pattern in self.patterns.items():
            if re.search(pattern, user_input, re.IGNORECASE):

                # se encontra, executa movimento e retorna feedback
                action_feedback = self.actions.get(place, lambda: "Ação desconhecida.")()
                self.get_logger().info(f"Intenção: movimentar-se, Local: {place}\n{action_feedback}")
                
                # publica feedback no topico 'chatbot_feedback'
                msg = String()
                msg.data = f"Intenção: movimentar-se, Local: {place}\n{action_feedback}"
                self.publisher_.publish(msg)
                return
        
        # se nao achar nada, printa para o user
        self.get_logger().info("Desculpe, não entendi o comando.")

    #
    def run(self):
        while rclpy.ok():

            # lê a entrada do user
            user_input = input("Digite um comando para o robô (ou 'sair' para encerrar): ")
            
            # encerra o chatbot
            if user_input.lower() == 'sair':
                self.get_logger().info("Encerrando o chatbot.")
                break
            
            # processa o comando
            self.process_command(user_input)


def main(args=None):
    rclpy.init(args=args)

    # inica o nó
    node = ChatbotNode()
    try:
        node.run()
    finally:

        # encerra o nó
        node.destroy_node()
        rclpy.shutdown()
