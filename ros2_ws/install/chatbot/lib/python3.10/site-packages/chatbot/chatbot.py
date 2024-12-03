import rclpy
from rclpy.node import Node
import re
from std_msgs.msg import String

class ChatbotNode(Node):
    def __init__(self):
        # Inicia o nó 'chatbot_node'
        super().__init__('chatbot_node')
        self.publisher_ = self.create_publisher(String, 'chatbot_feedback', 10)
        self.get_logger().info("Chatbot iniciado. Digite 'sair' para encerrar.")

        # Regex para reconhecer apenas os destinos
        self.patterns = re.compile(
            r"\b(laboratório|secretaria|biblioteca|auditório)\b",
            re.IGNORECASE
        )

        # Dicionário para mapear destinos
        self.actions = {
            "secretaria": lambda: "O robô está se dirigindo à secretaria.",
            "laboratório": lambda: "O robô está se dirigindo ao laboratório.",
            "biblioteca": lambda: "O robô está se dirigindo à biblioteca.",
            "auditório": lambda: "O robô está se dirigindo ao auditório."
        }

    # Função para processar os comandos enviados pelo usuário
    def process_command(self, user_input):
        # Procura pelos destinos no comando do usuário
        match = self.patterns.search(user_input)
        if match:
            destino = match.group(0).lower()
            acao = self.actions.get(destino, lambda: "Ação desconhecida.")()
            
            # Publica feedback no tópico 'chatbot_feedback'
            self.get_logger().info(f"Local reconhecido: {destino}\n{acao}")
            msg = String()
            msg.data = f"Local reconhecido: {destino}\n{acao}"
            self.publisher_.publish(msg)
            return
        
        # Se não achar nada, informa ao usuário
        self.get_logger().info("Desculpe, não entendi o comando.")

    # Função para rodar o chatbot
    def run(self):
        while rclpy.ok():
            # Lê a entrada do usuário
            user_input = input("Digite um comando para o robô (ou 'sair' para encerrar): ")

            # Encerra o chatbot
            if user_input.lower() == 'sair':
                self.get_logger().info("Encerrando o chatbot.")
                break
            
            # Processa o comando
            self.process_command(user_input)


def main(args=None):
    rclpy.init(args=args)

    # Inicia o nó
    node = ChatbotNode()
    try:
        node.run()
    finally:
        # Encerra o nó
        node.destroy_node()
        rclpy.shutdown()
