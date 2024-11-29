import rclpy
from rclpy.node import Node
import re
from std_msgs.msg import String

class ChatbotNode(Node):
    def __init__(self):
        super().__init__('chatbot_node')
        self.publisher_ = self.create_publisher(String, 'chatbot_feedback', 10)
        self.get_logger().info("Chatbot iniciado. Digite 'sair' para encerrar.")

        self.patterns = {
            "secretaria": r"(vá para|dirija-se a|me leve para) a secretaria",
            "laboratório": r"(vá para|dirija-se ao|me leve para) o laboratório",
            "biblioteca": r"(vá para|dirija-se a|me leve para) a biblioteca",
            "auditório": r"(vá para|dirija-se ao|me leve para) o auditório"
        }

        self.actions = {
            "secretaria": lambda: "O robô está se dirigindo à secretaria.",
            "laboratório": lambda: "O robô está se dirigindo ao laboratório.",
            "biblioteca": lambda: "O robô está se dirigindo à biblioteca.",
            "auditório": lambda: "O robô está se dirigindo ao auditório."
        }

    def process_command(self, user_input):
        for place, pattern in self.patterns.items():
            if re.search(pattern, user_input, re.IGNORECASE):
                action_feedback = self.actions.get(place, lambda: "Ação desconhecida.")()
                self.get_logger().info(f"Intenção: movimentar-se, Local: {place}\n{action_feedback}")
                msg = String()
                msg.data = f"Intenção: movimentar-se, Local: {place}\n{action_feedback}"
                self.publisher_.publish(msg)
                return
        self.get_logger().info("Desculpe, não entendi o comando.")

    def run(self):
        while rclpy.ok():
            user_input = input("Digite um comando para o robô (ou 'sair' para encerrar): ")
            if user_input.lower() == 'sair':
                self.get_logger().info("Encerrando o chatbot.")
                break
            self.process_command(user_input)


def main(args=None):
    rclpy.init(args=args)
    node = ChatbotNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
