# Chatbot para Controle de Robôs no ROS 2

Este projeto implementa um nó ROS 2 que funciona como um chatbot para interagir com um robô fictício. O chatbot interpreta comandos de linguagem natural, mapeia intenções para ações e fornece feedback ao usuário. Ele é desenvolvido em Python e utiliza o framework ROS 2 Humble.

## Estrutura do Projeto
```bash
ros2_ws/ 
├── src/ 
│ └── chatbot/ 
│ ├── chatbot/ 
│ │ ├── init.py 
│ │ └── chatbot.py 
│ ├── package.xml 
│ ├── setup.py 
├── build/ 
├── install/ 
├── log/
```
---

## Funcionalidades

- **Compreensão de linguagem natural**: Usa expressões regulares para interpretar comandos.
- **Mapeamento de intenções**: Tradução de intenções do usuário em ações para o robô.
- **Feedback ao usuário**: O chatbot informa ao usuário quando a ação foi compreendida ou se o comando não foi reconhecido.

### Exemplos de Comandos

- `Vá para a secretaria`
- `Dirija-se ao laboratório`
- `Me leve para a biblioteca`

### Respostas Esperadas

- **Comando válido**:
Intenção: movimentar-se, Local: biblioteca O robô está se dirigindo à biblioteca.

- **Comando inválido**:
Desculpe, não entendi o comando.


## Pré-requisitos

- **ROS 2 Humble**
- **Python 3**
- Dependências do ROS 2:
```bash
sudo apt install python3-colcon-common-extensions
sudo apt install ros-humble-rclpy ros-humble-std-msgs
```

## Configuração e Execução

1) Clone o repositório 
No seu terminal, insira o seguinte bloco de código
```bash
git clone https://github.com/gabriellemitoso793/modulo8-p2.git
```
2) Compile o Workspace
```bash
cd modulo8-p2/ros2_ws
colcon build
source install/setup.bash
```
3) Inicie o chatbot
```bash
ros2 run chatbot chatbot
```
4) Teste os comandos
- Digite comandos como Vá para a biblioteca ou Me leve para a biblioteca.
- O feedback será exibido no terminal.

## Estrutura do Código
- process_command: Interpreta o comando do usuário usando expressões regulares.
- run: Lê comandos do terminal e processa as intenções do usuário.
- main: Inicializa o nó ROS 2 e executa o chatbot.
