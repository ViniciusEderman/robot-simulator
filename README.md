# Relatório Técnico: Controle Simulado do Robô R2D2 com PyBullet e MQTT

## Descrição do Projeto

Este projeto propõe a simulação de um robô móvel do tipo R2D2 utilizando o ambiente de física PyBullet, controlado remotamente via mensagens MQTT. O sistema é capaz de receber comandos de movimento, executar ações correspondentes no simulador e detectar obstáculos para evitar colisões.

## Objetivos

- Desenvolver um ambiente simulado com física realista para um robô R2D2.
- Implementar controle remoto via protocolo MQTT para enviar comandos ao robô.
- Detectar obstáculos à frente do robô usando ray casting para prevenção de colisões.
- Prover uma interface gráfica simples para visualização da simulação.

## Abordagem Adotada

- Uso da biblioteca PyBullet para simulação física e renderização do ambiente e robô.
- Integração com MQTT para comunicação assíncrona, permitindo o controle remoto do robô.
- Implementação de lógica para movimentação baseada em comandos recebidos e parada automática em presença de obstáculos detectados à frente.
- Obstáculos estáticos posicionados em pontos estratégicos para teste da detecção e reação do robô.

## Arquitetura do Sistema

- **Simulador PyBullet:** Responsável pela física do robô, ambiente, e detecção de colisões.
- **Cliente MQTT:** Subscrito ao tópico `r2d2/command`, recebe comandos JSON que indicam ações para o robô.
- **Controlador R2D2:** Classe central que gerencia o estado do robô, execução dos comandos, detecção de obstáculos, e interface com o simulador e MQTT.

Fluxo resumido:
1. Inicia a simulação e carrega o modelo do robô e obstáculos.
2. Conecta-se ao broker MQTT e aguarda comandos.
3. Ao receber um comando válido, atualiza o estado do robô.
4. Executa o movimento correspondente no PyBullet, verificando obstáculos.
5. Repetição contínua até interrupção.

## Tecnologias Utilizadas

- **Python 3.x**
- **PyBullet:** Simulação física e renderização 3D.
- **paho-mqtt:** Biblioteca MQTT para comunicação assíncrona.
- **MQTT Broker ( Mosquitto):** Plataforma para troca de mensagens.

## Principais Decisões de Implementação

- **Uso do PyBullet GUI:** Para visualização em tempo real da simulação e debug.
- **Ray Casting para Obstáculo:** Técnica eficiente para detecção precisa de obstáculos à frente.
- **Separação das rodas esquerda e direita:** Permite controle diferencial para virar e ajustar movimentos.
- **Velocidades fixas para movimento e giro:** Simplicidade na lógica de controle e testes.
- **Mensagens MQTT JSON:** Facilita extensibilidade dos comandos futuros.
- **Loop principal com tratamento de KeyboardInterrupt:** Para encerramento limpo da simulação e conexão MQTT.

## Resultados Obtidos

- Simulação funcional do robô R2D2 que responde a comandos MQTT em tempo real.
- Movimentação básica implementada: avançar, retroceder, virar à esquerda, virar à direita e parar.
- Detecção de obstáculos eficaz, com parada automática ao detectar colisões à frente.
- Ambiente simulado contendo obstáculos posicionados em pontos fixos.
- Interface gráfica permite visualização do robô e das linhas de ray casting, com feedback visual sobre detecção (linha verde para livre e vermelha para obstáculo).
