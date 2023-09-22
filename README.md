# Controle de Robô com ROS e Sensores Laser

Este é um exemplo de código Python usado para controlar um robô com o ROS (Robot Operating System) e sensores laser. O código foi projetado para um robô que pode mover a cabeça, navegar em um ambiente e tomar decisões com base nas leituras do sensor laser.

## Requisitos

- ROS (Robot Operating System): O código foi projetado para funcionar em um ambiente ROS.
- Python: O código é escrito em Python e requer um ambiente Python funcional.
- Pacotes ROS: Certifique-se de ter os pacotes ROS necessários instalados para a funcionalidade correta.

## Funcionalidades

O código oferece as seguintes funcionalidades:

- Controle da cabeça do robô: O robô pode mover a cabeça em direções específicas (esquerda, direita e centro) usando um tópico ROS.
- Leitura da odometria: O código subscreve o tópico de odometria do robô e extrai a orientação atual.
- Leitura do sensor laser: O código subscreve o tópico do sensor laser e obtém leituras para o lado esquerdo, direito e centro do robô.
- Movimento reto: O robô pode mover-se em linha reta com uma velocidade linear específica.
- Parada: O robô pode parar de se mover.
- Girar: O robô pode girar para a esquerda ou direita com uma velocidade angular específica.

## Uso

Para usar este código:

1. Certifique-se de que o ambiente ROS esteja configurado e funcionando corretamente.
2. Clone este repositório ou copie o código em um arquivo Python.
3. Configure os tópicos ROS apropriados para o controle da cabeça, odometria, sensor laser e movimento da base, conforme necessário, nas variáveis de classe.

```python
# Exemplo de configuração dos tópicos ROS
self.head_pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=10)
self.sub_odo = rospy.Subscriber('/mobile_base_controller/odom', Odometry, self.callback_odometria, queue_size=1)
self.laser_sub = rospy.Subscriber('/scan_raw', LaserScan, self.callback_laser)
self.pub_vel = rospy.Publisher('/nav_vel', Twist, queue_size=1)
```

Execute o código Python em um ambiente ROS.
  * rosrun <nome_do_pacote> <nome_do_script>.py

O código deve começar a controlar o robô de acordo com a lógica definida no método decision.

## Contribuições
Este é um código de exemplo e pode ser adaptado para atender às necessidades específicas do seu robô e aplicação. Contribuições e melhorias são bem-vindas.
