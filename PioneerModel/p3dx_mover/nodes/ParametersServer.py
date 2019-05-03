class staticParameters:
    
    #velocidade linear
    linear_velocity = 0.6
    
    #minima velocidade linear
    min_linear_velocity = 0.2 #original 0.2 ateh 17.01.2019 12hs
    
    #velocidade maxima linear
    max_linear_velocity = 1.0 
    
    #variacao velocidade linear
    delta_linear_velocity = max_linear_velocity - min_linear_velocity
    
    #velocidade angular
    angular_velocity = 0.1
    
    #maxima velocidade angular
    max_angular_velocity = 1.0
    
    #minima velocidade angular
    min_angular_velocity = -max_angular_velocity
    
    #raio limite do cone do sensor
    sensor_cone_radius = 3.0 #original 3
    
    #maior raio descrito de aproximacao
    max_approach_radius = 3 * sensor_cone_radius
    
    #distancia maxima de leitura do cone do sensor
    max_sensor_dist = sensor_cone_radius # * 0.85
    
    #distancia minima do cone do sensor
    min_sensor_dist = 0.2
    
    #angulo do cone do sensor
    sensor_cone_angle = 270
    
    #angulo maximo de leitura do sensor
    max_sensor_angle = sensor_cone_angle / 2.0
     
    #angulo minimo de leitura do sensor
    min_sensor_angle = -sensor_cone_angle / 2.0
    
    
    #maximo de angulos obtidos pelo laser
    max_laser_reads = 270 #2 * sensor_cone_angle #original 270
    
    #angulo desejado ao target
    desired_angle_to_target = 90.0
    
    #diferenca entre o angulo desejado e o maior angulo do sensor
    delta_alpha = sensor_cone_angle / 2.0 - desired_angle_to_target
     
    
    #angulo maximo de controle angular
    max_ctrl_angle = sensor_cone_angle / 2.0
    
    #angulo minimo de controle angular
    min_ctrl_angle = desired_angle_to_target - delta_alpha
    
    #peso do coeficiente angular ao target
    proportional_angular_weight = 0.85
    
    #raio de identificacao do target
    target_radius_id = 0.10 # (0.085 ... 0.115)
    
    #raio de identificacao do Robo
    robot_radius_id = 0.15 # (0.1275 ... 0.1725)
    
    #raio desejado de circunavegacao
    desired_radius = 1.6 #meters original 1.6
    
    #distancia desejada entre robots
    desired_distance_to_robot = 1.2
    
    #distancia minima entre robots
    min_distance_to_robot = 0.85
    
    #distancia entre as rodas do robot #pioneer3dx.gazebo
    wheel_separation = 0.39 
    
    #diametro das rodas do robot #pioneer3dx.gazebo
    wheel_diameter = 0.15
    
    #diferenca entre a maxima velocidade angular e a velocidade angular media para o raio desejado
    delta_angular = max_angular_velocity - linear_velocity / desired_radius
    
    
    #roleranca para a identificacao enter robo e target - percentual decimal
    #radius_tolerance = 0.25
    
    #tipos de objetos identificaveis
    id_types = ["void", "target", "robot", "alien"]
    
    #mensagens de Status
    status_msg = ["Searching","Avoiding","Following","Circumnavegating","Escorting"]

    #constante de tempo para boltzmann
    boltzmann_time_constant = 0.05
